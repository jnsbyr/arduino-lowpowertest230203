/*****************************************************************************
 *
 * Low Power Test for SAMD21 and Teensy LC (both with ARM Cortex M0+)
 *
 * file:     LowPowerTest230203.ino
 * encoding: UTF-8
 * created:  03.02.2023
 *
* ****************************************************************************
 *
 * Copyright (C) 2023 Jens B.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 *****************************************************************************/

#ifdef ARDUINO_SAMD_ZERO

// Arduino includes, LGPL
#include <Arduino.h>

// comment in to test external interrupt
//#define TEST_EIC
// comment in to test timer interrupt + sleep modes with 2 s IDLE2 every 5 s STANDBY, otherwise slow blink for 50 ms every 2 min
//#define TEST_SLEEP
// comment in to use Arduino loop(), otherwise slow blink for 50 ms every 2 min
//#define TEST_LOOP
// comment in in combination with TEST_LOOP to execute blink(2000), execute while(1)
//#define TEST_BLINK

/*
   To be able to lower F_CPU of a Seed Studio XIAO SAMD21 via Arduino's board.txt
   the Arduino core file startup.c needs to be patched, see README.md for details.

   Note that USB communication will no longer work, because it requires an 48 MHz clock.

   The modified Arduino core and selected Arduino libraries taken together result in
   the following clock configuration for the SAMD21:

   OSC8M     -> GCLK0 (1 MHz)  -> (POR)
   DFLL48M   -> GCLK0 (F_CPU)  -> CPU, AHB, APBx, PM, ADC (F_CPU/32), DAC (F_CPU) (startup.c, wiring.c)
                                  USB (USBCore.cpp)
                                  TC (wiring_analog.c, PWM)

   XOSC32K   -> GCLK1 (32 kHz) -> GCLK Mux 0 -> DFLL48M (startup.c)

   OSCULP32K -> GCLK2 (32 kHz) -> WDT (POR)
   XOSC32K   -> GCLK2 (1 kHz)  -> RTC (RTCZero.cpp)

   OSC8M     -> GCLK3 (8 MHz)  -> unused (startup.c)

   OSCULP32K -> GCLK6 (32 kHz) -> EIC (ArduinoLowPower.cpp, RUNSTDBY)


   Seed XIAO SAM D21 power consumption

    - Vcc 3.0 V
    - green LED cut, yellow LED off
    - LDO pin 3 disconnected
    - USB disconnected
    - OSCULP32K for RTC
    - RTC and PORT enabled

     48 MHz : 12.97 mA - USB connected, yellow LED on, bootloader upload mode

     48 MHz : 12.23 mA - DFLL48M, Arduino core default startup, blink(2000)
     48 MHz :  8.14 mA - DFLL48M, Arduino core default startup, while(1)
     48 MHz :  5.72 mA - DFLL48M, PM tuning, bus dividers 1/1/1, while(1)
     48 MHz :  4.83 mA - DFLL48M, PM tuning, bus dividers 4/128/128, while(1)
     48 MHz :  4.78 mA - DFLL48M, PM+GCLKGEN tuning, bus dividers 4/128/128, while(1)
     48 MHz :  3.12 mA - DFLL48M, PM+GCLKGEN+GCLK tuning, bus dividers 4/128/128, while(1)
     48 MHz :  3.02 mA - DFLL48M, PM+GCLKGEN+GCLK+NVM tuning, bus dividers 4/128/128, while(1)
     32 MHz :  2.12 mA - OSC8M,   PM+GCLKGEN+GCLK+NVM tuning, bus dividers 4/128/128, while(1)
      6 MHz :  1.10 mA - DFLL48M, PM+GCLKGEN+GCLK+NVM tuning, bus dividers 1/1/1, while(1)
      6 MHz :  0.98 mA - DFLL48M, PM+GCLKGEN+GCLK+NVM tuning, bus dividers 4/128/128, while(1)
      8 MHz :  0.76 mA - OSC8M,   PM+GCLKGEN+GCLK+NVM tuning, bus dividers 1/1/1, while(1)
      8 MHz :  0.61 mA - OSC8M,   PM+GCLKGEN+GCLK+NVM tuning, bus dividers 4/128/128, while(1)
      4 MHz :  0.41 mA - OSC8M,   PM+GCLKGEN+GCLK+NVM tuning, bus dividers 4/128/128, while(1)

      -> 49 µA/MHz + 0.67 mA with DFLL48M, bus dividers 4/128/128
      -> 61 µA/MHz + 0.17 mA with OSC8M, bus dividers 4/128/128

      8 MHz :  0.31 mA - IDLE2, SLEEPONEXIT, PM+GCLKGEN+GCLK+NVM tuning, bus dividers 1/1/1
      8 MHz :  0.29 mA - IDLE2, SLEEPONEXIT, PM+GCLKGEN+GCLK+NVM tuning, bus dividers 4/128/128

      0 MHz :  16.1 µA - STANDBY, LDO pin 3 connected
      0 MHz :   2.0 µA - STANDBY

 */


void blink(byte pin, int on, int off=-1, byte count=1)
{
  for (byte i=0; i<count; i++)
  {
    digitalWrite(pin, LOW);
    if (on > 0) delay(on);
    digitalWrite(pin, HIGH);
    if (off < 0) delay(on);
    else if (off > 0) delay(off);
  }
  if (count > 0 && off > 0) delay(off);
}

void usb_disable()
{
#if defined(USBCON)
  //USBDevice.standby(); // no discernible effect
  USBDevice.detach();
  USBDevice.end();
#endif
}

/******************************************************************************
 *
 * SAMD21 utilities
 *
 ******************************************************************************/

/**
 * SAMD21 ADC access (CMSIS based implementation)
 *
 * note: unused, not tested
 *
 * Background:
 *
 * Arduino core does not provide solution to read:
 * - core voltages
 * - core temperature
 */
class Analog2DigitalConverter
{
public:
  /**
   * analog read (12 bits, max. 1 V)
   *
   * @param muxPos ADC_INPUTCTRL_MUXPOS_XXX
   * @return ADC result scaled to Vref [V]
   *
   * @see wiring_analog.c analogReadResolution(int)
   * @see wiring_analog.c analogReference(eAnalogReference)
   * @see wiring_analog.c analogRead(uint32_t)
   */
  static float read(uint32_t muxPos)
  {
    PM->APBCMASK.bit.ADC_ = 1;

    if (muxPos == ADC_INPUTCTRL_MUXPOS_TEMP)
    {
      SYSCTRL->VREF.bit.TSEN = 1;
    }

    ADC->CTRLB.bit.RESSEL = ADC_CTRLB_RESSEL_12BIT_Val;
    while (ADC->STATUS.bit.SYNCBUSY);

    ADC->REFCTRL.bit.REFSEL = ADC_REFCTRL_REFSEL_INT1V_Val;

    ADC->INPUTCTRL.bit.MUXPOS = muxPos;
    ADC->INPUTCTRL.bit.GAIN = ADC_INPUTCTRL_GAIN_1X_Val;
    while (ADC->STATUS.bit.SYNCBUSY);

    ADC->CTRLA.bit.ENABLE = 1;
    while (ADC->STATUS.bit.SYNCBUSY);

    // start 1st conversion and discard
    ADC->SWTRIG.bit.START = 1;
    ADC->INTFLAG.reg = ADC_INTFLAG_RESRDY;
    while (ADC->STATUS.bit.SYNCBUSY);

    // start 2nd conversion and read
    ADC->SWTRIG.bit.START = 1;
    while (!ADC->INTFLAG.bit.RESRDY);

    uint16_t result = ADC->RESULT.reg;
    while (ADC->STATUS.bit.SYNCBUSY);

    ADC->CTRLA.bit.ENABLE = 0;
    while (ADC->STATUS.bit.SYNCBUSY);

    if (muxPos == ADC_INPUTCTRL_MUXPOS_TEMP)
    {
      SYSCTRL->VREF.bit.TSEN = 0;
    }

    PM->APBCMASK.bit.ADC_ = 0;

    if (ADC->INPUTCTRL.bit.MUXPOS & ADC_INPUTCTRL_MUXPOS_SCALEDCOREVCC || ADC->INPUTCTRL.bit.MUXPOS & ADC_INPUTCTRL_MUXPOS_SCALEDIOVCC)
    {
      result = result<<2;
    }

    return ((float)result)/(2<<12);
  }
};


/**
 * SAMD21 generic clock and sleep mode utilities (CMSIS based implementation)
 *
 * Background:
 *
 * Arduino core does not provide public functions to handle:
 * - generic clock generators and generic clocks
 * - SysTick
 * and the ArduinoLowPowerClass requires an USBDevice.
 */
class System
{
public:
  enum SleepMode
  {
    IDLE0 = 0,
    IDLE1 = 1,
    IDLE2 = 2,
    STANDBY = 3
  };

public:
  /**
   * setup a generic clock generator with given divider
   * with OSCULP32K as source and RUNSTDBY option
   */
  static void setupClockGenOSCULP32K(byte genId, int8_t div = -1)
  {
    // setup GCLK divider
    GCLK->GENDIV.reg = GCLK_GENDIV_ID(genId)|
                       (div>=0? GCLK_GENDIV_DIV(div) : 0);
    while (GCLK->STATUS.reg & GCLK_STATUS_SYNCBUSY);

    // set OSCULP32K as source for GCLK
    GCLK->GENCTRL.reg = GCLK_GENCTRL_ID(genId) |
                        GCLK_GENCTRL_GENEN |
                        GCLK_GENCTRL_SRC_OSCULP32K |
                        (div>=0? GCLK_GENCTRL_DIVSEL : 0) |
                        (div>=0? GCLK_GENCTRL_IDC : 0) |
                        GCLK_GENCTRL_RUNSTDBY;
    while (GCLK->STATUS.reg & GCLK_STATUS_SYNCBUSY);
  }

  /**
   * setup a generic clock with given divider
   * with OSC8M as source
   * @param div divide OSC frequency by 2^(div + 1), -1 disables divider
   */
  static void setupClockGenOSC8M(byte genId, int8_t div = -1)
  {
    // ensure OSC8M is enabled and running at 8 MHz
    SYSCTRL->OSC8M.reg &= ~SYSCTRL_OSC8M_PRESC_Msk;
    SYSCTRL->OSC8M.reg &= ~SYSCTRL_OSC8M_RUNSTDBY;
    SYSCTRL->OSC8M.reg |= SYSCTRL_OSC8M_ENABLE;

    // setup GCLK divider
    GCLK->GENDIV.reg = GCLK_GENDIV_ID(genId) |
                       (div>=0? GCLK_GENDIV_DIV(div) : 0);
    while (GCLK->STATUS.reg & GCLK_STATUS_SYNCBUSY);

    // set OSC8M as source for GCLK
    GCLK->GENCTRL.reg = GCLK_GENCTRL_ID(genId) |
                        GCLK_GENCTRL_GENEN |
                        GCLK_GENCTRL_SRC_OSC8M |
                        (div>=0? GCLK_GENCTRL_DIVSEL : 0) |
                        (div>=0? GCLK_GENCTRL_IDC : 0);
    while (GCLK->STATUS.reg & GCLK_STATUS_SYNCBUSY);
  }

  static void disableClockGen(byte genId)
  {
    GCLK->GENCTRL.reg = GCLK_GENCTRL_ID(genId);
    while (GCLK->STATUS.reg & GCLK_STATUS_SYNCBUSY);
  }

  /**
   * enable generic clock by connecting it to a generic clock generator
   */
  static void enableClock(byte clkId, byte genId)
  {
    // set GCLK as source for TC
    GCLK->CLKCTRL.reg = (uint16_t)GCLK_CLKCTRL_ID(clkId);
    while (GCLK->STATUS.bit.SYNCBUSY);
    GCLK->CLKCTRL.reg = (uint16_t)(GCLK_CLKCTRL_ID(clkId) |
                                   GCLK_CLKCTRL_CLKEN |
                                   GCLK_CLKCTRL_GEN(genId));
    while (GCLK->STATUS.bit.SYNCBUSY);

    // enable module and B/C bus
    switch (clkId)
    {
      case GCM_RTC:
        PM->APBAMASK.reg |= PM_APBAMASK_RTC;
        break;

      case GCM_EIC:
        PM->APBAMASK.reg |= PM_APBAMASK_EIC;
        PM->AHBMASK.reg |= PM_AHBMASK_HPB1;
        break;

      case GCM_TCC2_TC3:
      case GCM_TC4_TC5:
        PM->AHBMASK.reg |= PM_AHBMASK_HPB2;
        break;
    }
  }

  /**
   * disable generic clock by connecting it to disabled generic clock generator 8
   */
  static void disableClock(byte clkId)
  {
    GCLK->CLKCTRL.reg = (uint16_t)GCLK_CLKCTRL_ID(clkId);
    while (GCLK->STATUS.bit.SYNCBUSY);
    GCLK->CLKCTRL.reg = (uint16_t)(GCLK_CLKCTRL_ID(clkId) |
                                   GCLK_CLKCTRL_GEN(GCLK_GEN_NUM_MSB));
    while (GCLK->STATUS.bit.SYNCBUSY);
  }

  /**
   * - disable clocks of non essential modules
   * - adjust NVM read wait states
   */
  static void tune()
  {
    // read core voltage
    //float vCore = readADC(ADC_INPUTCTRL_MUXPOS_SCALEDCOREVCC);

    // essential clocks: AHB: NVMCTRL, HPB0, APBA: SYSCTRL, PM, GCLK
    // additional clocks: APBA: RTC
    // disable all other clocks that are enabled at reset
    // @see product datasheet table 16-1 - Peripheral Clock Default State
    // @see product datasheet chapter 37.7 - Power Consumption
    PM->AHBMASK.reg = PM_AHBMASK_NVMCTRL | PM_AHBMASK_HPB0;
    PM->APBAMASK.reg = PM_APBAMASK_SYSCTRL | PM_APBAMASK_PM | PM_APBAMASK_GCLK | PM_APBAMASK_RTC;
    PM->APBBMASK.reg = 0;
    PM->APBCMASK.reg = 0;

    // set bus clocks
    if (SystemCoreClock <= 12000000L)
    {
      // lower CPU clock frequency, bus clocks undivided (to reduce sync overhead)
      PM->APBASEL.reg = PM_APBASEL_APBADIV_DIV1;
      PM->APBBSEL.reg = PM_APBBSEL_APBBDIV_DIV1;
      PM->APBCSEL.reg = PM_APBCSEL_APBCDIV_DIV1;
    }
    else
    {
      // higher CPU clock frequency, reduce bus clocks (to reduce consumption)
      PM->APBASEL.reg = PM_APBASEL_APBADIV_DIV4;
      PM->APBBSEL.reg = PM_APBBSEL_APBBDIV_DIV128;
      PM->APBCSEL.reg = PM_APBCSEL_APBCDIV_DIV128;
    }

    // optimize use of oscillators
    if (SystemCoreClock == 8000000L || SystemCoreClock == 4000000L || SystemCoreClock == 2000000L || SystemCoreClock == 1000000L)
    {
      // CPU clock frequency less equal 8 MHz or diveided by the power of 2, switch GCLKGEN0 from XOSC32K/DFLL48M to OSC8M oscillator
      setupClockGenOSC8M(0, 8000000L/SystemCoreClock - 2);
      // disable GCLKGEN1 and DFLL48M
      disableClockGen(1);
      disableClock(GCLK_CLKCTRL_ID_DFLL48_Val);
      SYSCTRL->DFLLCTRL.bit.ENABLE = 0;
      // disable XOSC32K
      SYSCTRL->XOSC32K.bit.ENABLE = 0;
    }
    else
    {
      // although unsed, do not disable OSC8M (would increase power consumption)
    }

    // disable GCLKGEN2 and higher
    for (size_t genId = 2; genId < GCLK_GEN_NUM; genId++)
    {
      disableClockGen(genId);
    }

    // disable most GCLKs (will reduce supply current by ~50 %)
    for (size_t clkId = 1; clkId < GCLK_NUM; clkId++)
    {
      if (clkId != GCLK_CLKCTRL_ID_RTC_Val)
      {
        disableClock(clkId);
      }
    }

    // reduce NVM wait states (or Vdd >= 2.7 V, F_CPU <= 24000000L)
    // @see product datasheet chapter 37.12 - NVM Characteristics
    if (SystemCoreClock <= 14000000L)
    {
      NVMCTRL->CTRLB.bit.RWS = 0;
    }
  }

  static void enableSysTick()
  {
    SysTick->CTRL |= SysTick_CTRL_TICKINT_Msk;
  }

  static void disableSysTick()
  {
    SysTick->CTRL &= ~SysTick_CTRL_TICKINT_Msk;
  }

  /**
   * @param mode 0..2 idle, 3 deep sleep
   */
  static void setSleepMode(SleepMode mode)
  {
    if (mode < STANDBY)
    {
      SCB->SCR &= ~SCB_SCR_SLEEPDEEP_Msk;
      PM->SLEEP.reg = mode;
    }
    else
    {
      SCB->SCR |= SCB_SCR_SLEEPDEEP_Msk;
    }

    // DSB required when called as last operation in ISR
    __DSB();
  }

  /**
   * put MCU to sleep when returning from ISR
   */
  static void setSleepOnExitISR(bool on)
  {
    if (on)
    {
      SCB->SCR |= SCB_SCR_SLEEPONEXIT_Msk;
    }
    else
    {
      SCB->SCR &= ~SCB_SCR_SLEEPONEXIT_Msk;
    }
  }

  /**
   * put MCU to sleep until wakeup interrupt
   *
   * @see ArduinoLowPower.cpp ArduinoLowPowerClass::idle()
   */
  static void sleep(SleepMode mode)
  {
    // disable systick interrupt
    if (mode == STANDBY)
    {
      disableSysTick();
    }

    // enter standby
    setSleepMode(STANDBY);
    __DSB();
    __WFI();

    // reenable systick interrupt
    if (mode == STANDBY)
    {
      enableSysTick();
    }
  }
};


/**
 * SAMD21 RTC in 32 bit counter mode for periodic interrupt callback (CMSIS based implementation)
 *
 * Background:
 *
 * Arduino library RTCZero does not support:
 * - RTC counter mode
 * - selectable generic clock
 */
class RealTimeClock
{
public:

  /**
   * setup RTC as 32 bit counter (mode 0)
   *
   * @param clkGenId GCLKGEN ID 0..7
   * @param clkGenFrequency frequency of GCLKGEN [Hz]
   *
   * @see RTCZero.cpp RTCZero::begin(bool)
   * @see RTCZero.cpp RTCZero::configureClock()
   */
  static void enable(byte clkGenId, uint32_t clkGenFrequency)
  {
    // save GCLK frequency for calculating time duration
    RealTimeClock::clkGenFrequency = clkGenFrequency;

    // set GCLK as source for RTC (GCLK Mux 4)
    System::enableClock(GCM_RTC, clkGenId);

    // RTC software reset
    RTC->MODE0.CTRL.reg |= RTC_MODE0_CTRL_SWRST;
    while (RTC->MODE0.CTRL.bit.SWRST);

    // setup RTC for mode 0, 1 Hz, clear on match (periodic timer), no continuous read
    RTC->MODE0.READREQ.reg &= ~RTC_READREQ_RCONT;
    RTC->MODE0.CTRL.reg = (uint16_t)(RTC_MODE0_CTRL_MODE_COUNT32 |
                                     RTC_MODE0_CTRL_PRESCALER_DIV1024 |
                                     RTC_MODE0_CTRL_MATCHCLR);
    RealTimeClock::clkDiv = 1024;

    // enable RTC interrupt
    NVIC_DisableIRQ(RTC_IRQn);
    NVIC_ClearPendingIRQ(RTC_IRQn);
    NVIC_SetPriority(RTC_IRQn, 0x00);
    NVIC_EnableIRQ(RTC_IRQn);

    // enable RTC
    RTC->MODE0.CTRL.reg |= RTC_MODE0_CTRL_ENABLE;
    while (RTC->MODE0.STATUS.bit.SYNCBUSY);
  }

  static void disable()
  {
    // RTC software reset
    RTC->MODE0.CTRL.reg |= RTC_MODE0_CTRL_SWRST;
    while (RTC->MODE0.CTRL.bit.SWRST);

    // disable RTC module
    PM->APBAMASK.reg &= ~PM_APBAMASK_RTC;

    // disable RTC GCM
    System::disableClock(GCM_RTC);

    // clear RTC interrupts
    NVIC_DisableIRQ(RTC_IRQn);
    NVIC_ClearPendingIRQ(RTC_IRQn);
  }

  /**
   * set timer period
   * @param millis [ms]
   *
   * note: only 1 second resolution available due to clock divider setup
   */
  static void setTimer(uint32_t millis, void (*callback)() = nullptr)
  {
    rtcHandler = callback;

    RTC->MODE0.COMP[0].reg = (uint32_t)clkGenFrequency/clkDiv*millis/1000;

    RTC->MODE0.INTENSET.reg = RTC_MODE0_INTENSET_CMP0;
    while (RTC->MODE0.STATUS.bit.SYNCBUSY);
  }

public:
  static void (*rtcHandler)();

private:
  static uint64_t clkGenFrequency; // [Hz]
  static uint16_t clkDiv; // 1..1024
};

uint64_t RealTimeClock::clkGenFrequency = 1024;
uint16_t RealTimeClock::clkDiv = 1024;
void (*RealTimeClock::rtcHandler)() = nullptr;

/**
 * SAMD21 RTC interrupt handler
 */
void RTC_Handler()
{
  // clear all interrupt flags
  RTC->MODE0.INTFLAG.reg = RTC->MODE0.INTENSET.reg;

  // handle interrupt
  if (RealTimeClock::rtcHandler)
  {
    RealTimeClock::rtcHandler();
  }
}


/**
 * SAMD21 timer counter for single or periodic interrupt callback (CMSIS based implementation)
 *
 * Background:
 *
 * Arduino core uses timer counters to provide PWM output but does not support:
 * - timer interrupts
 * - configurable generic clock
 *
 * Note that Arduino core uses a fixed mapping of timer counter to PWM pin
 * (see g_APinDescription in variant.h) that may cause conflicts if used
 * in parallel with this class.
 */
class TimerCounter
{
public:
  enum Prescaler
  {
    DIV1    = 0,
    DIV2    = 1,
    DIV4    = 2,
    DIV8    = 3,
    DIV16   = 4,
    DIV64   = 5,
    DIV256  = 6,
    DIV1024 = 7
  };

  enum Resolution
  {
    RES8  =  8,
    RES16 = 16,
    RES32 = 32
  };

public:
  TimerCounter() = default;

public:
  /**
   * @param id timer counter ID 3..5
   * @param clkGenId GCLKGEN ID 0..7
   * @param clkGenFrequency frequency of GCLKGEN [Hz]
   * @param div GCLK prescaler 0..7
   * @param resolution counter resolution [bits] 8, 16, TC4: 32
   */
  bool enable(byte id, byte clkGenId, uint32_t clkGenFrequency, Prescaler clkDiv, Resolution resolution)
  {
    if (id >= 3 && id <= 5 && (resolution == RES8 || resolution == RES16 || (id == 4 && resolution == RES32)))
    {
      this->clkGenFrequency = clkGenFrequency;
      this->resolution = resolution;

      switch (clkDiv)
      {
        case DIV2:
          this->clkDiv = 2;
          break;
        case DIV4:
          this->clkDiv = 4;
          break;
        case DIV8:
          this->clkDiv = 8;
          break;
        case DIV16:
          this->clkDiv = 16;
          break;
        case DIV64:
          this->clkDiv = 64;
          break;
        case DIV256:
          this->clkDiv = 256;
          break;
        case DIV1024:
          this->clkDiv = 1024;
          break;
        default:
          this->clkDiv = 1;
      }

      byte modeCount;
      switch (resolution)
      {
        case RES8:
          modeCount = TC_CTRLA_MODE_COUNT8;
          break;
        case RES32:
          modeCount = TC_CTRLA_MODE_COUNT32;
          break;
        default:
          modeCount = TC_CTRLA_MODE_COUNT16;
      }

      // enable GCM and TC
      switch (id)
      {
        case 3:
          System::enableClock(GCM_TCC2_TC3, clkGenId);
          PM->APBCMASK.reg |= PM_APBCMASK_TC3;
          tc = TC3;
          irq = TC3_IRQn;
          break;

        case 4:
          System::enableClock(GCM_TC4_TC5, clkGenId);
          PM->APBCMASK.reg |= PM_APBCMASK_TC4;
          if (resolution == 32)
          {
            // also enable TC5
            PM->APBCMASK.reg |= PM_APBCMASK_TC5;
          }
          tc = TC4;
          irq = TC4_IRQn;
          break;

        case 5:
          System::enableClock(GCM_TC4_TC5, clkGenId);
          PM->APBCMASK.reg |= PM_APBCMASK_TC5;
          tc = TC5;
          irq = TC5_IRQn;
          break;
      }

      // TC software reset
      tc->COUNT8.CTRLA.reg |= TC_CTRLA_SWRST;
      while (tc->COUNT8.CTRLA.bit.SWRST);

      // configure TC
      tc->COUNT8.CTRLA.reg = modeCount |
                             TC_CTRLA_WAVEGEN_MFRQ |
                             TC_CTRLA_PRESCALER(clkDiv);
      while (tc->COUNT8.STATUS.bit.SYNCBUSY);

      // setup IRQ
      NVIC_DisableIRQ(irq);
      NVIC_ClearPendingIRQ(irq);
      NVIC_SetPriority(irq, 0x00);
      NVIC_EnableIRQ(irq);

      // enable overflow interrupt (needed for oneshot mode)
      tc->COUNT8.INTENSET.reg = TC_INTENSET_OVF;
      //tc->COUNT8.INTENSET.reg = TC_INTENSET_MC0;

      // register instance
      timerCounter[id - 3] = this;

      return true;
    }
    else
    {
      // invalid parameters
      return false;
    }
  }

  /**
   * @param duration [ms]
   */
  void start(uint32_t duration, byte periodic = false, void (*callback)() = nullptr)
  {
    if (tc)
    {
      tcHandler = callback;

      // disable counter
      tc->COUNT8.CTRLA.bit.ENABLE = 0;
      while (tc->COUNT8.STATUS.bit.SYNCBUSY);

      // set counter period value
      uint64_t count = clkGenFrequency/clkDiv*duration/1000;
      switch (resolution)
      {
        case RES8:
          if (count > UCHAR_MAX) count = UCHAR_MAX;
          tc->COUNT8.CC[0].reg = (uint8_t)count;
          tc->COUNT8.COUNT.reg = 0;
          break;

        case RES32:
          if (count > ULONG_MAX) count = ULONG_MAX;
          tc->COUNT32.CC[0].reg = (uint32_t)count;
          tc->COUNT32.COUNT.reg = 0;
          break;

        default:
          if (count > USHRT_MAX) count = USHRT_MAX;
          tc->COUNT16.CC[0].reg = (uint16_t)count;
          tc->COUNT16.COUNT.reg = 0;
      }

      // set one-shot mode
      tc->COUNT8.CTRLBSET.bit.ONESHOT = periodic? 0 : 1;
      while (tc->COUNT8.STATUS.bit.SYNCBUSY);

      // enable counter
      tc->COUNT8.CTRLA.reg |= TC_CTRLA_ENABLE;
      while (tc->COUNT8.STATUS.bit.SYNCBUSY);
    }
  }

  /**
   * restart oneshot timer with previous setting for duration and callback
   */
  void restart()
  {
    if (tc && tc->COUNT8.CTRLBSET.bit.ONESHOT)
    {
      tc->COUNT8.CTRLBSET.reg |= TC_CTRLBSET_CMD_RETRIGGER;
      while (tc->COUNT8.STATUS.bit.SYNCBUSY);
    }
  }

public:
  static void isr(byte id)
  {
    if (id < 3)
    {
      TimerCounter* timterCounter = timerCounter[id];
      if (timterCounter)
      {
        // clear all interrupt flags
        timterCounter->tc->COUNT8.INTFLAG.reg = timterCounter->tc->COUNT8.INTENSET.reg;

        // handle interrupt
        if (timterCounter->tcHandler)
        {
          timterCounter->tcHandler();
        }
      }
    }
  }

private:
  static TimerCounter* timerCounter[3];

private:
  uint64_t clkGenFrequency; // [Hz]
  uint16_t clkDiv;          // 1..1024
  Resolution resolution;    // 8, 16, 32
  IRQn_Type irq;
  Tc* tc = nullptr;
  void (*tcHandler)() = nullptr;
};

TimerCounter* TimerCounter::timerCounter[3] = { nullptr };

/**
 * SAMD21 TC3 interrupt handler
 */
void TC3_Handler()
{
  TimerCounter::isr(0);
}

/**
 * SAMD21 TC4 interrupt handler
 */
void TC4_Handler()
{
  TimerCounter::isr(1);
}

/**
 * SAMD21 TC5 interrupt handler
 */
void TC5_Handler()
{
  TimerCounter::isr(2);
}


/******************************************************************************
 *
 * Application
 *
 ******************************************************************************/

TimerCounter timer;

void setup()
{
  //System::tune();

  // enable PORT
  PM->AHBMASK.reg |= PM_AHBMASK_HPB1;
  PM->APBBMASK.reg |= PM_APBBMASK_PORT;

  blink(PIN_LED3, 500);

  // setup 32 kHz ultra low power clock generator for RTC and EIC
  const byte GCLKGEN_ID_1K = GCLK_CLKCTRL_GEN_GCLK6_Val;
  System::setupClockGenOSCULP32K(GCLKGEN_ID_1K, 4); // 2^(4+1) = 32 -> 1 kHz

  // enable RTC timer
  RealTimeClock::enable(GCLKGEN_ID_1K, 1024);
#ifdef TEST_SLEEP
  RealTimeClock::setTimer(5*1000, []{ wakeup(); }); // 5 s
#else
  RealTimeClock::setTimer(2*60*1000, []{ wakeup(); }); // 2 min
#endif

#ifdef TEST_EIC
  // enable external interrut on D0 and change EIC GCLKGEN
  noInterrupts();
  pinMode(PIN_A0, INPUT_PULLUP);
  attachInterrupt(PIN_A0, []{ wakeup(); }, LOW);
  System::enableClock(GCM_EIC, GCLKGEN_ID_1K);
  interrupts();
#endif

  // setup TC
  //timer.enable(3, GCLK_CLKCTRL_GEN_GCLK0_Val, SystemCoreClock, TimerCounter::DIV1024, TimerCounter::RES16);
  timer.enable(4, GCLKGEN_ID_1K, 1024, TimerCounter::DIV1, TimerCounter::RES16);

  blink(PIN_LED, 500);

  // standby between ISR calls
#ifndef TEST_LOOP
  System::setSleepOnExitISR(true);
  System::setSleepMode(System::STANDBY);
#endif
}

/**
 * ISR called by RTC timer or external interrrupt
 */
void wakeup()
{
#ifndef TEST_LOOP
  #ifndef TEST_SLEEP
    // LED on
    digitalWrite(PIN_LED, LOW);
  #endif

  // delay LED off
  System::enableSysTick();
#ifdef TEST_SLEEP
  timer.start(2000, false, []
#else
  timer.start(50, false, []
#endif
  {
    // LED off
    digitalWrite(PIN_LED, HIGH);

    // standby
    System::disableSysTick();
    System::setSleepMode(System::STANDBY);
  });

  // idle until LED off
  System::setSleepMode(System::IDLE2);
#endif
}

void loop()
{
#ifdef TEST_LOOP
  #ifdef TEST_BLINK
    blink(PIN_LED, 2000);
  #else
    while(1);
  #endif
#else
  __WFI();
#endif
}

#elif defined(ARDUINO_TEENSYLC)

#include <Snooze.h>

#define PIN_LED 13

/*
   F_CPU of a Teensy LC (teensy3 core) can be changed directly via Arduino's board.txt
   without affecting USB functionality:
     C:\Program Files (x86)\Arduino\hardware\teensy\avr

   The Snooze library will only build with USB enabled and with the default values for
   F_CPU (48 MHz and 24 MHz).

   Teensy LC power consumption (3.0 V, USB on):

     48 MHz, LED on:  13.68 mA
     48 MHz, LED off: 11.39 mA
      8 MHz, LED on:   6.72 mA
      8 MHz, LED off:  4.42 mA
      0 MHz, LED off:  0.0043 mA - hibernate, RTC timer
 */

SnoozeTimer timer;
SnoozeBlock defaultSnooze(timer);

void setup()
{
  pinMode(PIN_LED, OUTPUT);
}

void loop()
{
  digitalWrite(PIN_LED, HIGH);
  delay(500);
  digitalWrite(PIN_LED, LOW);
  delay(500);

  //pinMode(PIN_LED, INPUT); will increase standby current by 0.04 mA

  Snooze.hibernate(defaultSnooze);
}

#endif
