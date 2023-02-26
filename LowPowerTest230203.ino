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

/*
   To be able to lower F_CPU of a Seed Studio XIAO SAMD21 via Arduino's board.txt
   the Arduino core file startup.c needs to be patched, see README.md for details.

   Note that USB communication will no longer work, because it requires an 48 MHz clock.

   The modified Arduino core and selected Arduino libraries taken together result in
   the following clock configuration for the SAMD21:

   OSC8M     -> GCLK0 (1 MHz)  -> (POR)
   DFLL48M   -> GCLK0 (F_CPU)  -> CPU, AHB, APBx, PM, ADC (F_CPU/32), DAC (F_CPU) (startup.c, wiring.c)
                                  USB (USBCore.cpp)

   XOSC32K   -> GCLK1 (32 kHz) -> GCLK Mux 0 -> DFLL48M (startup.c)

   OSCULP32K -> GCLK2 (32 kHz) -> WDT (POR)
   XOSC32K   -> GCLK2 (1 kHz)  -> RTC (RTCZero.cpp)

   OSC8M     -> GCLK3 (8 MHz)  -> unused (startup.c)

   OSCULP32K -> GCLK6 (32 kHz) -> EIC (ArduinoLowPower.cpp, RUNSTDBY)


   Seed XIAO SAM D21 power consumption (3.0 V, green LED cut, yellow LED off, LDO pin 3 disconnected):

     48 MHz : 12.97 mA - USB connected, yellow LED on, bootloader upload mode

     48 MHz : 10.88 mA - USB disconnected, RTC, EIC, DFLL48M, unnecessary clocks disabled
     24 MHz :  5.81 mA - USB disconnected, RTC, EIC, DFLL48M, unnecessary clocks disabled
      8 MHz :  2.47 mA - USB disconnected, RTC, EIC, DFLL48M, unnecessary clocks disabled
      8 MHz :  2.15 mA - USB disconnected, RTC, EIC, OSC8M,   unnecessary clocks disabled
      6 MHz :  2.09 mA - USB disconnected, RTC, EIC, DFLL48M, unnecessary clocks disabled
      4 MHz :  1.18 mA - USB disconnected, RTC, EIC, OSC8M,   unnecessary clocks disabled

      -> 0.21 mA/MHz + 0.83 mA with DFLL48M
      -> 0.24 mA/MHz + 0.22 mA with OSC8M

      0 MHz :  0.0161 mA - USB disconnected/init, MCU standby, RTC, LDO pin 3 connected
      0 MHz :  0.0021 mA - USB disconnected/init, MCU standby, RTC, EIC
      0 MHz :  0.0020 mA - USB disconnected/init, MCU standby

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

class Analog2DigitalConverter
{
public:
  /**
   * analog read (12 bits, max. 1 V)
   *
   * note: unused, not tested
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

class System
{
public:
  /**
   * setup a generic clock with given divider
   * with OSCULP32K as source and RUNSTDBY option
   */
  static void setupGCLKOSCULP32K(byte id, int8_t div = -1)
  {
    // setup GCLK divider
    GCLK->GENDIV.reg = GCLK_GENDIV_ID(id)|
                       (div>=0? GCLK_GENDIV_DIV(div) : 0);
    while (GCLK->STATUS.reg & GCLK_STATUS_SYNCBUSY);

    // set OSCULP32K as source for GCLK
    GCLK->GENCTRL.reg = GCLK_GENCTRL_ID(id) |
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
  static void setupGCLKOSC8M(byte id, int8_t div = -1)
  {
    // ensure OSC8M is enabled and running at 8 MHz
    SYSCTRL->OSC8M.reg &= ~SYSCTRL_OSC8M_PRESC_Msk;
    SYSCTRL->OSC8M.reg &= ~SYSCTRL_OSC8M_RUNSTDBY;
    SYSCTRL->OSC8M.reg |= SYSCTRL_OSC8M_ENABLE;

    if (div >= 0)
      for (int i=0; i<1+div; i++)
      {
        blink(PIN_LED3, 50, 500);
      }
    else
      blink(PIN_LED, 50, 500);

    // setup GCLK divider
    GCLK->GENDIV.reg = GCLK_GENDIV_ID(id) |
                       (div>=0? GCLK_GENDIV_DIV(div) : 0);
    while (GCLK->STATUS.reg & GCLK_STATUS_SYNCBUSY);

    // set OSC8M as source for GCLK
    GCLK->GENCTRL.reg = GCLK_GENCTRL_ID(id) |
                        GCLK_GENCTRL_GENEN |
                        GCLK_GENCTRL_SRC_OSC8M |
                        (div>=0? GCLK_GENCTRL_DIVSEL : 0) |
                        (div>=0? GCLK_GENCTRL_IDC : 0);
    while (GCLK->STATUS.reg & GCLK_STATUS_SYNCBUSY);
  }

  static void disableGCLK(byte id)
  {
    GCLK->GENCTRL.reg = GCLK_GENCTRL_ID(id);
    while (GCLK->STATUS.reg & GCLK_STATUS_SYNCBUSY);
  }

  /**
   * - disable clocks of unused modules
   * - adjust NVM read wait states
   */
  static void tune()
  {
    // read core voltage
    //float vCore = readADC(ADC_INPUTCTRL_MUXPOS_SCALEDCOREVCC);

    // essential clocks: AHB: NVMCTRL, HPB0-1, APB: SYSCTRL, PM, GCLK
    // additional clocks: APB: PORT, RTC, EIC
    // disable all other clocks that are enabled at reset
    // @see product datasheet table 16-1 - Peripheral Clock Default State
    // @see product datasheet chapter 37.7 - Power Consumption
    PM->AHBMASK.bit.USB_ = 0;
    PM->AHBMASK.bit.DMAC_ = 0;
    PM->AHBMASK.bit.DSU_ = 0;
    PM->AHBMASK.bit.HPB2_ = 0; // APBC bridge
    //PM->AHBMASK.bit.HPB1_ = 0; // APBB bridge
    PM->APBBMASK.bit.USB_ = 0;
    PM->APBBMASK.bit.DMAC_ = 0;
    PM->APBBMASK.bit.DSU_ = 0;
    PM->APBBMASK.bit.PAC1_ = 0;
    PM->APBAMASK.bit.WDT_ = 0;
    PM->APBAMASK.bit.PAC0_ = 0;
    PM->APBCMASK.bit.ADC_ = 0;

    if (F_CPU == 8000000L || F_CPU == 4000000L || F_CPU == 2000000L || F_CPU == 1000000L)
    {
      // switch GCLK0 to OSC8M and disable GCLK1 and DFLL48M
      setupGCLKOSC8M(0, 8000000L/F_CPU - 2);
      disableGCLK(1);
      SYSCTRL->DFLLCTRL.bit.ENABLE = 0;
      // disable GCLK2 and XOSC32K
      disableGCLK(2);
      SYSCTRL->XOSC32K.bit.ENABLE = 0;
    }
    else
    {
      // disable GCLK2
      disableGCLK(2);
    }

    // reduce NVM wait states (or Vdd >= 2.7 V, F_CPU <= 24000000L)
    // @see product datasheet chapter 37.12 - NVM Characteristics
    if (F_CPU <= 14000000L)
    {
      NVMCTRL->CTRLB.bit.RWS = 0;
    }
  }

  /**
   * put MCU into standby mode until wakeup interrupt
   *
   * @see ArduinoLowPower.cpp ArduinoLowPowerClass::sleep()
   */
  static void standby()
  {
    // disable systick interrupt
    SysTick->CTRL &= ~SysTick_CTRL_TICKINT_Msk;

    // enter standby
    SCB->SCR |= SCB_SCR_SLEEPDEEP_Msk;
    __DSB();
    __WFI();

    // reenable systick interrupt
    SysTick->CTRL |= SysTick_CTRL_TICKINT_Msk;
  }
};

class RealTimeClock
{
public:

  /**
   * setup RTC as 1 Hz 32 bit counter (mode 0)
   * @param gclklId ID of a 1 kHz GCLK
   *
   * @see RTCZero.cpp RTCZero::begin(bool)
   * @see RTCZero.cpp RTCZero::configureClock()
   */
  static void enable(byte gclklId)
  {
    // set GCLK as source for RTC (GCLK Mux 4)
    GCLK->CLKCTRL.reg = (uint16_t)GCLK_CLKCTRL_ID(GCM_RTC);
    while (GCLK->STATUS.bit.SYNCBUSY);
    GCLK->CLKCTRL.reg = (uint16_t)(GCLK_CLKCTRL_ID(GCM_RTC) |
                                   GCLK_CLKCTRL_CLKEN |
                                   GCLK_CLKCTRL_GEN(gclklId));
    while (GCLK->STATUS.bit.SYNCBUSY);

    // enable RTC module
    PM->APBAMASK.reg |= PM_APBAMASK_RTC;

    // disable RTC
    RTC->MODE0.CTRL.reg &= ~RTC_MODE0_CTRL_ENABLE;
    while (RTC->MODE0.STATUS.bit.SYNCBUSY);

    // RTC software reset
    RTC->MODE0.CTRL.reg |= RTC_MODE0_CTRL_SWRST;
    while (RTC->MODE0.STATUS.bit.SYNCBUSY);

    // setup RTC for mode 0, 1 Hz, clear on match (periodic timer), no continuous read
    RTC->MODE0.READREQ.reg &= ~RTC_READREQ_RCONT;
    RTC->MODE0.CTRL.reg = (uint16_t)(RTC_MODE0_CTRL_MODE_COUNT32 |
                                     RTC_MODE0_CTRL_PRESCALER_DIV1024 |
                                     RTC_MODE0_CTRL_MATCHCLR);

    // enable RTC interrupt
    NVIC_DisableIRQ(RTC_IRQn);
    NVIC_ClearPendingIRQ(RTC_IRQn);
    NVIC_SetPriority(RTC_IRQn, 0x00);
    NVIC_EnableIRQ(RTC_IRQn);

    // enable RTC
    RTC->MODE0.CTRL.reg |= RTC_MODE0_CTRL_ENABLE;
    while (RTC->MODE0.STATUS.bit.SYNCBUSY);

    // remove RTC software reset
    RTC->MODE0.CTRL.reg &= ~RTC_MODE0_CTRL_SWRST;
    while (RTC->MODE0.STATUS.bit.SYNCBUSY);

    // init RTC counter
    RTC->MODE0.COUNT.reg = 0;
    while (RTC->MODE0.STATUS.bit.SYNCBUSY);
  }

  static void disable()
  {
    // RTC software reset
    RTC->MODE0.CTRL.reg |= RTC_MODE0_CTRL_SWRST;
    while (RTC->MODE0.STATUS.bit.SYNCBUSY);

    // disable RTC
    RTC->MODE0.CTRL.reg &= ~RTC_MODE0_CTRL_ENABLE;
    while (RTC->MODE0.STATUS.bit.SYNCBUSY);

    // disable RTC module
    PM->APBAMASK.reg &= ~PM_APBAMASK_RTC;

    // disable RTC GCLK
    GCLK->CLKCTRL.reg = (uint16_t)GCLK_CLKCTRL_ID(GCM_RTC);
    while (GCLK->STATUS.bit.SYNCBUSY);

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
  static void setTimer(uint32_t millis)
  {
    RTC->MODE0.COMP[0].reg = millis/1000;

    RTC->MODE0.INTENSET.reg = RTC_MODE0_INTENSET_CMP0;
    RTC->MODE0.INTENCLR.reg = RTC_MODE0_INTENCLR_OVF | RTC_MODE0_INTENCLR_SYNCRDY;
    while (RTC->MODE0.STATUS.bit.SYNCBUSY);
  }
};

volatile bool rtcInt = false;

/**
 * RTC interrupt handler
 *
 * @see RTCZero.cpp RTC_Handler(void)
 */
void RTC_Handler(void)
{
  rtcInt = true;

  // clear interrupt flags
  RTC->MODE0.INTFLAG.reg = RTC_MODE0_INTFLAG_CMP0 | RTC_MODE0_INTFLAG_OVF | RTC_MODE0_INTFLAG_SYNCRDY;
}

class ExternalInterrupController
{
public:
  /**
   * change GCLK for EIC to run in standby, only needed for edge detection
   *
   * @see Winterrupts.c __initialize()
   */
  static void setGCLK(byte gclklId)
  {
    // set GCLK as source for EIC (GCLK Mux 5)
    GCLK->CLKCTRL.reg = (uint16_t)GCLK_CLKCTRL_ID(GCM_EIC);
    while (GCLK->STATUS.bit.SYNCBUSY);
    GCLK->CLKCTRL.reg = (uint16_t)(GCLK_CLKCTRL_ID(GCM_EIC) |
                                   GCLK_CLKCTRL_CLKEN |
                                   GCLK_CLKCTRL_GEN(gclklId));
    while (GCLK->STATUS.bit.SYNCBUSY);
  }
};

volatile bool eicInt = false;

void externalInterruptHandler(void)
{
  eicInt = true;
}


void setup()
{
  System::tune();

  blink(PIN_LED3, 2000);

  // setup low power clock generator for RTC
  const byte GCLK_ID = 6;
  const byte GCLK_DIV = 4; // 2^(4+1) = 32 -> 1 kHz
  System::setupGCLKOSCULP32K(GCLK_ID, GCLK_DIV);

  // enable RTC timer
  RealTimeClock::enable(GCLK_ID);
  RealTimeClock::setTimer(8000);  // 8 s

  // enable external interrut
  noInterrupts();
  pinMode(PIN_A0, INPUT_PULLUP);
  attachInterrupt(PIN_A0, externalInterruptHandler, LOW);
  ExternalInterrupController::setGCLK(GCLK_ID);
  interrupts();

  blink(PIN_LED, 2000);
}

void loop()
{
  // flash LED3 on interrupt
  if (rtcInt || eicInt)
  {
    rtcInt = false;
    eicInt = false;
    blink(PIN_LED3, 50, 500);
  }

  System::standby();

  // blink LED on wakeup
  blink(PIN_LED, 2000);
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
