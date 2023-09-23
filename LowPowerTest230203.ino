/*****************************************************************************
 *
 * Low Power Test for SAMD21 and Teensy LC (both with ARM Cortex M0+)
 *
 * file:     LowPowerTest230203.ino
 * encoding: UTF-8
 * created:  03.02.2023
 *
 *****************************************************************************
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

// Arduino includes, LGPL
#include <Arduino.h>


// comment in to test external interrupt
//#define TEST_EIC
// comment in to test timer interrupt + sleep modes with 2 s IDLE2 every 5 s STANDBY, otherwise slow blink for 50 ms every 2 min
//#define TEST_SLEEP
// comment in to use Arduino loop(), otherwise slow blink for 50 ms every 2 min
//#define TEST_LOOP
// comment in in combination with TEST_LOOP to execute blink(2000)
//#define TEST_BLINK
// comment in to read from ADC
#define TEST_ADC

#if F_CPU == 48000000L
  // comment in to use USB at 48 MHz
  //#define ENABLE_USB
#endif


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


#ifdef ARDUINO_SAMD_ZERO

#include "src/Analog2DigitalConverter.hpp"
#include "src/RealTimeClock.hpp"
#include "src/System.hpp"
#include "src/TimerCounter.hpp"
using namespace SAMD21LPE;


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


   Seed XIAO SAMD21 power consumption

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
      8 MHz :  1.47 mA - OSC8M,   PM+GCLKGEN+GCLK+NVM tuning, bus dividers 1/1/1, blink(2000), ADC
      8 MHz :  1.44 mA - OSC8M,   PM+GCLKGEN+GCLK+NVM tuning, bus dividers 1/1/1, blink(2000)
      6 MHz :  1.10 mA - DFLL48M, PM+GCLKGEN+GCLK+NVM tuning, bus dividers 1/1/1, while(1)
      6 MHz :  0.98 mA - DFLL48M, PM+GCLKGEN+GCLK+NVM tuning, bus dividers 4/128/128, while(1)
      8 MHz :  0.76 mA - OSC8M,   PM+GCLKGEN+GCLK+NVM tuning, bus dividers 1/1/1, while(1)
      8 MHz :  0.61 mA - OSC8M,   PM+GCLKGEN+GCLK+NVM tuning, bus dividers 4/128/128, while(1)
      4 MHz :  0.41 mA - OSC8M,   PM+GCLKGEN+GCLK+NVM tuning, bus dividers 4/128/128, while(1)

      -> 49 µA/MHz + 0.67 mA with DFLL48M, bus dividers 4/128/128
      -> 61 µA/MHz + 0.17 mA with OSC8M, bus dividers 4/128/128

      8 MHz :  0.34 mA - IDLE2, SLEEPONEXIT, PM+GCLKGEN+GCLK+NVM tuning, bus dividers 1/1/1, ADC
      8 MHz :  0.31 mA - IDLE2, SLEEPONEXIT, PM+GCLKGEN+GCLK+NVM tuning, bus dividers 1/1/1
      8 MHz :  0.29 mA - IDLE2, SLEEPONEXIT, PM+GCLKGEN+GCLK+NVM tuning, bus dividers 4/128/128

      0 MHz :  16.1 µA - STANDBY, LDO pin 3 connected
      0 MHz :   2.0 µA - STANDBY
 */


void usb_disable()
{
#if defined(USBCON)
  //USBDevice.standby(); // no discernible effect
  USBDevice.detach();
  USBDevice.end();
#endif
}


Analog2DigitalConverter& adc = Analog2DigitalConverter::instance();
RealTimeClock& rtc = RealTimeClock::instance();
TimerCounter timer;

void setup()
{
#ifdef ENABLE_USB
  SerialUSB.begin(9600);
#else
  System::tune();
#endif

  System::enablePORT();

  blink(PIN_LED3, 500);

#ifdef TEST_ADC
  // enable ADC, Vin = 0..2V (8 MHz / 8 -> 1 MHz)
  adc.enable(GCLK_CLKCTRL_GEN_GCLK0_Val, SystemCoreClock, Analog2DigitalConverter::DIV8);
  adc.setSampling(adc.durationToHalfPeriods(100), 1);
  adc.setGain(Analog2DigitalConverter::GAIN_DIV2);
  adc.disable();
#endif

#ifndef TEST_LOOP
  // setup GCLK 6 for RTC and EIC using 32 kHz ultra low power clock generator
  const byte GCLKGEN_ID_1K = GCLK_CLKCTRL_GEN_GCLK6_Val;
  System::setupClockGenOSCULP32K(GCLKGEN_ID_1K, 4); // 2^(4+1) = 32 -> 1 kHz

  // enable RTC timer (1 Hz)
  rtc.enable(GCLKGEN_ID_1K, 1024);
  #ifdef TEST_SLEEP
    rtc.setTimer(5*1000, []{ wakeup(); }); // 5 s
  #else
    rtc.setTimer(2*60*1000, []{ wakeup(); }); // 2 min
  #endif
#endif

#ifdef TEST_EIC
  // enable external interrut on D0 and change EIC GCLKGEN
  noInterrupts();
  pinMode(PIN_A0, INPUT_PULLUP);
  attachInterrupt(PIN_A0, []{ wakeup(); }, LOW);
  System::enableClock(GCM_EIC, GCLKGEN_ID_1K);
  interrupts();
#endif

#ifndef TEST_LOOP
  // setup TC (1 kHz)
  //timer.enable(3, GCLK_CLKCTRL_GEN_GCLK0_Val, SystemCoreClock, TimerCounter::DIV1024, TimerCounter::RES16);
  timer.enable(4, GCLKGEN_ID_1K, 1024, TimerCounter::DIV1, TimerCounter::RES16);
#endif

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
  // LED on
  #ifndef TEST_SLEEP
    digitalWrite(PIN_LED, LOW);
  #endif

  System::enableSysTick();

  // read temperature
  #ifdef TEST_ADC
    adc.reenable();
    float t = adc.read(ADC_INPUTCTRL_MUXPOS_TEMP_Val);
    adc.disable();
  #endif

  // delay LED off
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

  // idle until timer
  System::setSleepMode(System::IDLE2);
#endif
}

void loop()
{
#ifdef TEST_LOOP
  #ifdef TEST_BLINK
    blink(PIN_LED, 2000);
    #ifdef TEST_ADC
      // get MCU temperature
      //adc.reenable();
      float t = adc.read(ADC_INPUTCTRL_MUXPOS_TEMP_Val);
      float v = adc.read(ADC_INPUTCTRL_MUXPOS_SCALEDIOVCC_Val);
      float a = adc.read(g_APinDescription[PIN_A2].ulADCChannelNumber);
      //adc.disable();
      #ifdef ENABLE_USB
        while(!SerialUSB);
        SerialUSB.print(t);
        SerialUSB.print("/");
        SerialUSB.print(v);
        SerialUSB.print("/");
        SerialUSB.print(a);
        SerialUSB.print(" ");
      #endif
    #endif
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
