# Low Power Stamp Sized Development Boards

### Table of contents

[1. Motivation](#motivation)  
[2. MCUs](#mcus)  
[3. Low Power Strategies](#low-power-stragegies)  
[4. Seed Studio XIAO SAMD21](#seed-studio-xiao-samd21)  
[5. Seed Studio XIAO ESP32C3](#seed-studio-xiao-esp32c3)  
[6. Conclusions](#conclusions)  
[7. Licenses and Credits](#licenses-and-credits)  

## Motivation

After testing a solar powered energy harvesting module I was looking for a suitable microcontroller development board with the following features:

- variable supply voltage between 2 V and 3 V
- standby consumption less than 5 µA
- power on consumption less than 5 mA
- extended standby durations with periodic wakeup periods of at least 20 minutes
- I2C and SPI support
- read Vcc or 1 analog input
- 9 digital I/Os
- stamp sized, not larger than 25x20 mm
- optional 433 MHz RF transmitter

## MCUs

A brief web search resulted in a comparatively short list of candidates (definitely not representative). The following table lists the lowest possible consumption of the MCUs of these candidates and of a few additional popular MCUs. The values are taken from the manufacturer's datasheet, preferably for 3 V Vcc where available:

| MCU                   | Standby [µA]       | max. Wakeup Period | Notes                       | 
| ----------------------| ------------------ | ------------------ | --------------------------- | 
| Atmel ATtiny          |  10.00 (max)       | 8 s (watchdog)     | small flash, limited SPI    | 
| Atmel ATmega328P      |  44.00 (max)       | 8 s (watchdog)     | no stamp sized boards found | 
| Atmel ATmega32U2/4    |  10.00             | 8 s (watchdog)     |                             | 
| Espressif ESP8266     |  20.00             | 71 min (DSW)       |                             | 
| Espressif ESP32S1     |  10.00 (RTC)       | 4 h (RTC)          |                             | 
| Espressif ESP32C3     |   5.00 (RTC)       | 4 h (RTC)          |                             | 
| Feescale Kinetis KL26 |   1.07 (VLLS1 RTC) | 136 y (RTC)        | no stamp sized boards found | 
| Microchip SAM D21/DA1 |   4.06 (STDBY RTC) | 136 y (RTC)        |                             | 
| NORDIC nRF52832       |   1.90 (RTC)       | 24 d (RTC)         |                             | 
| NORDIC nRF52833       |   1.60 (RTC)       | 24 d (RTC)         |                             | 
| NORDIC nRF52840       |   1.50 (RTC)       | 24 d (RTC)         |                             | 
| Raspberry RP2040      | 180.00 (DORMANT)   | RTC alarm          |                             | 
| ST STM8L              |   0.92 (AH RTC)    | 36 h (RTC)         | no stamp sized boards found | 

While some MCUs include a WiFi and/or Bluetooth transmitter, I could not find a mainstream MCU with a sub 1 GHz transmitter. On the other hand there are several boards available that combine one of the MCUs listed with a separate RF chip. 

The table above should have a column for the power on consumption, but here things become difficult. The various manufacturers don't use standardized benchmarks limiting a comparison based on the datasheet values.

The listed MCUs allow changing the clock speed and disabling various features to save power, but this requires a MCU specific implementation. Making use of these MCU specific options and applying them to the use case will minimize the average power consumption. The result will probably differ significantly form the typical values listed in the datasheet.

Excluding all boards with a standby consumption that is higher than targeted and looking at current price and availability I selected the Seed Studio XIAO ESP32C3 and SAMD21 for closer inspection.

## Low Power Strategies

When designing a microcontroller application that should work with a power supply that is sustained by energy harvesting, the average energy consumption must stay significantly below the average energy yield, typically limiting the complexity of the application. To make the most of the energy available, the following strategies come to mind:

- adjusting the CPU speed to the task complexity
- adjusting the application activity to the available energy
- passive waiting
- entering standby as much as possible
- power optimized circuit design

Which of these strategies can be employed depends on the application. The best results can be achieved if all strategies can be combined.

Reducing the clock speed to a frequency suitable for the use case is typically the simplest way to reduce power consumption because many MCUs show a more or less linear correlation between clock speed and consumption.

The other strategies mentioned require a MCU specific implementation because the power saving features of the MCUs and their use differ significantly.

The code of this project provides an example how to use the features of a specific MCU to this end and it will also show that the programming by itself is not enough to minimize the power consumption to an acceptable level. Note that for the sake of easy current measurement active waiting instead of passive waiting is used by calling *delay()*.


## Seed Studio XIAO SAMD21

### Throttling the CPU

Starting with the Seed Studio XIAO SAM D21 and using the Arduino IDE the "blink" was running in no time, but then frustration set in because it was not possible to change the clock speed to 8 MHz by adding an entry for *F_CPU* to Arduino's boards.txt - the MCU just kept hanging in startup.

Repeating this test with a Teensy LC took only a few minutes - throttling the clock just worked as expected and even USB kept working. The Teensy LC has a Feescale Kinetis KL26 MCU with the same ARM Cortex-M0+ core as the Microchip SAM D21/DA1 MCU. This confirmed to me that the problem with the XIAO SAMD21 was not due to the MCU but probably due to the SDK. 

Looking through the Arduino SAMD SDK source files the cause could be traced to the initialization of the main oscillator in the reset handler *SystemInit()* in file startup.c (from Seed Studio SAMD SDK 1.8.3). The DFLL48M must be initialized for approximately 48 MHz, regardless of the desired main clock speed, but was using *F_CPU* as parameter. Setting *F_CPU* to 32 MHz still worked, but 8 MHz is definitely out of range. The following modification of startup.c allows the selection of several lower clock frequencies (e.g. 24 MHz and 6 MHz). Only USB will no longer work because it needs a GCLK running at 48 MHz.

```C++
* 4) Enable DFLL48M clock
...
/* The DFLL48M clock is specified for 48 MHz, cf product datasheet chapter 37.13.3 - Digital Frequency Locked Loop (DFLL48M) Characteristics. Configuring other frequencies might work, but is not supported. */
    
#if F_CPU >= 32000000L
  #define F_DFLL F_CPU
  #define GCLK0_DIV -1
#elif F_CPU == 24000000L
  #define F_DFLL 48000000L
  #define GCLK0_DIV 0
#elif F_CPU == 16000000L
  #define F_DFLL 32000000L
  #define GCLK0_DIV 0
#elif F_CPU == 12000000L
  #define F_DFLL 48000000L
  #define GCLK0_DIV 1
#elif F_CPU == 8000000L
  #define F_DFLL 32000000L
  #define GCLK0_DIV 1
#elif F_CPU == 6000000L
  #define F_DFLL 48000000L
  #define GCLK0_DIV 2
#elif F_CPU == 4000000L
  #define F_DFLL 32000000L
  #define GCLK0_DIV 2
#elif F_CPU == 3000000L
  #define F_DFLL 48000000L
  #define GCLK0_DIV 3
#elif F_CPU == 1500000L
  #define F_DFLL 48000000L
  #define GCLK0_DIV 4
#else
  #error supported F_CPU values are 1500000L, 3000000L, 6000000L, 12000000L, 24000000L and 48000000L
#endif

SYSCTRL->DFLLMUL.reg = SYSCTRL_DFLLMUL_CSTEP( 31 ) | // Coarse step is 31, half of the max value
                       SYSCTRL_DFLLMUL_FSTEP( 511 ) | // Fine step is 511, half of the max value
                       SYSCTRL_DFLLMUL_MUL( (F_DFLL + VARIANT_MAINOSC/2) / VARIANT_MAINOSC ) ; // External 32KHz is the reference
...
/* ----------------------------------------------------------------------------------------------
 * 5) Switch Generic Clock Generator 0 to DFLL48M. CPU will run at F_CPU.
 */
GCLK->GENDIV.reg = GCLK_GENDIV_ID( GENERIC_CLOCK_GENERATOR_MAIN ) | // Generic Clock Generator 0
                   (GCLK0_DIV >= 0? GCLK_GENDIV_DIV(GCLK0_DIV) : 0);  // Divide by 2^(GCLK0_DIV + 1)
...
/* Write Generic Clock Generator 0 configuration */
GCLK->GENCTRL.reg = GCLK_GENCTRL_ID( GENERIC_CLOCK_GENERATOR_MAIN ) | // Generic Clock Generator 0
                    GCLK_GENCTRL_SRC_DFLL48M | // Selected source is DFLL 48MHz
//                      GCLK_GENCTRL_OE | // Output clock to a pin for tests
                    GCLK_GENCTRL_IDC | // Set 50/50 duty cycle
                    GCLK_GENCTRL_GENEN |
                    (GCLK0_DIV >= 0? GCLK_GENCTRL_DIVSEL : 0);
```

An already patched version of [startup.c](arduino/startup.c "Seed Studio SAMD SDK 1.8.3 startup.c") is included with the project files.

With this blocker removed I was able to run "blink" at 6 MHz, feeding 3.0 V into the 3.3 V pin of the board. USB disconnected, green LED cut off on the cathode side and yellow LED on required 3.17 mA. Turning the yellow LED off let the current drop to 2.09 mA. 

### Standby

Putting the MCU into standby mode left 33 µA. Setting input pullup mode on all pins still left 16 µA, almost 12 µA higher than the value from the datasheet. It tried the EnergySaving class coming with the Seed Studio SAMD SDK to put the MCU into standby mode, but this increased the standby current by another 18 µA, because the library keeps the non volatile memory controller enabled.

Repeating this test with the Teensy LC using the Snooze library the standby current was immediately at 4.3 µA. Another study of the Microchip datasheet and another web search indicated that the high standby current is not due to a misconfiguration of the MCU but may be caused by the board components. Checking the schematic of the XIAO board, a reverse voltage into the voltage regulator U1 comes up as prime candidate. Using a handheld milling cutter I weakened the top edges of the RF shield, opening it with a needle, similar to opening a can. The remaining sides of the RF shield could be removed from the PCB by bending them back and forth a few times. This is what the board looks like without a RF shield:

![Seed Studio XIAO SAMD21 without RF shield](assets/XIAO-SAMD21-open.jpg "Seed Studio XIAO SAMD21 without RF shield")

With a needle it was possible to lift pin 3 of U1 from the PCB with very little force. Now the standby current reads 2 µA, even without changing the mode of all pins to input pullup.

### Arduino Framework Limitations

The project code is specifically tailored for the SAMD21. Most of the code is not using the Arduino framework but the CMSIS-Core interface to access the MCU registers directly. Several methods of the project code are variations of methods of the Arduino core SDK, the Arduino library RTCZero and the ArduinoLowPowerClass. Using the Arduino implementations directly will not provide the same results in regard to power saving because the Arduino framework is designed for ease of use. 

### RTC and EIC

Using the Arduino library RTCZero would have been convenient, but only the clock/calendar mode is implemented and XOSC32K is used as clock source. Using OSCULP32K instead should minimize power consumption during standby further at the cost of a little timing precision. Using the RTC counter mode 0 will provide a periodic wakeup from standby. The class RealTimeClock of the project codes wraps the necessary setup tasks.

The standby current with the RTC based on OSCULP32K measured 2 µA - this is less than half the value from the datasheet with RTC based on OSC32K. 

Activating EIC additionally the same way does not change this value. This allows both timed and interrupt driven wakeup from standby without a significant change in power consumption.

### SAMD21 Asynchronous Clocks

As the SAMD21 can be configured to use asynchronous clocks with different frequencies for the CPU and the peripheral functions, almost every data exchange between the CPU and a peripheral must be synchronized, requiring extra code. This becomes apparent if you look at the method *RealTimeClock::enable()* in the project code, where around 40 % of the operations are sync waits. In such an environment the effectiveness of the code execution significantly depends on how much the clock speeds differ and how often peripheral access is used.

The Arduino core initializes all bus clock dividers at the end of *SystemInit()* to 1 to improve performance in this regard. This results in an almost classic MCU behaviour, where CPU and important peripherals use the same clock. At the same time this setting has the highest power consumption. 

There is no ideal solution for the trade-off between sync waits and power consumption, so the clock divider settings should be adjusted to the project requirements.

When throttling the MCU main clock below 12 MHz the NVM read wait states can be set to zero, further reducing the power consumption. This is done by the method *System::tune()* in the project code.

### Power On

The board supply current in bootloader mode with USB connected is 13 mA and 12 mA with USB disconnected and running "blink" at 48 MHz. This is a value that many other users report for an unmodified XIAO SAMD21.

Disconnecting USB and running a *while(1)* reduced the supply current to 8.1 mA, but this is still significantly more than the 3.4 mA listed in the SAMD21 datasheet.

To be able to reproduce the datasheet value, the power management settings and the generic clock settings resulting from the SAMD21 bootloader and the Arduino core have to be modified to match the test conditions described in the datasheet chapter 37.7 - "Power Consumption".

It is essential to:
- disable clocks of unused modules
- disable unused buses and adjust the bus clock dividers to 4/128/128
- disable unused clock generators
- assign unused clocks to a disabled clock generator

This is done by the method *System::tune()* in the project code. Especially the last aspect is a little curious, because one would expect that disabling a clock would have the same or better effect.

Tests with this configuration at different clock frequencies show the following relation between clock frequency and supply current:

$49 µA/MHz + 670 µA$

While the frequency dependent part is comparatively low, the static part is rather high. This is caused by the combination of XOSC32K and DFLL48M for the main clock.

Almost unsurprisingly the supply current could be decreased from 1 mA at 6 MHz to 610 µA at 8 MHz when using OSC8M instead of XOSC32K and DFLL48M, showing the following relation between clock frequency and supply current, that is significantly better at lower frequencies:

$61 µA/MHz + 170 µA$

More configurations and the measured consumptions are listed at the beginning of the project code.

### Passive Waiting

Peripheral operations often need several clock cycles to complete, e.g. the ADC. Typical solutions use active waiting by testing status values. Power efficiency can be improved by using an event driven approach instead, e.g. using interrupts or the event system of the SAMD21. In the time between the start of the operation and the completed event the MCU can either sleep or perform other operations.

The project code shows how to use timer interrupts as alternative to a delay based solution. The RTC timer interrupt will wakeup the MCU periodically every 2 minutes from standby, turn the LED on and immediately enter IDLE 2 sleep mode, reducing the supply current for the MCU from 0.61 mA at 8 MHz by 50 % to 0.29 mA. After 50 ms the timer counter interrupt will wakeup the MCU again, turn the LED off and put the MCU back into standby with a current consumption of 2 µA. At a 50 % duty cycle the average current would be around 150 µA compared to the 12 mA the MCU requires for a delay based "blink" at 48 MHz, reducing the power consumption of the MCU by a factor of 80.

Turning the LED on and off could be handled by the RTC timer alone if the RTC prescaler is set to support millisecond resolution and the I/O pin controller (PORT) is configured to use a generic clock generator that continues to run in standby. This should reduce the average current to a few µA.

## Seed Studio XIAO ESP32C3

pending ...

## Conclusions

Low power optimization of an MCU is not possible without a thorough understanding of the individual MCU features. There is no simple switch to be flipped to just make it work, instead an use case specific implementation should be considered. A solution can be based on

- an existing framework (e.g. the Arduino framework) and a readily available microcontroller board
- MCU register access and an individual PCB

Best results typically require using MCU register access because frameworks are more focused on ease of use and MCU abstraction.

This projects shows that a slightly modified XIAO SAMD21 board combined with a slightly modified Arduino framework and some extra code to support SAMD21 specific power management features can be a good choice for a low power project where the active time is short in comparison to the standby time. With a board supply current of 2 µA in standby and 610 µA at 8 MHz it can be powered by a battery for a year or more, or indefinitely through energy harvesting whenever enough energy has been collected.

## Licenses and Credits

### Documentation and Photos

Copyright (c) 2023 [Jens B.](https://github.com/jnsbyr)

[![License: CC BY-NC-SA 4.0](https://img.shields.io/badge/License-CC%20BY--NC--SA%204.0-lightgrey.svg)](https://creativecommons.org/licenses/by-nc-sa/4.0/)

### Source Code

Copyright (c) 2023 [Jens B.](https://github.com/jnsbyr)

[![License: Apache 2.0](https://img.shields.io/badge/License-Apache%202.0-blue.svg)](https://opensource.org/licenses/Apache-2.0)

The source code was edited and build using the [Arduino IDE](https://www.arduino.cc/en/software/), [Arduino CLI](https://github.com/arduino/arduino-cli) and [Microsoft Visual Studio Code](https://code.visualstudio.com).

The source code depends on:

#### Arduino SDK & Seed Studio SDK for SAMD21 and SAMD51

Copyright (C) 2014 [Arduino LLC](https://github.com/arduino/Arduino)

[![License: LGPL v2.1](https://img.shields.io/badge/License-LGPL%202.1%20only-blue.svg)](https://www.gnu.org/licenses/lgpl-2.1)

#### CMSIS Atmel

Copyright (C) 2015 [Atmel Corporation](https://github.com/arduino/ArduinoModule-CMSIS-Atmel)

[![License](https://img.shields.io/badge/License-BSD_2--Clause-orange.svg)](https://opensource.org/licenses/BSD-2-Clause)

Also many thanks to the Open Source community, the bloggers and the forum users that have shared their experience with the SAM D21 MCU.