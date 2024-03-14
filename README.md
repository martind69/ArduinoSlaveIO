# Arduino I/O Extender
This project solves the problem of task centralization. The effort of coding all tasks into a single project can be difficult. So let's think about decentralization networks!

For modular programming we can use the Arduino I/O Extender. It works as a I2C slave device.

# Features
Performing tasks in a decentralized way.

Supported: GPIO, PWM, ADC

# Registers
## General
**Device-ID**
|Offset|Bit 7|Bit 6|Bit 5|Bit 4|Bit 3|Bit 2|Bit 1|Bit 0|
|---|---|---|---|---|---|---|---|---|
|`0x00`|0|0|1|0|0|1|1|1|
||R|R|R|R|R|R|R|R|

## Digital
**GPIO** <sub>0x10 to 0x25</sub>
|Offset|Bit 7|Bit 6|Bit 5|Bit 4|Bit 3|Bit 2|Bit 1|Bit 0|
|---|---|---|---|---|---|---|---|---|
|`0x10`|0|x|x|x|0|0|0|x|
||R|R/W|R/W|R/W|R|R|R|R/W|
|||PWM|PUR|DIR||||PIN|

**PWM** <sub>0x26 to 0x2B</sub>
|Offset|Bit 7|Bit 6|Bit 5|Bit 4|Bit 3|Bit 2|Bit 1|Bit 0|
|---|---|---|---|---|---|---|---|---|
|`0x26`|x|x|x|x|x|x|x|x|
||R/W|R/W|R/W|R/W|R/W|R/W|R/W|R/W|

** 8-bit resulution on target ATMega328P

## Analog
**ADC** <sub>0x2C to 0x3B</sub>
|Offset|Bit 15|Bit 14|Bit 13|Bit 12|Bit 11|Bit 10|Bit 9|Bit 8|Bit 7|Bit 6|Bit 5|Bit 4|Bit 3|Bit 2|Bit 1|Bit 0|
|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|
|`0x2C:2D`|0|0|0|0|0|x|x|x|x|x|x|x|x|x|x|x|
||R|R|R|R|R|R|R|R|R|R|R|R|R|R|R|R|

** right justified format

** 10-bit resulution on target ATMega328P

## Counter
> [!NOTE]
> Time/Frequency measurement
> 
> This feature is not implementet yet

## Terminology
**PIN** - Pin state, regardless of DIR

    0 = LOW
    1 = HIGH

**DIR** - Pin direction

    0 = INPUT
    1 = OUTPUT
    
**PUR** - Pull-up resistor

    0 = DISABLE
    1 = ENABLE

**PWM** - Pulse-width modulation

    0 = DISABLE
    1 = ENABLE

