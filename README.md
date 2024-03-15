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
|`0x10`|x|0|0|0|x|x|x|x|
||R/W|R|R|R|R/W|R/W|R/W|R/W|
||!LOCK||||PWM|PUR|DIR|PIN|

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

# Examples
I2C Master device code snippets
## Output
Setting GPIO 0 and 1 as Output - GPIO 0 set High - GPIO 1 set Low

Each manipulation on bits 1-6 needs an active unlock bit.
```
Wire.beginTransmission(0x50);
Wire.write(0x10);
Wire.write(0x83);
Wire.write(0x82);
Wire.endTransmission();
```
Changing GPIO 0 and 1 output status - GPIO 0 set Low - GPIO 1 set High

Each consequent write to an output pin doesn't need an unlock bit.
```
Wire.beginTransmission(0x50);
Wire.write(0x10);
Wire.write(0x00);
Wire.write(0x01);
Wire.endTransmission();
```
## Input
Setting GPIO 2 and 4 as Input - GPIO 4 with enabled pull-up

Each manipulation on bits 1-6 needs an active unlock bit.
```
Wire.beginTransmission(0x50);
Wire.write(0x12);
Wire.write(0x82);
Wire.endTransmission();

Wire.beginTransmission(0x50);
Wire.write(0x14);
Wire.write(0x86);
Wire.endTransmission();
```
Or in a sequential manner
```
Wire.beginTransmission(0x50);
Wire.write(0x12);
Wire.write(0x82);
Wire.write(0x00); // may influence previous setting of pin 3
Wire.write(0x86);
Wire.endTransmission();
```
Reading GPIO 2 input
```
static uint8_t buffer[BUFFER_LENGTH], length;
uint8_t i = 0;

Wire.beginTransmission(0x50);
Wire.write(0x12);
Wire.endTransmission(false); // don't send stop
Wire.requestFrom(0x50, 1, 1); // send stop after 1 byte rx

while(Wire.available()) {
    buffer[i++] = Wire.read(); // receive a byte
    length = i; // save length
}
```

