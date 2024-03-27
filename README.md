# Arduino I/O Extender
This project solves the problem of task centralization. The effort of coding all tasks into a single project can be difficult. So let's think about decentralization networks!

For modular programming you can use the Arduino I/O Extender. It acts as an I2C slave on an ATMega328P target.

# Features
Performing tasks in a decentralized way.

Supported: GPIO, PWM, ADC, COUNTER/TIMER

# Registers
## General
### Device-Address <sub>0x50</sub> (changeable)

### Device-ID <sub>0x00</sub> (changeable)
|Offset|Bit 7|Bit 6|Bit 5|Bit 4|Bit 3|Bit 2|Bit 1|Bit 0|
|---|---|---|---|---|---|---|---|---|
|`0x00`|0|0|1|0|0|1|1|1|
||R|R|R|R|R|R|R|R|

### Counter-CFG <sub>0x07</sub>
|Offset|Bit 7|Bit 6|Bit 5|Bit 4|Bit 3|Bit 2|Bit 1|Bit 0|
|---|---|---|---|---|---|---|---|---|
|`0x07`|1|0|1|0|1|0|1|0|
||TOIE1|CS12|CS11|CS10|ISC11|ISC10|ISC01|ISC00|
||R/W|R/W|R/W|R/W|R/W|R/W|R/W|R/W|

** must be written before GPIO registers

|ISCx1|ISCx0|Interrupt Select|
|---|---|---|
|0|1|Both edges|
|1|0|Falling edge|
|1|1|Rising edge|

|CS12|CS11|CS10|Clock Select|
|---|---|---|---|
|0|0|1|OSC/1|
|0|1|0|OSC/8|
|0|1|1|OSC/64|
|1|0|0|OSC/256|
|1|0|1|OSC/1024|

|TOIE1|Overflow Event|
|---|---|
|0|Disabled, keeps last value at `0x3C:3D`|
|1|Enabled, writes `0xFFFF` into `0x3C:3D` on overflow|

## Digital
### GPIO <sub>0x10 to 0x25</sub>
|Offset|Bit 7|Bit 6|Bit 5|Bit 4|Bit 3|Bit 2|Bit 1|Bit 0|
|---|---|---|---|---|---|---|---|---|
|`0x10`|x|0|0|x|x|x|x|x|
||R/W|R|R|R/W|R/W|R/W|R/W|R/W|
||!LOCK|||CNT|PWM|PUR|DIR|PIN|

### PWM <sub>0x26 to 0x2B</sub>
|Offset|Bit 7|Bit 6|Bit 5|Bit 4|Bit 3|Bit 2|Bit 1|Bit 0|
|---|---|---|---|---|---|---|---|---|
|`0x26`|x|x|x|x|x|x|x|x|
||R/W|R/W|R/W|R/W|R/W|R/W|R/W|R/W|

** 8-bit resulution on target ATMega328P

## Analog
### ADC <sub>0x2C to 0x3B</sub>
|Offset|Bit 15|Bit 14|Bit 13|Bit 12|Bit 11|Bit 10|Bit 9|Bit 8|Bit 7|Bit 6|Bit 5|Bit 4|Bit 3|Bit 2|Bit 1|Bit 0|
|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|
|`0x2C:2D`|0|0|0|0|0|x|x|x|x|x|x|x|x|x|x|x|
||R|R|R|R|R|R|R|R|R|R|R|R|R|R|R|R|

** right-justified format

** 10-bit resulution on target ATMega328P

## Counter
### CNT <sub>0x3C to 0x3D</sub>
|Offset|Bit 15|Bit 14|Bit 13|Bit 12|Bit 11|Bit 10|Bit 9|Bit 8|Bit 7|Bit 6|Bit 5|Bit 4|Bit 3|Bit 2|Bit 1|Bit 0|
|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|
|`0x3C:3D`|x|x|x|x|x|x|x|x|x|x|x|x|x|x|x|x|
||R|R|R|R|R|R|R|R|R|R|R|R|R|R|R|R|

** right-justified format

** 16-bit resulution on target ATMega328P

## Terminology
### PIN - Pin status (regardless of DIR)
> [!NOTE]
> Usable pins: 0-21

    0 = LOW
    1 = HIGH

### DIR - Pin direction
> [!NOTE]
> Usable pins: 0-21
> 
> Pin 13 aka `LED_BUILTIN` already mapped as output

    0 = INPUT
    1 = OUTPUT
    
### PUR - Pull-up resistor
> [!NOTE]
> Usable pins: 0-21

    0 = DISABLE
    1 = ENABLE

### PWM - Pulse-width modulation
> [!IMPORTANT]
> Usable pins: 3, 5, 6, 9-11

    0 = DISABLE
    1 = ENABLE

### CNT - Pulse-counter
> [!IMPORTANT]
> Usable pins: 2, 3
>
> Only one pin is mappable as counter module

    0 = DISABLE
    1 = ENABLE

# Examples
I2C Master device code snippets

## Input setup
Setting GPIO 2 and 4 as input - GPIO 4 with enabled pull-up

> [!IMPORTANT]
> Each manipulation on bits 1-6 needs an active unlock bit.
```
Wire.beginTransmission(DEVICE_ADDRESS);
Wire.write(0x12);
Wire.write(0x80);
Wire.endTransmission();

Wire.beginTransmission(DEVICE_ADDRESS);
Wire.write(0x14);
Wire.write(0x84);
Wire.endTransmission();
```
Or in a sequential manner
```
Wire.beginTransmission(DEVICE_ADDRESS);
Wire.write(0x12);
Wire.write(0x80);
Wire.write(0x00); // doesn't influence previous setting of pin 3
Wire.write(0x84);
Wire.endTransmission();
```

### DI
Reading GPIO 2 input
```
static uint8_t buffer[BUFFER_LENGTH], length;
uint8_t i = 0;

Wire.beginTransmission(DEVICE_ADDRESS);
Wire.write(0x12);
Wire.endTransmission(false); // don't send stop
Wire.requestFrom(DEVICE_ADDRESS, 1, 1); // send stop after 1 byte rx

while(Wire.available()) {
    buffer[i++] = Wire.read() & 1; // receive a byte, mask only position 0
    length = i; // save length
}
```

### ADC
Reading GPIO 14 aka ADC 0 input
```
Wire.beginTransmission(DEVICE_ADDRESS);
Wire.write(0x1E);
Wire.write(0x80);
Wire.endTransmission();

Wire.beginTransmission(DEVICE_ADDRESS);
Wire.write(0x2C);
Wire.endTransmission(false); // don't send stop
Wire.requestFrom(DEVICE_ADDRESS, 2, 1); // send stop after 2 byte rx

while(Wire.available()) {
    buffer[i++] = Wire.read(); // receive a byte
    length = i; // save length
}
```

## Output setup
Setting GPIO 0 and 1 as output

> [!IMPORTANT]
> Each manipulation on bits 1-6 needs an active unlock bit.
```
Wire.beginTransmission(DEVICE_ADDRESS);
Wire.write(0x10);
Wire.write(0x82);
Wire.write(0x82);
Wire.endTransmission();
```

### DO
Changing GPIO 0 and 1 output status - GPIO 0 set Low - GPIO 1 set High

> [!TIP]
> Each consequent write to an output pin doesn't need an unlock bit.
```
Wire.beginTransmission(DEVICE_ADDRESS);
Wire.write(0x10);
Wire.write(0x00);
Wire.write(0x01);
Wire.endTransmission();
```

### PWM
Setting PWM 1 and 2 value - PWM 1 to 50 - PWM 2 to 200
```
Wire.beginTransmission(DEVICE_ADDRESS);
Wire.write(0x15);
Wire.write(0x8A);
Wire.write(0x8A);
Wire.endTransmission();

Wire.beginTransmission(DEVICE_ADDRESS);
Wire.write(0x27);
Wire.write(0x32);
Wire.write(0xC8);
Wire.endTransmission();
```

### CNT
Reading GPIO 2 input frequency - Using default settings of `0x07`
```
Wire.beginTransmission(DEVICE_ADDRESS);
Wire.write(0x12);
Wire.write(0x90);
Wire.endTransmission();
...

uint8_t i = 0;
uint16_t time, freq;
float freq;

Wire.beginTransmission(DEVICE_ADDRESS);
Wire.write(0x3C);
Wire.endTransmission(false); // don't send stop
Wire.requestFrom(DEVICE_ADDRESS, 2, 1); // send stop after 2 byte rx

while(Wire.available()) {
    buffer[i++] = Wire.read(); // receive a byte
    length = i; // save length
}

time = (buffer[0] << 8) | buffer[1];
freq = 2000000 / time; // MCU_CLOCK / 8 = 2e6 Hz
```
