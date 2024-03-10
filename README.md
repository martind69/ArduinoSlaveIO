# Arduino I/O Extender
This project solves the problem of task centralization. The effort of coding all tasks into a single project can be difficult. So let's think about decentralization networks!

For modular programming we can use the Arduino I/O Extender. It works as a I2C slave device.

# Features

## Device-ID
Register Address `0x00` - R

Return Value `0x27`

## Digital: I/O
Input Register Address `0x10` to `0x25`

Output Register Address `0x01`

## Analog: ADC/DAC
Input Register Address `0xA0` to `0xA7`

## Waveform: PWM

## Counter: Time/Frequency measurement
> [!NOTE]
> This feature is not implementet yet
