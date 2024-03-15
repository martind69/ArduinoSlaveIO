/*

  Universal Transceiver I/O Extender
  Atmel ATMega328P aka Arduino Nano

  Copyright (c) 2024, Martin Dittrich
  All rights reserved.

*/

#define DEVICE_ADDRESS 0x50
#define DEVICE_ID 0x27
#define DEVICE_PIN_MAX 21
#define DEVICE_ADC_OFFSET 14
#define DEVICE_ADC_MAX 21
#define DEVICE_PWM_AMOUNT 6
#define REGISTER_LENGTH 0xFF
#define REGISTER_PIN_OFFSET 0x10
#define REGISTER_PWM_OFFSET 0x26
#define REGISTER_ADC_OFFSET 0x2C
#define BUFFER_LENGTH 32
#define TX_LENGTH 16
#define PIN_OFFSET 0x01
#define DIR_OFFSET 0x02
#define PUR_OFFSET 0x04
#define PWM_OFFSET 0x08
#define UNLOCK_BIT 0x80
#define DEBUG 1

#include <Arduino.h>
#include <Wire.h>

uint8_t   buffer[BUFFER_LENGTH],
          device[REGISTER_LENGTH],
          transferLength,
          pwmMapping[DEVICE_PWM_AMOUNT] = {
            3, 5, 6, 9, 10, 11
          };
byte      availlable = false;

/*
  callback on requests by master
  START|ADDRESS|R|ACK|DATA|NACK|STOP
    |      |    |       |    |   |
    master is sender    |    master is sender
                        slave is sender
*/
void requestCallback() {
  static uint8_t i, address, *offset;
  if(transferLength == 1) { // check if write modify (master sends only register address) read was consistent
    address = buffer[0];
    offset = &device[address];
    Wire.write(offset, TX_LENGTH); // write until nack is received
    if(DEBUG) {
      Serial.print("I2C stream dump [Sr]-");
      Serial.print(address, HEX);
      Serial.print("-[1-A]-");
      for(i = 0; i < TX_LENGTH; i ++) {
        Serial.print(offset[i], HEX);
        Serial.print("-");
      }
      Serial.print("[NA-P], ");
      Serial.print(TX_LENGTH);
      Serial.println(" bytes send");
    }
  }
}

/*
  callback to receive data from master
  START|ADDRESS|W|ACK|REG|ACK|DATA|...|ACK|STOP
    |      |    |   \  |   |                |
    master is sender | |   slave is sender  |
                     | master is sender ----'
                     slave is sender
*/
void receiveCallback(int length) {
  static uint8_t address, i, j, pin, lat, dir, pur, pwm;
  transferLength = length;
  for(i = 0; i < transferLength; i ++) {
    buffer[i] = Wire.read(); // receive a byte
  }
  address = buffer[0];
  if(transferLength == 1) { // check if write was a request
    if(DEBUG) {
      Serial.print("I2C stream dump [S]-");
      Serial.print(buffer[0], HEX);
      Serial.print("-[0-A], ");
      Serial.print(transferLength);
      Serial.println(" byte register request received");
    }
  }
  else if(transferLength > 1) {
    switch(address) {
      // single pin registers
      case 0x10 ... 0x25:
        pin = buffer[0] - REGISTER_PIN_OFFSET;
        if(buffer[1] & UNLOCK_BIT) { // checking unlock bit
          device[address] = buffer[1] & (PWM_OFFSET | PUR_OFFSET | DIR_OFFSET);
          dir = buffer[1] & DIR_OFFSET;
          pur = buffer[1] & PUR_OFFSET;
          pwm = buffer[1] & PWM_OFFSET;
          if(DEBUG) {
            Serial.print("Pin ");
            Serial.print(pin);
            Serial.print(" set as ");
            if(dir == 0 && pur == 0) Serial.println("INPUT");
            else if(dir == 0 && pur == 1) Serial.println("INPUT_PULLUP");
            else if(dir == 1 && pwm == 1) Serial.println("OUTPUT_PWM");
            else if(dir == 1) Serial.println("OUTPUT");
          }
          else {
            if(dir == 0 && pur == 0) pinMode(pin, INPUT); // set input
            else if(dir == 0 && pur == 1) pinMode(pin, INPUT_PULLUP); // set input pullup
            else if(dir == 1) pinMode(pin, OUTPUT); // set output
          }
        }
        else { // writing latches
          lat = buffer[1] & PIN_OFFSET;
          if(DEBUG) {
            Serial.print("Pin ");
            Serial.print(pin);
            Serial.print(" set ");
            if(lat == 0) Serial.println("LOW");
            else if(lat == 1) Serial.println("HIGH");
          }
          else {
            device[address] = buffer[1] & PIN_OFFSET;
          }
        }
        break;
      // pwm registers
      case 0x26 ... 0x2B:
        device[address] = buffer[1];
        break;
    }
    if(DEBUG) {
      Serial.print("I2C stream dump [S]-");
      for(i = 0; i < transferLength; i ++) {
        Serial.print(buffer[i], HEX);
        i < 1 ? Serial.print("-[0-A]-") : (i + 1) < transferLength ? Serial.print("-[A]-") : Serial.print("-[A-");
      }
      Serial.print("P], ");
      Serial.print(transferLength);
      Serial.println(" bytes written to register");
    }
  }
  availlable = true;
}

/*
  setup block
*/
void setup(void) {
  Serial.begin(115200); // start serial for output
  Serial.print("Startup device with slave address 0x");
  Serial.println(DEVICE_ADDRESS, HEX);
  if(DEBUG) Serial.println("Application in debug-mode (dry-run)");

  Wire.begin(DEVICE_ADDRESS); // start i2c slave mode
  Wire.onRequest(requestCallback);
  Wire.onReceive(receiveCallback);

  pinMode(LED_BUILTIN, OUTPUT);
  analogReference(DEFAULT); // set analog reference to 5 V

  device[0x00] = DEVICE_ID;
}

/*
  loop block
*/
void loop(void) {
  static uint8_t i, *offset;
  static uint16_t value;
  // read operations
  offset = &device[REGISTER_PIN_OFFSET];
  for(i = 0; i <= DEVICE_PIN_MAX; i ++) {
    value = digitalRead(i); // read 1 bit, store at position 0
    bitWrite(offset[i], 0, value);
  }
  offset = &device[REGISTER_ADC_OFFSET];
  for(i = 0; i <= (DEVICE_ADC_MAX - DEVICE_ADC_OFFSET); i ++) {
    value = analogRead(i); // read 10 bit, store right justified format
    offset[i * 2] = highByte(value);
    offset[i * 2 + 1] = lowByte(value);
  }
  // write operations
  offset = &device[REGISTER_PIN_OFFSET];
  for(i = 0; i <= DEVICE_PIN_MAX; i ++) {
    if(device[REGISTER_PIN_OFFSET + i] & DIR_OFFSET) { // check if pin has output latch enabled
      digitalWrite(i, offset[i] & PIN_OFFSET); // set latch bit
    }
  }
  offset = &device[REGISTER_PWM_OFFSET];
  for(i = 0; i < DEVICE_PWM_AMOUNT; i ++) {
    if(device[REGISTER_PIN_OFFSET + pwmMapping[i]] & PWM_OFFSET) { // check if pin has pwm enabled
      analogWrite(pwmMapping[i], offset[i]); // set pwm duty
    }
  }
  // blink on update
  if(availlable) {
    availlable = false;
    digitalWrite(LED_BUILTIN, HIGH);
    delay(100);
    digitalWrite(LED_BUILTIN, LOW);
  }
}
