/*

  Universal Transceiver I/O Extender
  Atmel ATMega328P aka Arduino Nano

  https://github.com/martind69/ArduinoSlaveIO
  Copyright (c) 2024, Martin Dittrich
  All rights reserved.

*/

#define MCU_CLOCK 16000000
#define DEVICE_ADDRESS 0x50
#define DEVICE_ID 0x27
#define DEVICE_PIN_AMOUNT 22
#define DEVICE_ADC_AMOUNT 6
#define DEVICE_PWM_AMOUNT 6
#define DEVICE_CNT_AMOUNT 2
#define BUFFER_LENGTH 0x21
#define REGISTER_LENGTH 0xFF
#define REGISTER_PIN_OFFSET 0x10
#define REGISTER_PWM_OFFSET 0x26
#define REGISTER_ADC_OFFSET 0x2C
#define REGISTER_CNT_OFFSET 0x3C
#define PIN_OFFSET 0x01
#define DIR_OFFSET 0x02
#define PUR_OFFSET 0x04
#define PWM_OFFSET 0x08
#define CNT_OFFSET 0x10
#define UNLOCK_BIT 0x80
#define DEVICE_TX_MAX 16
#define DEBUG 0

#include <Arduino.h>
#include <Wire.h>

uint8_t     buffer[BUFFER_LENGTH],
            transferLength,
            error = false,
            availlable = false,
            adcMapping[DEVICE_ADC_AMOUNT] = {
              14, 15, 16, 17, 20, 21
            },
            pwmMapping[DEVICE_PWM_AMOUNT] = {
              3, 5, 6, 9, 10, 11
            },
            cntMapping[DEVICE_CNT_AMOUNT] = {
              2, 3
            };

volatile uint8_t    device[REGISTER_LENGTH] = { 0 };

/*
  global helper functions
*/
uint8_t containsInArray(uint8_t *array, uint8_t value) {
  uint8_t result;
  for(uint8_t available : array) {
    if(value == available) {
      result = true;
      break;
    }
    else result = false;
  }
  return result;
}

/*
  callback on requests by master
  START|ADDRESS|R|ACK|DATA|NACK|STOP
    |      |    |       |    |   |
    master is sender    |    master is sender
                        slave is sender
*/
void requestCallback() {
  static uint8_t i, address, *offset;
  if(transferLength == 1) { // check if write modify (master only sends register address) read was consistent
    address = buffer[0];
    offset = &device[address];
    Wire.write(offset, DEVICE_TX_MAX); // write until nack is received
    if(DEBUG) {
      Serial.print("I2C stream dump [Sr]-");
      Serial.print(address, HEX);
      Serial.print("-[1-A]-");
      for(i = 0; i < DEVICE_TX_MAX; i ++) {
        Serial.print(offset[i], HEX);
        Serial.print("-");
      }
      Serial.print("[NA-P], probably ");
      Serial.print(DEVICE_TX_MAX);
      Serial.println(" bytes were sent");
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
  static uint8_t i, address, pin, ulb, cnt, pwm, pur, dir, lat;
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
    switch(address) {
      // single pin registers
      case 0x10 ... 0x25:
        pin = buffer[0] - REGISTER_PIN_OFFSET;
        for(i = 1; i < transferLength; i ++) {
          ulb = buffer[i] & UNLOCK_BIT;
          cnt = buffer[i] & CNT_OFFSET;
          pwm = buffer[i] & PWM_OFFSET;
          pur = buffer[i] & PUR_OFFSET;
          dir = buffer[i] & DIR_OFFSET;
          lat = buffer[i] & PIN_OFFSET;
          error = false;
          // checking unlock bit
          if(ulb) {
            device[address] = buffer[i] & (CNT_OFFSET | PWM_OFFSET | PUR_OFFSET | DIR_OFFSET);
            // check mapping
            if(pwm && !containsInArray(pwmMapping, pin)) {
              error = true;
            }
            else if(cnt && !containsInArray(cntMapping, pin)) {
              error = true;
            }
            if(DEBUG) {
              Serial.print("Pin ");
              Serial.print(pin);
              Serial.print(" set as ");
              if(!dir && !pur && !pwm && !cnt) Serial.println("INPUT");
              else if(!dir && pur && !pwm && !cnt) Serial.println("INPUT_PULLUP");
              else if(!dir && !pwm && cnt) Serial.println("INPUT_COUNTER");
              else if(dir && !pur && !pwm && !cnt) Serial.println("OUTPUT");
              else if(dir && !pur && pwm && !cnt) Serial.println("OUTPUT_PWM");
              else Serial.println("UNKNOWN!");
              if(error) Serial.println("Error PIN_OUTSIDE_MAPPING");
            }
            // set pinMode
            else if(!error) {
              if(!dir && !pur && !pwm) pinMode(pin, INPUT); // set input
              else if(!dir && pur && !pwm) pinMode(pin, INPUT_PULLUP); // set input pull-up
              else if(dir && !pur && !cnt) pinMode(pin, OUTPUT); // set output
              if(!dir && !pwm && cnt) { // set input counter
                timerInit(); // setup timer registers
                // attach interrupt-vect. to pin. @tbd: no detach implemented, reset slave device!
                if(pin == cntMapping[0]) attachInterrupt(digitalPinToInterrupt(pin), &intCallback, device[0x07] & 0x03);
                else if(pin == cntMapping[1]) attachInterrupt(digitalPinToInterrupt(pin), &intCallback, (device[0x07] & 0x0C) >> 2);
              }
            }
          }
          // writing latches
          else if(dir) {
            if(DEBUG) {
              Serial.print("Pin ");
              Serial.print(pin);
              Serial.print(" set ");
              if(lat) Serial.println("HIGH");
              else Serial.println("LOW");
            }
            else if(!error) {
              bitWrite(device[address], 0, lat);
            }
          }
          // all other scenarios
          else {
            error = true;
            if(DEBUG) Serial.println("Error CONFIG_MISMATCH");
          }
          pin ++; // next pin
        }
        break;
      // pwm registers
      case 0x26 ... 0x2B:
        for(i = 1; i < transferLength; i ++) {
          if(DEBUG) {
            Serial.print("PWM register 0x");
            Serial.print(address, HEX);
            Serial.print(" updated to 0x");
            Serial.println(buffer[i], HEX);
          }
          device[address] = buffer[i];
        }
        break;
    }
  }
  availlable = true;
}

/*
  interrupt callback
  read 16 bit, store in right-justified format
  note: reading the 16-bit value is done by first reading the low byte (TCNT1L) and then the high
  byte (TCNT1H). when the low byte is read the high byte is copied into the high byte temporary register (TEMP).
  when the CPU reads the TCNT1H location it will access the TEMP register!
  info: for flexible input multiplexing i'm not using the input capture unit!
*/
void intCallback() {
  uint8_t *offset;
  offset = &device[REGISTER_CNT_OFFSET];
  offset[1] = TCNT1L;
  offset[0] = TCNT1H;
  TCNT1 = 0; // reset TCNT1H/L
  if(DEBUG) Serial.println("Interrupt triggered!");
}

ISR(TIMER1_OVF_vect) {
  uint8_t *offset;
  offset = &device[REGISTER_CNT_OFFSET];
  offset[1] = 0xFF;
  offset[0] = 0xFF;
  if(DEBUG) Serial.println("Timer overflow occurred!");
}

/*
  timer setup
  MCU_CLOCK / 8 = 2e6 Hz | 500 ns default value
*/
void timerInit() {
  TCCR1A = 0x00; // mode = normal
  TCCR1B = device[0x07] & 0x70 >> 4; // prescaler
  TIMSK1 = device[0x07] & 0x80 >> 7; // overflow isr
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

  pinMode(LED_BUILTIN, OUTPUT); // set pin 13 as output
  analogReference(DEFAULT); // set analog reference to 5 V

  device[0x00] = DEVICE_ID;
  device[0x07] = 0xAA;
}

/*
  loop block
*/
void loop(void) {
  static uint8_t i, *offset;
  static uint16_t value;
  static uint32_t mtime;
  // read operations
  offset = &device[REGISTER_PIN_OFFSET];
  for(i = 0; i < DEVICE_PIN_AMOUNT; i ++) {
    value = digitalRead(i); // read 1 bit, store at position 0
    if(~offset[i] & DIR_OFFSET) { // check if pin is configured as input
      bitWrite(offset[i], 0, value);
    }
  }
  offset = &device[REGISTER_ADC_OFFSET];
  for(i = 0; i < DEVICE_ADC_AMOUNT; i ++) {
    value = analogRead(adcMapping[i]); // read 10 bit, store in right-justified format
    offset[i * 2] = highByte(value);
    offset[i * 2 + 1] = lowByte(value);
  }
  // write operations
  offset = &device[REGISTER_PIN_OFFSET];
  for(i = 0; i < DEVICE_PIN_AMOUNT; i ++) {
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
    mtime = millis();
    digitalWrite(LED_BUILTIN, HIGH);
  }
  else if(millis() > mtime + 100) {
    digitalWrite(LED_BUILTIN, LOW);
  }
  // error handling
  while(error) {}
}
