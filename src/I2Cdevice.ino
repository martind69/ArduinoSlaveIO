/*

  Universal Transceiver I/O Extender
  Atmel ATMega328P aka Arduino Nano

  Copyright (c) 2024, Martin Dittrich
  All rights reserved.

*/

#define MCU_CLOCK 16000000
#define DEVICE_ADDRESS 0x50
#define DEVICE_ID 0x27
#define DEVICE_PIN_MAX 21
#define DEVICE_ADC_OFFSET 14
#define DEVICE_ADC_MAX 21
#define DEVICE_PWM_AMOUNT 6
#define DEVICE_CNT_AMOUNT 2
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
#define BUFFER_LENGTH 32
#define TX_LENGTH 16
#define DEBUG 0

#include <Arduino.h>
#include <Wire.h>

uint8_t   buffer[BUFFER_LENGTH],
          transferLength,
          availlable = false,
          pwmMapping[DEVICE_PWM_AMOUNT] = {
            3, 5, 6, 9, 10, 11
          },
          cntMapping[DEVICE_CNT_AMOUNT] = {
            2, 3
          };

volatile uint8_t  device[REGISTER_LENGTH];

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
  static uint8_t i, address, error, pin, lat, dir, pur, pwm, cnt;
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
          // checking unlock bit
          if(buffer[i] & UNLOCK_BIT) {
            device[address] = buffer[i] & (CNT_OFFSET | PWM_OFFSET | PUR_OFFSET | DIR_OFFSET);
            dir = buffer[i] & DIR_OFFSET;
            pur = buffer[i] & PUR_OFFSET;
            pwm = buffer[i] & PWM_OFFSET;
            cnt = buffer[i] & CNT_OFFSET;
            error = false;
            // check mapping
            if(pwm) for(uint8_t available : pwmMapping) {
              if(pin == available) {
                error = false;
                break;
              } else error = true;
            }
            if(cnt) for(uint8_t available : cntMapping) {
              if(pin == available) {
                error = false;
                break;
              } else error = true;
            }
            if(DEBUG || error) {
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
            else {
              if(!dir && !pur && !pwm) pinMode(pin, INPUT); // set input
              else if(!dir && pur && !pwm) pinMode(pin, INPUT_PULLUP); // set input pull-up
              else if(dir && !pur && !cnt) pinMode(pin, OUTPUT); // set output
              if(!dir && !pwm && cnt) { // set input counter
                timerInit(); // setup timer registers
                attachInterrupt(digitalPinToInterrupt(pin), &intCallback, FALLING); // attach interrupt vect to pin. note: no detach implemented, reset slave device!
              }
            }
          }
          // writing latches
          else {
            lat = buffer[i] & PIN_OFFSET;
            if(DEBUG) {
              Serial.print("Pin ");
              Serial.print(pin);
              Serial.print(" set ");
              if(!lat) Serial.println("LOW");
              else Serial.println("HIGH");
            }
            else {
              bitWrite(device[address], 0, lat);
            }
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
  MCU_CLOCK / 8 = 2e6 Hz | 500 ns
*/
void timerInit() {
  TCCR1A = 0x00; // mode = normal
  TCCR1B = 0x02; // prescaler = 8
  TIMSK1 = 0x01; // overflow isr
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
  for(i = 0; i <= DEVICE_PIN_MAX; i ++) {
    value = digitalRead(i); // read 1 bit, store at position 0
    if(~offset[i] & DIR_OFFSET) { // check if pin is configured as input
      bitWrite(offset[i], 0, value);
    }
  }
  offset = &device[REGISTER_ADC_OFFSET];
  for(i = 0; i <= (DEVICE_ADC_MAX - DEVICE_ADC_OFFSET); i ++) {
    value = analogRead(i); // read 10 bit, store in right-justified format
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
    mtime = millis();
    digitalWrite(LED_BUILTIN, HIGH);
  }
  else if(millis() > mtime + 100) {
    digitalWrite(LED_BUILTIN, LOW);
  }
}
