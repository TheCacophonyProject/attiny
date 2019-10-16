
// Code to run on a attiny85 connected to a raspberry pi through i2c.
// The two main functions of this is to power off the raspberry pi during the day. This is done by the raspberry pi telling the attiny through i2c how long to be turned off for.
// The attiny then turns the pi off using a mosfet. And will power it back on when needed.
// The second function is to work as a WDT for the raspberry Pi when is is runnign through the nihgt. So if the pi doesn't reset the WDT the attiny will power off and on the pi.

// There is a power LED that will pulse when the code is running. This LED can also blink to help debugging. Below are a list of what the different number of blinks mean.
// 1 Blink: Reset Pi WDT.
// 2 Blinks: Finished setup.
// 3 Blinks: Pi WDT failed.
// 4 Blinks: Starting Pi sleep.
// 5 Blinks: Set register to read from
// 10 Blinks: Error with reading i2c message.

#include "TinyWireS.h" //https://github.com/rambo/TinyWire
#include <avr/interrupt.h>
#include <avr/io.h>
#include <avr/wdt.h>
#include <avr/sleep.h>
#include "RunningAverage.h" //https://github.com/RobTillaart/Arduino/tree/master/libraries/RunningAverage

#define VERSION 4

#define I2C_SLAVE_ADDRESS 0x04 // Address of the slave
#define POWER_LED 1
#define PI_POWER_PIN 3
#define PI_POWER_OFF_WAIT_TIME 3000 // 30 seconds. = 30/0.01 as each tick is 0.01 seconds.
#define MINUTE_COUNTDOWN 108 // = (60/0.5)*90% as each interupt occurs every 0.5 seconds. -10% because of inaccurate internal clock
#define PI_WDT_RESET_VAL 30000 // 5 minutes 100*60*5
#define BATTERY_VOLTAGE_PIN A2


// VOLTAGE_POWER_SUPPLY is the maximum voltage reading when powered from a power supply
// VOLTAGE_EMPTY_BATTERY is the voltage of an empty battery
// VOLTAGE_LOW_BATTERY is the voltage needed to reach to turn back on after reaching VOLTAGE_EMPTY_BATTERY

// 3.3V LiFePO4
#define VOLTAGE_POWER_SUPPLY 32
#define VOLTAGE_EMPTY_BATTERY 63
#define VOLTAGE_LOW_BATTERY 141

// 3.3V Li-ion
/*
#define VOLTAGE_POWER_SUPPLY 32
#define VOLTAGE_EMPTY_BATTERY 369
#define VOLTAGE_LOW_BATTERY 455
*/

// 5V LiFePO4
/*
#define VOLTAGE_POWER_SUPPLY 434
#define VOLTAGE_EMPTY_BATTERY 470
#define VOLTAGE_LOW_BATTERY 522
*/

volatile uint16_t piSleepTime = 0; // Counting down the time until the pi will be turned on in minutes. If this is 0 the pi will be powered on.
volatile uint8_t minuteCountdown = MINUTE_COUNTDOWN;
unsigned int piWDTCountdown = PI_WDT_RESET_VAL;
volatile bool wdt_interrupt_f = false;
volatile bool onWiFi = false;
volatile bool gotWDPing = false;


RunningAverage batteryRA(20);
uint16_t batteryVoltageI2c;  // Battery voltage reading for I2c

volatile uint8_t blinks = 0;
enum State {
  PI_POWERED,
  PI_POWER_OFF_WAIT,        // Wait 30 seconds before powering off the Pi
  PI_SLEEPING,              // RPi is sleeping, 
  PI_WDT_FAILED             // WDT for RPi failed. Turn off and on.
};
State state = PI_POWERED;

#define I2C_READ_REG_LEN 3
#define I2C_READ_BATTERY_VOLTAGE_LO 0x20
#define I2C_READ_BATTERY_VOLTAGE_HI 0x21
#define I2C_READ_VERSION 0x22



volatile byte i2cReadRegVal = 0x00;
volatile byte i2cReadRegs[I2C_READ_REG_LEN] =
{
    I2C_READ_BATTERY_VOLTAGE_LO,
    I2C_READ_BATTERY_VOLTAGE_HI,
    I2C_READ_VERSION
};



void setup() {
  pinMode(BATTERY_VOLTAGE_PIN, INPUT);
  pinMode(PI_POWER_PIN, OUTPUT);
  pinMode(POWER_LED, OUTPUT);
  digitalWrite(PI_POWER_PIN, LOW);
  digitalWrite(POWER_LED, LOW);
  checkBattery();
  digitalWrite(POWER_LED, LOW);
  initTimer1();
  sei(); // enable interupts
  TinyWireS.begin(I2C_SLAVE_ADDRESS);
  TinyWireS.onReceive(receiveEvent);
  TinyWireS.onRequest(requestEvent);
  setBlinks(2);
}

void checkBattery() {
  batteryRA.addValue(analogRead(BATTERY_VOLTAGE_PIN));
  int a = batteryRA.getAverage();
  if (VOLTAGE_POWER_SUPPLY <= a && a <= VOLTAGE_EMPTY_BATTERY) {           // If voltage reading is lower than EMPTY_BATTERY and higher than POWER_SUPPLY then the battery is likely empty.
    setup_watchdog_interrpt();          // Use WDT for waking up device every 8 seconds.
    digitalWrite(PI_POWER_PIN, LOW);    // Single long LED flash to indicate low battery
    digitalWrite(POWER_LED, HIGH);
    delay(1000);
    digitalWrite(POWER_LED, LOW);
    while (VOLTAGE_POWER_SUPPLY <= a && a <= VOLTAGE_LOW_BATTERY) {      // Wait for the battery voltage to rise a to LOW_BATTERY before turning back on.
      wdt_interrupt_f = true;
      set_sleep_mode(SLEEP_MODE_PWR_DOWN);
      sleep_enable();
      sleep_mode();                     // Wait for WDT Interrupt
      sleep_disable();
      delay(100);
      a = analogRead(BATTERY_VOLTAGE_PIN);
    }
  }
  digitalWrite(PI_POWER_PIN, HIGH);
  wdt_enable(WDTO_8S);
}

void setup_watchdog_interrpt() {
  // For more detail see section 8.4 of the ATtiny 85 datasheet http://ww1.microchip.com/downloads/en/DeviceDoc/Atmel-2586-AVR-8-bit-Microcontroller-ATtiny25-ATtiny45-ATtiny85_Datasheet.pdf
  WDTCR |= (1 << WDCE) | (1 << WDE);  // These bits have to be set before disabling the WDT (have to disable the WDT restart to enable the WDT interrupt)
  WDTCR = (1 << WDCE) | (1 << WDP3) | (1 << WDP0);  // Disable the WDT and set the interval to 8 seconds
  WDTCR |= (1 << WDIE); // This will enable the WDT interrupt (not WDT restart).
}

ISR(WDT_vect) {
  if (!wdt_interrupt_f) {
    wdt_enable(WDTO_15MS);  // Restart if this flag is not set.
    while(1) {};
  }
  wdt_interrupt_f = false;
}


void loop() {
  checkBattery();
  switch(state) {
    case PI_POWERED:
      digitalWrite(PI_POWER_PIN, HIGH);
      break;
    case PI_POWER_OFF_WAIT:
      set_timer_if_not_running(PI_POWER_OFF_WAIT_TIME);
      if (timer_finished()) {
        state = PI_SLEEPING;
      }
      break;
    case PI_SLEEPING:
      digitalWrite(PI_POWER_PIN, LOW);
      cli();
      if (piSleepTime == 0) {
        state = PI_POWERED;
      }
      sei();
      break;
    case PI_WDT_FAILED:
      digitalWrite(PI_POWER_PIN, LOW);
      set_timer_if_not_running(1000); // wait 10 seconds.
      if (timer_finished()) {
        piWDTCountdown = PI_WDT_RESET_VAL;
        state = PI_POWERED;
        setBlinks(3);
      }
      break;
  }
  if (piWDTCountdown <= 0) {
    state = PI_WDT_FAILED;
  }
  tws_delay(10);
  wdt_reset();
  TinyWireS_stop_check();
  update_power_led();
  timer_tick();
  pi_wdt_tick();
}

void pi_wdt_tick() {
  if (state != PI_POWERED) {
    return;
  }
  if (piWDTCountdown >= 1) {
    piWDTCountdown--;
  } else {
    state = PI_WDT_FAILED;
  }
}

int t = -1;  // -1 = no timer running, 0 = timer finished. After timer finished is checked it gets set to -1 again.

void set_timer_if_not_running(int x) {
  if (t == -1) {
    t = x;
  }
}

bool timer_finished() {
  if (t == 0) {
    t = -1;
    return true;
  } else {
    return false;
  }
}

void timer_tick() {
  if (t >=1) {
    t--;
  }
}

// Gets called when the ATtiny receives an i2c request
void requestEvent() {
  switch(i2cReadRegVal) {
    case 0x00:
      TinyWireS.send(0x03);
      break;
    case I2C_READ_BATTERY_VOLTAGE_LO:
      batteryVoltageI2c = batteryRA.getAverage();
      TinyWireS.send(batteryVoltageI2c & 0xff);
      break;
    case I2C_READ_BATTERY_VOLTAGE_HI:
      TinyWireS.send(batteryVoltageI2c >> 8);
      break;
    case I2C_READ_VERSION:
      TinyWireS.send(VERSION);
      break;
  }
  i2cReadRegVal = 0x00;
}

void receiveEvent(uint8_t howMany) {
  if (howMany < 1) {
    // Message too short.
    setBlinks(10);
    return;
  }
  bool successfulRead = false;
  byte l;
  byte h;
  byte val = TinyWireS.receive();
  switch (val) {
    case 0x11:
      if (howMany != 3) {
          break;
      }
      h = TinyWireS.receive();
      l = TinyWireS.receive();
      piSleepTime = (h << 8) + l;
      minuteCountdown = MINUTE_COUNTDOWN;
      setBlinks(4);
      state = PI_POWER_OFF_WAIT;
      successfulRead = true;
      break;
    case 0x12:
      if (howMany != 1) {
        break;
      }
      gotWDPing = true;
      setBlinks(1);
      piWDTCountdown = PI_WDT_RESET_VAL;
      successfulRead = true;
      break;
    case 0x13:
      if (howMany != 2) {
        break;
      }
      onWiFi = TinyWireS.receive() == 0x01;
      successfulRead = true;
      break;
     default:
      if (isI2cReadReg(val)) {
        successfulRead = true;
        i2cReadRegVal = val;
        setBlinks(5);
      }
      break;
  }

  while(TinyWireS.available()) {
    TinyWireS.receive();
  }
  if (!successfulRead) {
    setBlinks(10);
  }
}

// Usefull when debugging
void testBlink(int x) {
  while(x--) {
    wdt_reset();
    digitalWrite(POWER_LED, LOW);
    tws_delay(200);
    digitalWrite(POWER_LED, HIGH);
    tws_delay(200);
    digitalWrite(POWER_LED, LOW);
    tws_delay(200);
  }
}

void setBlinks(int i) {
  if (blinks <= 0) {
    blinks = i;
  }
}

//========POWER PIN=========
// Slowly turns the LED on and off
int powerLedState = 1;  // Pulsing on or off
int powerLedIntensity = 0;
int blinkTimer = 0;

void update_power_led() {
  blinkTimer++;
  switch(powerLedState){
    case 0: // Pulsing on 
      powerLedIntensity++;
      analogWrite(POWER_LED, powerLedIntensity);
      if (powerLedIntensity >= 150) {
        powerLedState = 1;
      }
      break;
    case 1: // Pulsing off
      powerLedIntensity--;
      analogWrite(POWER_LED, powerLedIntensity);
      if (powerLedIntensity <= 1) {
        if (!gotWDPing) {
          blinks = 2;
        }
        else if (!onWiFi) {
          blinks = 1;
        }
        if (blinks >= 1) {
          powerLedState = 2; // Starting the blink here makes it easier to count the blinks
          blinkTimer = 0;
        } else {
          powerLedState = 0;
        }
      }
      break;
    case 2: // New blink wait.
      digitalWrite(POWER_LED, LOW);
      if (blinkTimer >= 50) {
        blinkTimer = 0;
        powerLedState = 3;
      }
      break;
    case 3: // Blink on
      digitalWrite(POWER_LED, HIGH);
      if (blinkTimer >= 30) {
        blinkTimer = 0;
        powerLedState = 4;
      }
      break;
    case 4: // Blink off
      digitalWrite(POWER_LED, LOW);
      if (blinkTimer >= 40) {
        blinkTimer = 0;
        blinks--;
        if (blinks <= 0) {
          powerLedState = 5;
        } else {
          powerLedState = 3;
        }
      }
      break;
    case 5:
      if (blinkTimer >= 100) {
        powerLedState = 0;
      }
      break;
    default:
      powerLedState = 0;
      break;
  }
}

//=======TIMER=======
// This timer interrupt is used to countdown the piSleepTime. 

void initTimer1() {
  TCCR1 |= (1 << CTC1);  // clear timer on compare match
  TCCR1 |= (1 << CS13) | (1 << CS12) | (1 << CS11) | (1 << CS10); //clock prescaler 16384
  OCR1C = 244; // 0.5*8MHz/16384=244.1 Will interrupt every 0.5 seconds
  TIMSK |= (1 << OCIE1A); // enable compare match interrupt
}

ISR(TIMER1_COMPA_vect) {
  if(minuteCountdown <= 0) {
    minuteCountdown = MINUTE_COUNTDOWN;
    if (piSleepTime > 0) {
      piSleepTime--;
    }
  } else {
    minuteCountdown--;
  }
}

bool isI2cReadReg(byte val) {
  for (int i = 0; i < I2C_READ_REG_LEN; i++) {
    if (i2cReadRegs[i] == val) {
      return true;
    }
  }
  return false;
}
