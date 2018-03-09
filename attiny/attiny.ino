#include <TinyWireS.h>
#include <avr/interrupt.h>
#include <avr/io.h>

#define I2C_SLAVE_ADDRESS 0x04 // Address of the slave
#define POWER_LED 1
#define PI_POWER_PIN 4 
#define PI_POWER_OFF_WAIT_TIME 1000 // 10 seconds. = 10/0.01 as each tick is 0.01 seconds.
#define MINUTE_COUNTDOWN 108 // = (60/0.5)*90% as each interupt occurs every 0.5 seconds. -10% because of inaccurate internal clock


int piPowerOffWait = PI_POWER_OFF_WAIT_TIME;
uint16_t piSleepTime = 0; // Counting down the time until the pi will be turned on in minutes. If this is 0 the pi will be powered on.
volatile int minuteCountdown = MINUTE_COUNTDOWN;

void setup() {
  initTimer1();
  sei(); // enable interupts
  pinMode(POWER_LED, OUTPUT);
  TinyWireS.begin(I2C_SLAVE_ADDRESS);
  TinyWireS.onReceive(receiveEvent);
  TinyWireS.onRequest(requestEvent);
  blink(2);
}
 
void loop() { 
  updatePiPower();
  TinyWireS_stop_check();
  updatePowerPin();
  tws_delay(10);
}

void updatePiPower(){
  if (piSleepTime >= 1) {
    // Pi should be off
    if (piPowerOffWait <= 0) {
      digitalWrite(PI_POWER_PIN, LOW);
    } else {
      piPowerOffWait--;
    }
  } else {
    // Pi should be on
    digitalWrite(PI_POWER_PIN, HIGH);
    piPowerOffWait = PI_POWER_OFF_WAIT_TIME;
  }
}
 
// Gets called when the ATtiny receives an i2c request
void requestEvent() {
  TinyWireS.send(0x10);
}

void receiveEvent(uint8_t howMany) {
  howMany--;
  switch(TinyWireS.receive()) {
    case 0x11:
      minuteCountdown = MINUTE_COUNTDOWN;
      byte h = TinyWireS.receive();
      byte l = TinyWireS.receive();
      cli();
      piSleepTime = (h << 8) + l;
      sei();
  }
}


void blink(int x) {
  while(x--) {
    digitalWrite(POWER_LED, LOW);
    tws_delay(200);
    digitalWrite(POWER_LED, HIGH);
    tws_delay(200);
    digitalWrite(POWER_LED, LOW);
    tws_delay(200);
  }
}

//========POWER PIN=========
// Slowly turns the LED on and off
int powerPinState = 0;
int powerPinIntensity = 0;

void updatePowerPin() {
  switch(powerPinState){
    case 0:
      powerPinIntensity++;
      if (powerPinIntensity >= 255) {
        powerPinState = 1;
      }
      break;
    case 1:
     powerPinIntensity--;
      if (powerPinIntensity <= 0) {
        powerPinState = 0;
      }
      break;
  }
  analogWrite(POWER_LED, powerPinIntensity);
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
    minuteCountdown = 120; // =60/0.5 because interrupt every 0.5 seconds
    if (piSleepTime > 0) {
      piSleepTime--;
    }
  } else {
    minuteCountdown--;
  }
}


