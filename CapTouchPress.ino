// Wearable Touch, Pressure and Vibrotactile - LSR2018 (based on Dildx module - neurodildo)
// L. M. GOMES 02/07/2018

#include <SoftTimer.h> //threads
#include <avr/sleep.h> //deep sleep mode
#include <avr/power.h> //power mode management
#include <math.h>      //necessary for breathing led
#include <SoftwareSerial.h>

#include <MPR121.h>
#include <Wire.h>

#define numElectrodes 12


// pinout declaration
//IO
#define motorPin 3
#define fbtn     2
#define BTen     4
#define fsr1     A0
#define fsr2     A1
#define led      13   ///trocar para 5 depois

SoftwareSerial btSerial(10,11);   //RX , TX

#define debounce 200 //ms
#define totalModes 2 // plus 2 charging modes

//Vibration intensity
#define level0  0
#define level1  70
#define level2  128
#define level3  255


// Variables

int fsr1Val       = 0;
int fsr2Val       = 0;

char bufOut[21]     = {'0','0','0','0','0','0','0','0','0','0','0','0','0','0','0','0','0','0','0','0','\0'};

int mspeed        = 0;
int motorState    = 0;
int motorCount    = 0;

int count         = 0;
int interruptFlag = 0;

int modes         = 0;
int lastmodes     = 0;
int timeSleep     = 100;
//int timeConnect   = 80;

int btLevel = 0;

void callBack1(Task* me);
void callBack2(Task* me);
void callBack3(Task* me);
void callBack4(Task* me);

Task  t1(100, callBack1);     //main task, control motor speed
Task  t2(25, callBack2);      //counts fbtm pressed time to enter sleep mode
Task  t3(50, callBack3);      //check BT and change mode
Task  t4(25, callBack4);     //check pressure sensors and update values / touch electrodes

// here the interrupt is handled after wakeup
void wakeUpNow()
{
  // execute code here after wake-up before returning to the loop() function
  // timers and code using timers (serial.print and more...) will not work here.
  // we don't really need to execute any special functions here, since we
  // just want the thing to wake up
}

// fuction to change between vibration modes
void changeMode()
{
  static unsigned long last_millis = 0;
  unsigned long m = millis();
  if (m - last_millis < debounce)
  {  }
  else
  {
    modes = modes + 1;

    if (modes > totalModes)
    {
      modes = 1;
    }
  }
  last_millis = m;
}


void setup() {
  // put your setup code here, to run once:


  Serial.begin(9600);
  btSerial.begin(9600);

  Wire.begin();

  // turns on BT module
  pinMode(BTen, OUTPUT);
  digitalWrite(BTen,HIGH);
  
  // configure motor
  pinMode(motorPin, OUTPUT);
  digitalWrite(motorPin, LOW);

  // configure LED
  pinMode(led, OUTPUT);
  digitalWrite(led, LOW);


  // configures thread tasks
  SoftTimer.add(&t1);
  SoftTimer.add(&t2);
  SoftTimer.add(&t3);
  SoftTimer.add(&t4);

  // configure interrupt for the function button
  pinMode(fbtn, INPUT_PULLUP);
  attachInterrupt(0, changeMode, FALLING); // use interrupt 0 (pin 2) and run function
  interruptFlag = 1;

  // 0x5A is the MPR121 I2C address
  if(!MPR121.begin(0x5A)){ 
    Serial.println("error setting up MPR121");  
    switch(MPR121.getError()){
      case NO_ERROR:
        Serial.println("no error");
        break;  
      case ADDRESS_UNKNOWN:
        Serial.println("incorrect address");
        break;
      case READBACK_FAIL:
        Serial.println("readback failure");
        break;
      case OVERCURRENT_FLAG:
        Serial.println("overcurrent on REXT pin");
        break;      
      case OUT_OF_RANGE:
        Serial.println("electrode out of range");
        break;
      case NOT_INITED:
        Serial.println("not initialised");
        break;
      default:
        Serial.println("unknown error");
        break;      
    }
    while(1);
  }

  // pin 6 is the MPR121 interrupt 
  MPR121.setInterruptPin(6);

  // this is the touch threshold - setting it low makes it more like a proximity trigger
  // default value is 40 for touch
  MPR121.setTouchThreshold(40);
  
  // this is the release threshold - must ALWAYS be smaller than the touch threshold
  // default value is 20 for touch
  MPR121.setReleaseThreshold(20);  

  // initial data update
  MPR121.updateTouchData();


  // initialization ok
  helloDildo();

  delay(500);
  modes = 1;

}

// initialization function - see if it is working
void helloDildo() {
  analogWrite(motorPin, level3);
  delay(250);
  analogWrite(motorPin, 0);
}


//// breathing LED
//void breathingLED() {
//  float val = (exp(sin(millis() / 2000.0 * PI)) - 0.36787944) * 108.0;
//  leds[0] = CRGB::White;
//  leds[0].fadeLightBy(val);
//  FastLED.show();
//}
//
//void breathingLEDCharging() {
//  float val = (exp(sin(millis() / 2000.0 * PI)) - 0.36787944) * 108.0;
//  leds[0] = CRGB::Red;
//  leds[0].fadeLightBy(val);
//  FastLED.show();
//  //Serial.println(val);
//}

//void breathingDildo() {
//  float val = (exp(sin(millis() / 2000.0 * PI)) - 0.36787944) * 108.0;
//  analogWrite(motorPin, 255 - val);
//}

void breathingLed() {
  float val = (exp(sin(millis() / 2000.0 * PI)) - 0.36787944) * 108.0;
  analogWrite(led, 255 - val);
}

// here we put the arduino to sleep
void sleepNow()
{
  /* Now is the time to set the sleep mode. In the Atmega8 datasheet
     http://www.atmel.com/dyn/resources/prod_documents/doc2486.pdf on page 35
     there is a list of sleep modes which explains which clocks and
     wake up sources are available in which sleep mode.

     In the avr/sleep.h file, the call names of these sleep modes are to be found:

     The 5 different modes are:
         SLEEP_MODE_IDLE         -the least power savings
         SLEEP_MODE_ADC
         SLEEP_MODE_PWR_SAVE
         SLEEP_MODE_STANDBY
         SLEEP_MODE_PWR_DOWN     -the most power savings

     For now, we want as much power savings as possible, so we
     choose the according
     sleep mode: SLEEP_MODE_PWR_DOWN

  */
  //set_sleep_mode(SLEEP_MODE_PWR_DOWN);   // sleep mode is set here
  set_sleep_mode(SLEEP_MODE_ADC);   // sleep mode is set here

  //  power_spi_disable();
  //  power_usart0_disable();
  //  power_twi_disable();


  //power_timer1_enable();

  sleep_enable();          // enables the sleep bit in the mcucr register


  // so sleep is possible. just a safety pin

  /* Now it is time to enable an interrupt. We do it here so an
     accidentally pushed interrupt button doesn't interrupt
     our running program. if you want to be able to run
     interrupt code besides the sleep function, place it in
     setup() for example.

     In the function call attachInterrupt(A, B, C)
     A   can be either 0 or 1 for interrupts on pin 2 or 3.

     B   Name of a function you want to execute at interrupt for A.

     C   Trigger mode of the interrupt pin. can be:
                 LOW        a low level triggers
                 CHANGE     a change in level triggers
                 RISING     a rising edge of a level triggers
                 FALLING    a falling edge of a level triggers

     In all but the IDLE sleep modes only LOW can be used.
  */

  attachInterrupt(0, wakeUpNow, LOW); // use interrupt 0 (pin 2) and run function
  // wakeUpNow when pin 2 gets LOW

  sleep_mode();            // here the device is actually put to sleep!!
  // THE PROGRAM CONTINUES FROM HERE AFTER WAKING UP

  sleep_disable();         // first thing after waking from sleep:
  // disable sleep...
  detachInterrupt(0);      // disables interrupt 0 on pin 2 so the
  // wakeUpNow code will not be executed
  // during normal running time.
}

// state modes and motor activation
void callBack1(Task* me) {

  switch (modes) {
    case 0:
      //vibro off   led   on
      analogWrite(motorPin, level0);
      //setLED(defaultColor, LED0);
      lastmodes = 0;
      //Serial.println("modo 0");
      break;
      
    case 1:
      //Wait for BT connection mode

      if (lastmodes == 0){
        helloDildo();
      }      

//      digitalWrite(led,HIGH);
//      delay(200);
//      digitalWrite(led,LOW);
//      delay(200);
//      digitalWrite(led,HIGH);
//      delay(200);
//      digitalWrite(led,LOW);
//      delay(1000);
      
      lastmodes = 1;

      motorState    = 0;
      motorCount    = 0;

      //Serial.println("modo 1");
      break;
      
    case 8:

      // stop button actions
      detachInterrupt(digitalPinToInterrupt(fbtn));
      // disables motor
      analogWrite(motorPin, level0);
      // led indication of low power
      //setLED('r', LEDLOWBAT);

      //      charging = digitalRead(chargingPin);
      //      if (charging) {
      //        modes = 9;
      //      }

      lastmodes = 8;
      //Serial.println("modo 8");
      break;
      
    case 9:

      
      Serial.print("BtLevel");
      Serial.println(btLevel);

      lastmodes = 9;
      //Serial.println("modo 9");
      break;
    default:
      //vibro off   led   on
      analogWrite(motorPin, level0);
      //setLED(defaultColor, LED0);
      //Serial.println("modo 0");
      break;
  }
}

// check button and changes between modes
void callBack2(Task* me) {

  if (digitalRead(fbtn) == LOW) {
    count = count + 1;

    if (count == timeSleep) {
      count = 0;


      //setLED('k', 0);
      analogWrite(motorPin, 0);
      
      // turns off BT module when it sleeps
      pinMode(BTen, OUTPUT);
      digitalWrite(BTen,LOW);
      
      while (digitalRead(fbtn) == LOW) {}


      attachInterrupt(0, wakeUpNow, LOW); // use interrupt 0 (pin 2) and run function
      // wakeUpNow when pin 2 gets LOW


      //Serial.print("sleeping ");

      delay(100);
      sleepNow();

      helloDildo();
      delay(100);
      //setLED(defaultColor, 50);
      attachInterrupt(0, changeMode, FALLING);
      modes = 1;
      setup();

    }
  }
  else {
    count = 0;
  }
}

// BT monitoring
void callBack3(Task* me) {
  String c = "";
  if (btSerial.available()) {
    c = btSerial.readString();
    Serial.write(btSerial.read());
    if (c == "1") {
      modes = 8;
      btSerial.println("Connected!");
    }
    else {
      if (c == "2") {
        attachInterrupt(0, changeMode, FALLING);
        btSerial.println("Disconnected!");
        modes = 1;
      }
      else
      {
        modes = 9;
        btLevel = c.toInt();
      }
    }
  }
}


// pressure sensors monitoring
void callBack4(Task* me) {

  String packP = "";
  
  fsr1Val = analogRead(fsr1);
  fsr2Val = analogRead(fsr2);

  char buf[4];
  sprintf(buf,"%04i",fsr1Val);
  bufOut[12] = buf[0];
  bufOut[13] = buf[1];
  bufOut[14] = buf[2];
  bufOut[15] = buf[3];

  sprintf(buf,"%04i",fsr2Val);
  bufOut[16] = buf[0];
  bufOut[17] = buf[1];
  bufOut[18] = buf[2];
  bufOut[19] = buf[3];  

if(MPR121.touchStatusChanged()){
    MPR121.updateTouchData();
    for(int i=0; i<numElectrodes; i++){
      if(MPR121.isNewTouch(i)){
        //Serial.print("electrode ");
        //Serial.print(i, DEC);
        //Serial.println(" was just touched");  
        bufOut[i] = '1';
      } else if(MPR121.isNewRelease(i)){
        //Serial.print("electrode ");
        //Serial.print(i, DEC);
        //Serial.println(" was just released");  
        bufOut[i] = '0';
      }
    } 
  }

  //btSerial.println(bufOut);
  Serial.println(bufOut);
}

