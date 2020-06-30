#include <AccelStepper.h>
#include <MultiStepper.h>

/*********************************************************************
 This is an example for our nRF51822 based Bluefruit LE modules

 Pick one up today in the adafruit shop!

 Adafruit invests time and resources providing this open source code,
 please support Adafruit and open-source hardware by purchasing
 products from Adafruit!

 MIT license, check LICENSE for more information
 All text above, and the splash screen below must be included in
 any redistribution  
*********************************************************************/

#include <string.h>
#include <Arduino.h>
#include <SPI.h>
#include "Adafruit_BLE.h"
#include "Adafruit_BluefruitLE_SPI.h"
#include "Adafruit_BluefruitLE_UART.h"
#include <Wire.h>
#include <Adafruit_MotorShield.h>

#include "BluefruitConfig.h"

#if SOFTWARE_SERIAL_AVAILABLE
  #include <SoftwareSerial.h>
#endif

#define FACTORYRESET_ENABLE         1
#define MINIMUM_FIRMWARE_VERSION    "0.6.6"
#define MODE_LED_BEHAVIOUR          "MODE"
Adafruit_BluefruitLE_SPI ble(BLUEFRUIT_SPI_CS, BLUEFRUIT_SPI_IRQ, BLUEFRUIT_SPI_RST);

// Create the motor shield object with the default I2C address
Adafruit_MotorShield AFMS = Adafruit_MotorShield(); 

// Connect a stepper motor with 200 steps per revolution (1.8 degree)
// to motor port #1 (M3 and M4)
Adafruit_StepperMotor *myMotor = AFMS.getStepper(200, 1);

// A small helper
void error(const __FlashStringHelper*err) {
  Serial.println(err);
  while (1);
}

// declare variables
uint8_t readPacket(Adafruit_BLE *ble, uint16_t timeout);
float parsefloat(uint8_t *buffer);
void printHex(const uint8_t * data, const uint32_t numBytes);
int numberOfSteps = 200*32;
float wheelCircum = 395.841;
float stepDistance = wheelCircum/numberOfSteps; 
float fastStepDistance = wheelCircum/200;
int lastPressed = 0;
int motorStatus = 0;
int firstPress = 0;
int Distance= 100;
int TotalTime = 60;
long TotalSteps;
float LapseSpeed;
int debug=0;
bool isRunning=0;
long startTime;

int motorSpeed = 20;
float maxiSpeed = 20;
int motorAccel = 2000;
uint8_t stepType=MICROSTEP; 
#define VBATPIN A9
#define READ_BUFSIZE                    (20)
// the packet buffer
extern uint8_t packetbuffer[];


void forwardstep1() {  
  myMotor->onestep(FORWARD, stepType);
}
void backwardstep1() {  
  myMotor->onestep(BACKWARD, stepType);
}

AccelStepper stepper(forwardstep1, backwardstep1);

void setup(void)
{
  AFMS.begin();  // create with the default frequency 1.6KHz
  TWBR = ((F_CPU /400000l) - 16) / 2; // Change the i2c clock to 400KHz
  myMotor->setSpeed(25);
  myMotor->step(10, FORWARD, MICROSTEP); 
  myMotor->step(10, BACKWARD, MICROSTEP); 

  if ( !ble.begin(VERBOSE_MODE) )
  {
    error(F("Couldn't find Bluefruit, make sure it's in command mode & check wiring?"));
  }
  Serial.println( F("OK!") );

  if ( FACTORYRESET_ENABLE )
  {
    /* Perform a factory reset to make sure everything is in a known state */
    Serial.println(F("Performing a factory reset: "));
    if ( ! ble.factoryReset() ){
      error(F("Couldn't factory reset"));
    }
  }


  /* Disable command echo from Bluefruit */
  ble.echo(false);

  Serial.println("Requesting Bluefruit info:");
  /* Print Bluefruit information */
  ble.info();
  ble.verbose(false);  // debug info is a little annoying after this point!

  /* Wait for connection */
  while (! ble.isConnected()) {
      delay(500);
  }


  // LED Activity command is only supported from 0.6.6
  if ( ble.isVersionAtLeast(MINIMUM_FIRMWARE_VERSION) )
  {
    // Change Mode LED Activity
    Serial.println(F("Change LED activity to " MODE_LED_BEHAVIOUR));
    ble.sendCommandCheckOK("AT+HWModeLED=" MODE_LED_BEHAVIOUR);
  }

  // Set Bluefruit to DATA mode
  Serial.println( F("Switching to DATA mode!") );
  ble.setMode(BLUEFRUIT_MODE_DATA);

  stepper.setMaxSpeed(maxiSpeed);
  stepper.setSpeed(motorSpeed);
  stepper.setAcceleration(motorAccel);
  startTime = millis() + 10000;
}

void loop() {
  uint8_t len = readPacket(&ble, BLE_READPACKET_TIMEOUT);
  if (firstPress ==0) {
    if (millis() > startTime) {
      ble.println("Press any key to start...");
      startTime = millis() + 10000;
    }
  }
  if (len == 0) return;

  if (packetbuffer[1] == 'B') {
    uint8_t buttnum = packetbuffer[2] - '0';
    boolean pressed = packetbuffer[3] - '0';
    Serial.print ("Button "); Serial.print(buttnum);
    
    if (pressed) {
      if (firstPress == 0) {
        ble.println("Welcome to the Universal Dolly and Timelapse Rig, created by Mark Rhodes.");
        ble.println("Please select a mode: ");
        ble.println("1.  Smooth Dolly.");
        ble.println("2.  Fast Dolly.");
        ble.println("3.  Time Lapse.");
        firstPress = 1;
      } else {
        if (buttnum == 1) {
          firstPress = 0;
          ble.println();
          ble.println("Smooth Dolly Mode.");
          ble.println("Instructions");   
          ble.println("Left - Start/Stop Backward");
          ble.println("Right - Start/Stop Forward");
          ble.println("Up - Increase Speed 20%");
          ble.println("Down - Decrease Speed 20%");
          ble.println("1 - Increase Speed 1%");
          ble.println("3 - Decrease Speed 1%");
          ble.println("4 - Main Menu");
          stepType=MICROSTEP; 
          updateDollyDisp();
          smoothDolly();   
        } else if (buttnum == 2) {
          firstPress = 0;
          ble.println();
          ble.println("Fast Dolly Mode.");
          // Speeds in excess of 100% can be had, if you hack your libraries
          // I figure if you can see this comment, you can work it out.  If not, find me on social media.
          ble.println("Instructions");   
          ble.println("Left - Start/Stop Backward");
          ble.println("Right - Start/Stop Forward");
          ble.println("Up - Increase Speed 20%");
          ble.println("Down - Decrease Speed 20%");
          ble.println("1 - Increase Speed 1%");
          ble.println("3 - Decrease Speed 1%");
          ble.println("2 - Toggle StepType");
          ble.println("4 - Main Menu");
          stepType=SINGLE;
          updateFastDollyDisp();
          stepper.setAcceleration(500);
          fastDolly();
        } else if (buttnum == 3) {
          firstPress = 0;
          ble.println();
          ble.println("Timelapse Mode.");
          ble.println("Instructions");   
          ble.println("Left - Start/Stop Backward");
          ble.println("Right - Start/Stop Forward");
          ble.println("Up - Increase Distance 100mm");
          ble.println("Down - Decrease Distance 100mm");
          ble.println("1 - Add 1 hour");
          ble.println("3 - Subtract 1 hour");
          ble.println("2 - Add 5 minutes");
          ble.println("4 - Main Menu");
          stepType=MICROSTEP; 
          UpdateLapseDisplay();
          lapse();
        } else {
          ble.println();
          ble.println("Select Mode: ");
          ble.println("1.  Smooth Dolly.");
          ble.println("2.  Fast Dolly.");
          ble.println("3.  Time Lapse.");
        }
      }
    }
  }
}

void lapse() {
  uint8_t len = readPacket(&ble, 1);
  if (packetbuffer[1] == 'B') {
    uint8_t buttnum = packetbuffer[2] - '0';
    boolean pressed = packetbuffer[3] - '0';
    if (pressed) {
      if (lastPressed == buttnum && buttnum > 6) {
        stepper.stop();
        delay(1000);
        stepper.setCurrentPosition(0);
        stepper.moveTo(0);
        ble.println("Cancelled Time Lapse");
        myMotor->release();
        lastPressed = 0;
      } else if (buttnum == 8) {
        ble.println("Starting Forward Timelapse");
        stepper.setCurrentPosition(0);
        stepper.moveTo(TotalSteps*-1);
        isRunning=1;
      } else if (buttnum == 7) {
        ble.println("Starting Reverse Timelapse");
        stepper.setCurrentPosition(0);
        stepper.moveTo(TotalSteps);
        isRunning=1;
      } else if (buttnum == 5) {
        Distance += 100;
        UpdateLapseDisplay();
      } else if (buttnum == 6) {
        Distance -= 100;
        UpdateLapseDisplay();
      } else if (buttnum == 1) {
        TotalTime += 60;
        lastPressed = 0;
        UpdateLapseDisplay();
      } else if (buttnum == 3) {
        TotalTime -= 60;
        lastPressed = 0;
        UpdateLapseDisplay();      
      } else if (buttnum == 2) {
        TotalTime += 5;
        lastPressed = 0;
        UpdateLapseDisplay(); 
      } else if (buttnum == 4) {
        loop();
      }
    }
  }
  stepper.run();

  if (isRunning==1 && !stepper.isRunning()) {
    myMotor->release();
    isRunning=0;
    ble.println("Finished Time Lapse");
  }
  lapse();
}

void UpdateLapseDisplay() {
  TotalSteps = Distance / stepDistance;
  float TotalTimeSecs = TotalTime*60;
  LapseSpeed = TotalSteps / TotalTimeSecs;
  stepper.setSpeed(LapseSpeed);
  stepper.setMaxSpeed(LapseSpeed);
  ble.print(Distance);
  ble.print(" mm over ");
  int hours = TotalTime / 60;
  int mins = TotalTime % 60;
  ble.print(hours);
  ble.print(" hour and ");
  ble.print(mins);
  ble.println(" mins");
  if (debug) {
    ble.print("Total Steps: ");
    ble.println(TotalSteps);
    ble.print(" Steps/Sec: ");
    ble.println(LapseSpeed);
    ble.print("Stepper Speed: ");
    ble.println(stepper.speed());
  }
}

void smoothDolly() {
  uint8_t len = readPacket(&ble, 1);
  if (packetbuffer[1] == 'B') {
    uint8_t buttnum = packetbuffer[2] - '0';
    boolean pressed = packetbuffer[3] - '0';
    if (pressed) {
      if (lastPressed == buttnum && buttnum > 6) {
        stepper.stop();
        delay(1000);
        stepper.setCurrentPosition(0);
        stepper.moveTo(0);
        ble.println("Finished Dolly Move");
        myMotor->release();
        lastPressed = 0;
      } else if (buttnum == 8) {
        ble.println("Starting Forward Dolly");
        stepper.moveTo(-1000000); //move many steps - more then mechanical needed
        lastPressed = buttnum;
      } else if (buttnum == 7) {
        ble.println("Moving Reverse Dolly");
        stepper.moveTo(1000000); //move many steps - more then mechanical needed
        lastPressed = buttnum;
      } else if (buttnum == 5) {
          maxiSpeed += 20;
          updateDollyDisp();
      } else if (buttnum == 6) {
          maxiSpeed -= 20;
          updateDollyDisp();
      } else if (buttnum == 1) {
          maxiSpeed += 1;
          updateDollyDisp();
      } else if (buttnum == 3) {
          maxiSpeed -= 1;
          updateDollyDisp();
      } else if (buttnum == 4) {
        loop();
      }
    }
  }
  stepper.run();
  smoothDolly();
}

void updateDollyDisp() {
  maxiSpeed = constrain(maxiSpeed, 1, 500);
  stepper.setMaxSpeed(maxiSpeed*5);
  ble.print("Speed: ");
  ble.print(maxiSpeed);
  ble.println("%");
}

void fastDolly() {
  uint8_t len = readPacket(&ble, 1);
  if (packetbuffer[1] == 'B') {
    uint8_t buttnum = packetbuffer[2] - '0';
    boolean pressed = packetbuffer[3] - '0';
    if (pressed) {
      if (lastPressed == buttnum && buttnum > 6) {
        stepper.stop();
        delay(1000);
        stepper.setCurrentPosition(0);
        stepper.moveTo(0);
        ble.println("Finished Dolly Move");
        myMotor->release();
        lastPressed = 0;
      } else if (buttnum == 8) {
        ble.println("Starting Forward Dolly");
        stepper.moveTo(-1000000); //move a crazy amount of steps
        lastPressed = buttnum;
      } else if (buttnum == 7) {
        ble.println("Moving Reverse Dolly");
        stepper.moveTo(1000000); //move a crazy amount of steps
        lastPressed = buttnum;
      } else if (buttnum == 5) {
        maxiSpeed += 20;
        updateFastDollyDisp();
      } else if (buttnum == 6) {
        maxiSpeed -= 20;
        updateFastDollyDisp();
      } else if (buttnum == 1) {
        maxiSpeed += 1;
        updateFastDollyDisp();
      } else if (buttnum == 3) {
        maxiSpeed -= 1;
        updateFastDollyDisp();
      } else if (buttnum == 2) {
        if (stepType == SINGLE) {
          stepType = DOUBLE;
          ble.println("Set Step to Double");
        } else if (stepType == DOUBLE) {
          stepType = INTERLEAVE;
          ble.println("Set Step to Interleave");            
        } else if (stepType == INTERLEAVE) {
          stepType = SINGLE;
          ble.println("Set Step to Single");            
        }
      } else if (buttnum == 4) {
        loop();
      }
    }
  }
  stepper.run();
  fastDolly();
}

void updateFastDollyDisp() {
  maxiSpeed = constrain(maxiSpeed, 1,300);
  stepper.setMaxSpeed(maxiSpeed*3.8);
  ble.print("Speed: ");
  ble.print(maxiSpeed);
  ble.println("%");
}
