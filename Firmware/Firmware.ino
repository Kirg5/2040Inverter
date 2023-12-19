#include <RP2040_PWM.h>
#include "Sine.h"
#include "Notes.h"

RP2040_PWM* PWM_Instance[6];

  // Config Vars
  int wheelSize = 26;                 //Wheel diameter in inches
  int polePairs = 42;                 //Motor pole pair count
  int maxForwardCurrent = 50;         //Maximum acceleration current
  int maxReverseCurrent = 10;         //Maximum regen braking current
  int seriesPairs = 20;               //Battery series pairs 
  int undervoltageThreshold = 15;     //Hard undervoltage cutoff to prevent mosfet underdriving
  uint32_t refenFreqSwitching = 1000; //Static PWM frequency for regen braking
  uint32_t freqSwitching = 2000;      //Static PWM frequency for acceleration
  
  // MOSFET pins
  const int phaseAGateLowPin = 20;
  const int phaseBGateLowPin = 19;
  const int phaseCGateLowPin = 18;
  const int phaseAGateHighPin = 23;
  const int phaseBGateHighPin = 22;
  const int phaseCGateHighPin = 21;
  
  // Hall sensor pins
  const int hallSensorAPin = 13;
  const int hallSensorBPin = 14;
  const int hallSensorCPin = 15;

  // ADC pins
  const int currentSensePin = 26;
  const int voltageSensePin = 27;
  const int tempSensePin = 28;
  const int ambientTempSensePin = 29;

  // Misc pins
  const int ledR = 4;
  const int ledG = 5;
  const int ledB = 6;
  const int Fan = 7;
  const int OCP_Int = 24;

  // System Vars
  int currentAngle = 0;
  int estimatedAngle = 0;
  int lastAngle = 0;
  int voltageRaw = 0;
  int currentRaw = 0;
  float voltage = 0;
  float current = 0;
  float phaseCurrent = 0;
  float speed = 0;
  float RotationRate = 0;
  float acceleration = 0;
  float jerk = 0;
  unsigned long timer = 0;
  int phaseTime0 = 0;
  int phaseTime1 = 0;
  int phaseTime2 = 0;
  unsigned long period = 0;
  int dutyCycle = 60;
  int reverseDutyCycle = 0;
  int pulseCount = 0;
  uint32_t freqSwitchingDyn = 0;
  int PWMMode = 0; 
    //PWMMode 0 = Power-on test
    //PWMMode 1 = Square wave, static frequency
    //PWMMode 2 = Square wave, dynamic frequency
    //PWMMode 3 = Sine wave, static frequency
    //PWMMode 4 = Sine wave, dynamic frequency
    //PWMMode 5 = Multi pulse synchronous sine
    //PWMMode 6 = Square wave tone

  void setup() {
  analogWrite(Fan, 255);

  // LED pins
  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(ledR, OUTPUT);
  pinMode(ledG, OUTPUT);
  pinMode(ledB, OUTPUT);

  //Initialize High Side PWM Channels
  PWM_Instance[1] = new RP2040_PWM(phaseAGateHighPin, freqSwitching, 0);
  PWM_Instance[2] = new RP2040_PWM(phaseBGateHighPin, freqSwitching, 0);
  PWM_Instance[3] = new RP2040_PWM(phaseCGateHighPin, freqSwitching, 0);

  //Initialize Low Side PWM Channels
  PWM_Instance[4] = new RP2040_PWM(phaseAGateLowPin, freqSwitching, 0);
  PWM_Instance[5] = new RP2040_PWM(phaseBGateLowPin, freqSwitching, 0);
  PWM_Instance[6] = new RP2040_PWM(phaseCGateLowPin, freqSwitching, 0);

  // Set the pins as inputs for hall effect sensors
  pinMode(hallSensorAPin, INPUT);
  pinMode(hallSensorBPin, INPUT);
  pinMode(hallSensorCPin, INPUT);

  // Setup ADC
  pinMode(currentSensePin, INPUT);
  pinMode(voltageSensePin, INPUT);
  pinMode(tempSensePin, INPUT);
  pinMode(ambientTempSensePin, INPUT);

  // Initialize serial
  Serial.begin(9600);

  delay(100);
  analogWrite(Fan, 0);
}

void loop() 
  {
  // Read raw current and voltage values
  int currentRaw = analogRead(currentSensePin);
  int voltageRaw = analogRead(voltageSensePin);

  // Calculate actual voltage and current
  current = (currentRaw - 511.5) * 50 / 511.5;
  voltage = voltageRaw * 108.9 / 1023;

  //Calculate phase current
  //phaseCurrent = (current * (255 / dutyCycle));

  // Read hall sensors
  int hallSensorAState = digitalRead(hallSensorAPin);
  int hallSensorBState = digitalRead(hallSensorBPin);
  int hallSensorCState = digitalRead(hallSensorCPin);

  // Calculate electrical angle
         if (hallSensorAState == HIGH && hallSensorBState == LOW && hallSensorCState == HIGH) {
    currentAngle = 0;
    timer = micros();
  } else if (hallSensorAState == HIGH && hallSensorBState == LOW && hallSensorCState == LOW) {
    currentAngle = 60;
  } else if (hallSensorAState == HIGH && hallSensorBState == HIGH && hallSensorCState == LOW) {
    currentAngle = 120;
    period = (micros() - timer);
  } else if (hallSensorAState == LOW && hallSensorBState == HIGH && hallSensorCState == LOW) {
    currentAngle = 180;
  } else if (hallSensorAState == LOW && hallSensorBState == HIGH && hallSensorCState == HIGH) {
    currentAngle = 240;
  } else if (hallSensorAState == LOW && hallSensorBState == LOW && hallSensorCState == HIGH) {
    currentAngle = 300;
  }

  //Braking
  //if (reverseDutyCycle > 0) {
    //analogWrite(ledR, 255);
    //Regen control loop here
  //} else {
    //analogWrite(ledR, 0);
  //}

  //Derivatives calculation
  //if (lastAngle != currentAngle) {
    //int phaseTime2 = phaseTime1;
    //int phaseTime1 = (millis() - phaseTime0);
    //int lastAngle = currentAngle;
    //int phaseTime0 = millis();
 // }
  //add the shit for slope and exponent here

  //Calculate RPM
  //RotationRate = (60000/period/polePairs);

  //Calculate Speed from RPM
  //speed = (RotationRate * wheelSize * 0.00479);

  //Placeholder dynamic frequency eqn
  //freqSwitchingDyn = ((RotationRate * 0.1) + 2000);
      if (period > 12000) {
      freqSwitchingDyn = 1500; 
  } else if (period > 8000 && period < 11999) {
      freqSwitchingDyn = 2000; 
    } else if (period > 6000 && period < 7999) {
      freqSwitchingDyn = 3000; 
    } else if (period > 4000 && period < 5999) {
freqSwitchingDyn = 4000; 
      } else if (period > 2000 && period < 3999) {
freqSwitchingDyn = 5000; 
        } else if (period < 1999) {
freqSwitchingDyn = 8000; 
        }

  //Motor commutation
    if (PWMMode == 1) { // Square wave, static frequency
    if (currentAngle == 0) {
      //High Side
      PWM_Instance[1]->setPWM(phaseAGateHighPin, freqSwitching, dutyCycle);
      PWM_Instance[2]->setPWM(phaseBGateHighPin, freqSwitching, 0);
      PWM_Instance[3]->setPWM(phaseCGateHighPin, freqSwitching, 0);
      //Low Side
      PWM_Instance[4]->setPWM(phaseAGateLowPin, freqSwitching, 0);
      PWM_Instance[5]->setPWM(phaseBGateLowPin, freqSwitching, dutyCycle);
      PWM_Instance[6]->setPWM(phaseCGateLowPin, freqSwitching, 0);
    } else if (currentAngle == 60) {
      //High Side
      PWM_Instance[1]->setPWM(phaseAGateHighPin, freqSwitching, dutyCycle);
      PWM_Instance[2]->setPWM(phaseBGateHighPin, freqSwitching, 0);
      PWM_Instance[3]->setPWM(phaseCGateHighPin, freqSwitching, 0);
      //Low Side
      PWM_Instance[4]->setPWM(phaseAGateLowPin, freqSwitching, 0);
      PWM_Instance[5]->setPWM(phaseBGateLowPin, freqSwitching, 0);
      PWM_Instance[6]->setPWM(phaseCGateLowPin, freqSwitching, dutyCycle);
    } else if (currentAngle == 120) {
      //High Side
      PWM_Instance[1]->setPWM(phaseAGateHighPin, freqSwitching, 0);
      PWM_Instance[2]->setPWM(phaseBGateHighPin, freqSwitching, dutyCycle);
      PWM_Instance[3]->setPWM(phaseCGateHighPin, freqSwitching, 0);
      //Low Side
      PWM_Instance[4]->setPWM(phaseAGateLowPin, freqSwitching, 0);
      PWM_Instance[5]->setPWM(phaseBGateLowPin, freqSwitching, 0);
      PWM_Instance[6]->setPWM(phaseCGateLowPin, freqSwitching, dutyCycle);
    } else if (currentAngle == 180) {
      //High Side
      PWM_Instance[1]->setPWM(phaseAGateHighPin, freqSwitching, 0);
      PWM_Instance[2]->setPWM(phaseBGateHighPin, freqSwitching, dutyCycle);
      PWM_Instance[3]->setPWM(phaseCGateHighPin, freqSwitching, 0);
      //Low Side
      PWM_Instance[4]->setPWM(phaseAGateLowPin, freqSwitching, dutyCycle);
      PWM_Instance[5]->setPWM(phaseBGateLowPin, freqSwitching, 0);
      PWM_Instance[6]->setPWM(phaseCGateLowPin, freqSwitching, 0);
    } else if (currentAngle == 240) {
      //High Side
      PWM_Instance[1]->setPWM(phaseAGateHighPin, freqSwitching, 0);
      PWM_Instance[2]->setPWM(phaseBGateHighPin, freqSwitching, 0);
      PWM_Instance[3]->setPWM(phaseCGateHighPin, freqSwitching, dutyCycle);
      //Low Side
      PWM_Instance[4]->setPWM(phaseAGateLowPin, freqSwitching, dutyCycle);
      PWM_Instance[5]->setPWM(phaseBGateLowPin, freqSwitching, 0);
      PWM_Instance[6]->setPWM(phaseCGateLowPin, freqSwitching, 0);
    } else if (currentAngle == 300) {
      //High Side
      PWM_Instance[1]->setPWM(phaseAGateHighPin, freqSwitching, 0);
      PWM_Instance[2]->setPWM(phaseBGateHighPin, freqSwitching, 0);
      PWM_Instance[3]->setPWM(phaseCGateHighPin, freqSwitching, dutyCycle);
      //Low Side
      PWM_Instance[4]->setPWM(phaseAGateLowPin, freqSwitching, 0);
      PWM_Instance[5]->setPWM(phaseBGateLowPin, freqSwitching, dutyCycle);
      PWM_Instance[6]->setPWM(phaseCGateLowPin, freqSwitching, 0);
    }
  } else if (PWMMode == 2) { // Square wave, dynamic frequency
    if (currentAngle == 0) {
      //High Side
      PWM_Instance[1]->setPWM(phaseAGateHighPin, freqSwitchingDyn, dutyCycle);
      PWM_Instance[2]->setPWM(phaseBGateHighPin, freqSwitchingDyn, 0);
      PWM_Instance[3]->setPWM(phaseCGateHighPin, freqSwitchingDyn, 0);
      //Low Side
      PWM_Instance[4]->setPWM(phaseAGateLowPin, freqSwitchingDyn, 0);
      PWM_Instance[5]->setPWM(phaseBGateLowPin, freqSwitchingDyn, dutyCycle);
      PWM_Instance[6]->setPWM(phaseCGateLowPin, freqSwitchingDyn, 0);
    } else if (currentAngle == 60) {
      //High Side
      PWM_Instance[1]->setPWM(phaseAGateHighPin, freqSwitchingDyn, dutyCycle);
      PWM_Instance[2]->setPWM(phaseBGateHighPin, freqSwitchingDyn, 0);
      PWM_Instance[3]->setPWM(phaseCGateHighPin, freqSwitchingDyn, 0);
      //Low Side
      PWM_Instance[4]->setPWM(phaseAGateLowPin, freqSwitchingDyn, 0);
      PWM_Instance[5]->setPWM(phaseBGateLowPin, freqSwitchingDyn, 0);
      PWM_Instance[6]->setPWM(phaseCGateLowPin, freqSwitchingDyn, dutyCycle);
    } else if (currentAngle == 120) {
      //High Side
      PWM_Instance[1]->setPWM(phaseAGateHighPin, freqSwitchingDyn, 0);
      PWM_Instance[2]->setPWM(phaseBGateHighPin, freqSwitchingDyn, dutyCycle);
      PWM_Instance[3]->setPWM(phaseCGateHighPin, freqSwitchingDyn, 0);
      //Low Side
      PWM_Instance[4]->setPWM(phaseAGateLowPin, freqSwitchingDyn, 0);
      PWM_Instance[5]->setPWM(phaseBGateLowPin, freqSwitchingDyn, 0);
      PWM_Instance[6]->setPWM(phaseCGateLowPin, freqSwitchingDyn, dutyCycle);
    } else if (currentAngle == 180) {
      //High Side
      PWM_Instance[1]->setPWM(phaseAGateHighPin, freqSwitchingDyn, 0);
      PWM_Instance[2]->setPWM(phaseBGateHighPin, freqSwitchingDyn, dutyCycle);
      PWM_Instance[3]->setPWM(phaseCGateHighPin, freqSwitchingDyn, 0);
      //Low Side
      PWM_Instance[4]->setPWM(phaseAGateLowPin, freqSwitchingDyn, dutyCycle);
      PWM_Instance[5]->setPWM(phaseBGateLowPin, freqSwitchingDyn, 0);
      PWM_Instance[6]->setPWM(phaseCGateLowPin, freqSwitchingDyn, 0);
    } else if (currentAngle == 240) {
      //High Side
      PWM_Instance[1]->setPWM(phaseAGateHighPin, freqSwitchingDyn, 0);
      PWM_Instance[2]->setPWM(phaseBGateHighPin, freqSwitchingDyn, 0);
      PWM_Instance[3]->setPWM(phaseCGateHighPin, freqSwitchingDyn, dutyCycle);
      //Low Side
      PWM_Instance[4]->setPWM(phaseAGateLowPin, freqSwitchingDyn, dutyCycle);
      PWM_Instance[5]->setPWM(phaseBGateLowPin, freqSwitchingDyn, 0);
      PWM_Instance[6]->setPWM(phaseCGateLowPin, freqSwitchingDyn, 0);
    } else if (currentAngle == 300) {
      //High Side
      PWM_Instance[1]->setPWM(phaseAGateHighPin, freqSwitchingDyn, 0);
      PWM_Instance[2]->setPWM(phaseBGateHighPin, freqSwitchingDyn, 0);
      PWM_Instance[3]->setPWM(phaseCGateHighPin, freqSwitchingDyn, dutyCycle);
      //Low Side
      PWM_Instance[4]->setPWM(phaseAGateLowPin, freqSwitchingDyn, 0);
      PWM_Instance[5]->setPWM(phaseBGateLowPin, freqSwitchingDyn, dutyCycle);
      PWM_Instance[6]->setPWM(phaseCGateLowPin, freqSwitchingDyn, 0);
    }
  } else if (PWMMode == 3) { // Sine wave, static frequency
      //int estimatedAngle = (currentAngle + (phaseTime1 * idk));
  } else if (PWMMode == 4) { // Sine wave, dynamic frequency

  } else if (PWMMode == 5) { // Multi pulse synchronous sine

  } else if (PWMMode == 6) { // Square wave tone

  } else if (PWMMode == 0) { // Test
      //High Side
      PWM_Instance[1]->setPWM(phaseAGateHighPin, 1000, 10);
      PWM_Instance[2]->setPWM(phaseBGateHighPin, 1000, 0);
      PWM_Instance[3]->setPWM(phaseCGateHighPin, 1000, 0);
      //Low Side
      PWM_Instance[4]->setPWM(phaseAGateLowPin, 1000, 0);
      PWM_Instance[5]->setPWM(phaseBGateLowPin, 1000, 10);
      PWM_Instance[6]->setPWM(phaseCGateLowPin, 1000, 0);
    delay(100);
      //High Side
      PWM_Instance[1]->setPWM(phaseAGateHighPin, 2000, 10);
      PWM_Instance[2]->setPWM(phaseBGateHighPin, 2000, 0);
      PWM_Instance[3]->setPWM(phaseCGateHighPin, 2000, 0);
      //Low Side
      PWM_Instance[4]->setPWM(phaseAGateLowPin, 2000, 0);
      PWM_Instance[5]->setPWM(phaseBGateLowPin, 2000, 0);
      PWM_Instance[6]->setPWM(phaseCGateLowPin, 2000, 10);
    delay(100);
      //High Side
      PWM_Instance[1]->setPWM(phaseAGateHighPin, 3000, 0);
      PWM_Instance[2]->setPWM(phaseBGateHighPin, 3000, 10);
      PWM_Instance[3]->setPWM(phaseCGateHighPin, 3000, 0);
      //Low Side
      PWM_Instance[4]->setPWM(phaseAGateLowPin, 3000, 0);
      PWM_Instance[5]->setPWM(phaseBGateLowPin, 3000, 0);
      PWM_Instance[6]->setPWM(phaseCGateLowPin, 3000, 10);
    delay(100);
      //High Side
      PWM_Instance[1]->setPWM(phaseAGateHighPin, 4000, 0);
      PWM_Instance[2]->setPWM(phaseBGateHighPin, 4000, 10);
      PWM_Instance[3]->setPWM(phaseCGateHighPin, 4000, 0);
      //Low Side
      PWM_Instance[4]->setPWM(phaseAGateLowPin, 4000, 10);
      PWM_Instance[5]->setPWM(phaseBGateLowPin, 4000, 0);
      PWM_Instance[6]->setPWM(phaseCGateLowPin, 4000, 0);
    delay(100);
      //High Side
      PWM_Instance[1]->setPWM(phaseAGateHighPin, 5000, 0);
      PWM_Instance[2]->setPWM(phaseBGateHighPin, 5000, 0);
      PWM_Instance[3]->setPWM(phaseCGateHighPin, 5000, 10);
      //Low Side
      PWM_Instance[4]->setPWM(phaseAGateLowPin, 5000, 10);
      PWM_Instance[5]->setPWM(phaseBGateLowPin, 5000, 0);
      PWM_Instance[6]->setPWM(phaseCGateLowPin, 5000, 0);
    delay(100);
      //High Side
      PWM_Instance[1]->setPWM(phaseAGateHighPin, 6000, 0);
      PWM_Instance[2]->setPWM(phaseBGateHighPin, 6000, 0);
      PWM_Instance[3]->setPWM(phaseCGateHighPin, 6000, 10);
      //Low Side
      PWM_Instance[4]->setPWM(phaseAGateLowPin, 6000, 0);
      PWM_Instance[5]->setPWM(phaseBGateLowPin, 6000, 10);
      PWM_Instance[6]->setPWM(phaseCGateLowPin, 6000, 0);
      delay(100);
    PWMMode = 2; 
  }
if (Serial.available() > 0) {
  dutyCycle = 100;
}

analogWrite(LED_BUILTIN, dutyCycle);

Serial.print("angle = ");
Serial.print(currentAngle);
Serial.print("\t Period = ");
Serial.print(period);
Serial.print("\t Frequency = ");
Serial.print(freqSwitchingDyn);
Serial.print("\t RPM = ");
Serial.print(RotationRate);
Serial.print("\t voltage = ");
Serial.print(voltage);
Serial.print("\t current = ");
Serial.println(current);
  delay(0);
}
