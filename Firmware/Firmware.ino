#include <RP2040_PWM.h>
#include "Sine.h"
#include "Notes.h"

RP2040_PWM* PWM_Instance[6];

// MOSFET pins
  const int phaseAGateLowPin = 18;
  const int phaseBGateLowPin = 19;
  const int phaseCGateLowPin = 20;
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

  // Vars
  int currentAngle = 0;
  int voltageRaw = 0;
  int currentRaw = 0;
  float voltage = 0;
  float current = 0;
  uint32_t freqSwitching = 8000;
  int PWM = 60;
  int PWMMode = 1; 

  //PWMMode 1  = Square wave, static frequency
  //PWMMode 2  = Square wave, dynamic frequency
  //PWMMode 3  = Stepped sine wave, static frequency
  //PWMMode 4  = Stepped sine wave, dynamic frequency 
  //PWMMode 5  = Sine wave, static frequency
  //PWMMode 6  = Sine wave, dynamic frequency
  //PWMMode 7  = 7 pulse synchronous sine
  //PWMMode 8  = 5 pulse synchronous sine
  //PWMMode 9  = 3 pulse synchronous sine
  //PWMMode 10 = Square wave tone
  
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

  // Read hall sensors
  int hallSensorAState = digitalRead(hallSensorAPin);
  int hallSensorBState = digitalRead(hallSensorBPin);
  int hallSensorCState = digitalRead(hallSensorCPin);

  // Calculate electrical angle
         if (hallSensorAState == HIGH && hallSensorBState == LOW && hallSensorCState == HIGH) {
    currentAngle = 0;
  } else if (hallSensorAState == HIGH && hallSensorBState == LOW && hallSensorCState == LOW) {
    currentAngle = 60;
  } else if (hallSensorAState == HIGH && hallSensorBState == HIGH && hallSensorCState == LOW) {
    currentAngle = 120;
  } else if (hallSensorAState == LOW && hallSensorBState == HIGH && hallSensorCState == LOW) {
    currentAngle = 180;
  } else if (hallSensorAState == LOW && hallSensorBState == HIGH && hallSensorCState == HIGH) {
    currentAngle = 240;
  } else if (hallSensorAState == LOW && hallSensorBState == LOW && hallSensorCState == HIGH) {
    currentAngle = 300;
  }

  //Motor commutation
  if (PWMMode == 1) {
    if (currentAngle == 0) {
      PWM_Instance[4]->setPWM(phaseAGateLowPin, freqSwitching, 0);
      PWM_Instance[5]->setPWM(phaseBGateLowPin, freqSwitching, PWM);
      PWM_Instance[6]->setPWM(phaseCGateLowPin, freqSwitching, 0);

      PWM_Instance[1]->setPWM(phaseAGateHighPin, freqSwitching, PWM);
      PWM_Instance[2]->setPWM(phaseBGateHighPin, freqSwitching, 0);
      PWM_Instance[3]->setPWM(phaseCGateHighPin, freqSwitching, 0);
    } else if (currentAngle == 60) {
      PWM_Instance[4]->setPWM(phaseAGateLowPin, freqSwitching, 0);
      PWM_Instance[5]->setPWM(phaseBGateLowPin, freqSwitching, 0);
      PWM_Instance[6]->setPWM(phaseCGateLowPin, freqSwitching, PWM);

      PWM_Instance[1]->setPWM(phaseAGateHighPin, freqSwitching, PWM);
      PWM_Instance[2]->setPWM(phaseBGateHighPin, freqSwitching, 0);
      PWM_Instance[3]->setPWM(phaseCGateHighPin, freqSwitching, 0);
    } else if (currentAngle == 120) {
      PWM_Instance[4]->setPWM(phaseAGateLowPin, freqSwitching, 0);
      PWM_Instance[5]->setPWM(phaseBGateLowPin, freqSwitching, 0);
      PWM_Instance[6]->setPWM(phaseCGateLowPin, freqSwitching, PWM);

      PWM_Instance[1]->setPWM(phaseAGateHighPin, freqSwitching, 0);
      PWM_Instance[2]->setPWM(phaseBGateHighPin, freqSwitching, PWM);
      PWM_Instance[3]->setPWM(phaseCGateHighPin, freqSwitching, 0);
    } else if (currentAngle == 180) {
      PWM_Instance[4]->setPWM(phaseAGateLowPin, freqSwitching, PWM);
      PWM_Instance[5]->setPWM(phaseBGateLowPin, freqSwitching, 0);
      PWM_Instance[6]->setPWM(phaseCGateLowPin, freqSwitching, 0);

      PWM_Instance[1]->setPWM(phaseAGateHighPin, freqSwitching, 0);
      PWM_Instance[2]->setPWM(phaseBGateHighPin, freqSwitching, PWM);
      PWM_Instance[3]->setPWM(phaseCGateHighPin, freqSwitching, 0);
    } else if (currentAngle == 240) {
      PWM_Instance[4]->setPWM(phaseAGateLowPin, freqSwitching, PWM);
      PWM_Instance[5]->setPWM(phaseBGateLowPin, freqSwitching, 0);
      PWM_Instance[6]->setPWM(phaseCGateLowPin, freqSwitching, 0);

      PWM_Instance[1]->setPWM(phaseAGateHighPin, freqSwitching, 0);
      PWM_Instance[2]->setPWM(phaseBGateHighPin, freqSwitching, 0);
      PWM_Instance[3]->setPWM(phaseCGateHighPin, freqSwitching, PWM);
    } else if (currentAngle == 300) {
      PWM_Instance[4]->setPWM(phaseAGateLowPin, freqSwitching, 0);
      PWM_Instance[5]->setPWM(phaseBGateLowPin, freqSwitching, PWM);
      PWM_Instance[6]->setPWM(phaseCGateLowPin, freqSwitching, 0);

      PWM_Instance[1]->setPWM(phaseAGateHighPin, freqSwitching, 0);
      PWM_Instance[2]->setPWM(phaseBGateHighPin, freqSwitching, 0);
      PWM_Instance[3]->setPWM(phaseCGateHighPin, freqSwitching, PWM);
    }
  } else if (PWMMode == 2) { // Same as previous mode but dynamic freq

  } else if (PWMMode == 3) { // Mode 1 but 1/2 PWM

  } else if (PWMMode == 4) { // Same as previous mode but dynamic freq

  } else if (PWMMode == 5) { // Full sine control

  } else if (PWMMode == 6) { // Same as previous mode but dynamic freq

  } else if (PWMMode == 7) { // Multipulse mode

  } else if (PWMMode == 8) { // Multipulse mode again

  } else if (PWMMode == 9) { // Multipulse mode yet again

  } else if (PWMMode == 10) { // Similar to mode 2 but using tones

  }
if (Serial.available() > 0) {
  PWM = 250;
}

analogWrite(LED_BUILTIN, PWM);

Serial.print("angle = ");
Serial.print(currentAngle);
Serial.print("\t voltage = ");
Serial.print(voltage);
Serial.print("\t current = ");
Serial.println(current);
  delay(0);
}
