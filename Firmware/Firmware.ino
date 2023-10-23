#include <RP2040_PWM.h>
#include "Sine.h"
#include "Notes.h"

RP2040_PWM* PWM_Instance[6];

  // Config Vars
  int wheelSize = 26;
  int polePairs = 42;
  int multiPulse_minFreq = 2000;
  int multiPulse_maxFreq = 30000;
  uint32_t freqSwitching = 8000;
  int PWMMode = 1; 

  //PWMMode 1 = Square wave, static frequency
  //PWMMode 2 = Square wave, dynamic frequency
  //PWMMode 3 = Sine wave, static frequency
  //PWMMode 4 = Sine wave, dynamic frequency
  //PWMMode 5 = Multi pulse synchronous sine
  //PWMMode 6 = Square wave tone
  
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
  const int OCP_Int = 24;

  // Misc Vars
  int currentAngle = 0;
  int estimatedAngle = 0;
  int voltageRaw = 0;
  int currentRaw = 0;
  float voltage = 0;
  float current = 0;
  float speed = 0;
  float rpm = 0;
  float acceleration = 0;
  float jerk = 0;
  int timer = 0;
  int period = 0;
  int dutyCycle = 0;
  uint32_t freqSwitchingDyn = 0;

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

  // Read hall sensors
  int hallSensorAState = digitalRead(hallSensorAPin);
  int hallSensorBState = digitalRead(hallSensorBPin);
  int hallSensorCState = digitalRead(hallSensorCPin);

  // Calculate electrical angle
         if (hallSensorAState == HIGH && hallSensorBState == LOW && hallSensorCState == HIGH) {
    currentAngle = 0;
    int timer = millis();
  } else if (hallSensorAState == HIGH && hallSensorBState == LOW && hallSensorCState == LOW) {
    currentAngle = 60;
  } else if (hallSensorAState == HIGH && hallSensorBState == HIGH && hallSensorCState == LOW) {
    currentAngle = 120;
    int period = (millis() - timer);
  } else if (hallSensorAState == LOW && hallSensorBState == HIGH && hallSensorCState == LOW) {
    currentAngle = 180;
  } else if (hallSensorAState == LOW && hallSensorBState == HIGH && hallSensorCState == HIGH) {
    currentAngle = 240;
  } else if (hallSensorAState == LOW && hallSensorBState == LOW && hallSensorCState == HIGH) {
    currentAngle = 300;
  }

  //Calculate RPM
  float rpm = (60000.0f/period/polePairs);

  //Calculate Speed from RPM
  float speed = (rpm * wheelSize * 0.00479);

  //Placeholder dynamic frequency eqn
  freqSwitchingDyn = ((rpm * 0.1) + 2000);

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

  } else if (PWMMode == 4) { // Sine wave, dynamic frequency

  } else if (PWMMode == 5) { // Multi pulse synchronous sine

  } else if (PWMMode == 6) { // Square wave tone

  }
if (Serial.available() > 0) {
  dutyCycle = 250;
}

analogWrite(LED_BUILTIN, dutyCycle);

Serial.print("angle = ");
Serial.print(currentAngle);
Serial.print("\t voltage = ");
Serial.print(voltage);
Serial.print("\t current = ");
Serial.println(current);
  delay(0);
}
