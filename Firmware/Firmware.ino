#include <math.h>
#include <uart.h>

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

// UART Data pins
const int uartTX = 8;
const int uartRX = 9;

// Misc system Pins
const int fanPin = 7;
const int ledR = 4;
const int ledG = 5;
const int ledB = 6;
const int ledInternal = 25;
const int VOC = 24;
const int extGPIO = 17;

// System parameters
const int motorPolePairs = 46;        // Number of pole pairs in the motor, used to calculate RPM
const int maxForwardCurrent = 48;     // Motor current limit in amps
const int maxReverseCurrent = 10;     // Regen current limit in amps
const float undervolageCutoff = 3.2;  // Minimum cell voltage to power the motor in volts
const int parallelGroupCount = 20;    // How many cells are in parallel in the battery pack
const float

// Control variables
float targetPower = 0;                // Target output power set by the user
float currentAngle = 0;               // Current electrical angle of the motor
float errorSum = 0;                   // Error sum for integral control
float prevError = 0;                  // Previous error for derivative control

// Control constants
const float Kp = 0.01;                // Proportional gain
const float Ki = 0.001;               // Integral gain
const float Kd = 0.001;               // Derivative gain

// Sine lookup table
const int numAngles = 360;
const float angleStep = 360.0 / numAngles;
float sinTable[numAngles];

void setup() {
  // Set the pins as outputs for high-side MOSFET gate drives
  pinMode(phaseAGateHighPin, OUTPUT);
  pinMode(phaseBGateHighPin, OUTPUT);
  pinMode(phaseCGateHighPin, OUTPUT);

  // Set the pins as outputs for low-side MOSFET gate drives
  pinMode(phaseAGateLowPin, OUTPUT);
  pinMode(phaseBGateLowPin, OUTPUT);
  pinMode(phaseCGateLowPin, OUTPUT);

  // Set the pins as inputs for hall effect sensors
  pinMode(hallSensorAPin, INPUT);
  pinMode(hallSensorBPin, INPUT);
  pinMode(hallSensorCPin, INPUT);

  //Initialize UART communication
  #define UART_ID uart0
  #define BAUD_RATE 115200
  #define DATA_BITS 40
  #define STOP_BITS 1
  #define PARITY    UART_PARITY_NONE

  // Populate the sine lookup table
  for (int i = 0; i < numAngles; i++) {
    float radian = i * angleStep * PI / 180.0;
    sinTable[i] = sin(radian);
  }
}

void loop() {
  // Read the state of hall effect sensors
  int hallSensorAState = digitalRead(hallSensorAPin);
  int hallSensorBState = digitalRead(hallSensorBPin);
  int hallSensorCState = digitalRead(hallSensorCPin);

  // Calculate the electrical angle based on the hall sensor states
  if (hallSensorAState == HIGH && hallSensorBState == LOW && hallSensorCState == LOW) {
    currentAngle = 0;
  } else if (hallSensorAState == HIGH && hallSensorBState == LOW && hallSensorCState == HIGH) {
    currentAngle = 60;
  } else if (hallSensorAState == LOW && hallSensorBState == HIGH && hallSensorCState == HIGH) {
    currentAngle = 120;
  } else if (hallSensorAState == LOW && hallSensorBState == HIGH && hallSensorCState == LOW) {
    currentAngle = 180;
  } else if (hallSensorAState == HIGH && hallSensorBState == HIGH && hallSensorCState == LOW) {
    currentAngle = 240;
  } else if (hallSensorAState == HIGH && hallSensorBState == LOW && hallSensorCState == LOW) {
    currentAngle = 300;
  }

  // Calculate the index in the sine lookup table
  int index = static_cast<int>(currentAngle) % numAngles;

  // Get the sine value from the lookup table
  float sinValue = sinTable[index];

  //Get throttle value from other core
  int throttle = idk

  // Clip the throttle value within the valid range
  throttle = constrain(throttle, 0, maxPWM);

  // Apply control signal to the motor phases using sinusoidal control
  int pwmA = throttle * (sinValue + 0.5);
  int pwmB = throttle * (sinValue - 0.5);
  int pwmC = -pwmA - pwmB;

  // Apply the PWM values to the gate drives
  analogWrite(phaseAGateHighPin, pwmA);
  analogWrite(phaseAGateLowPin, pwmA);    // Set low-side gate drive equal to high-side for current control
  analogWrite(phaseBGateHighPin, pwmB);
  analogWrite(phaseBGateLowPin, pwmB);
  analogWrite(phaseCGateHighPin, pwmC);
  analogWrite(phaseCGateLowPin, pwmC);

  // Add a suitable delay between motor control updates
  delay(10);
}
