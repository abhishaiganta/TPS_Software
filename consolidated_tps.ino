#include <Arduino.h>
#include <IntervalTimer.h>
#include "ICM_20948.h" // http://librarymanager/All#SparkFun_ICM_20948_IMU
#include "HX711.h"

// ==========================================
// 1. HARDWARE PINS & MOTOR CONFIGURATION
// ==========================================
// Updated to 5, 6, 7 per Electrical Team specs
constexpr uint8_t STEP_N_PIN = 5;
constexpr uint8_t DIR_N_PIN  = 6;
constexpr uint8_t EN_N_PIN   = 7;

constexpr uint8_t DATA_PIN = 2; // HX711 DT
constexpr uint8_t CLOCK_PIN = 3; // HX711 SCK

constexpr uint8_t DM_OFF = HIGH;
constexpr uint8_t DM_ON  = LOW;

IntervalTimer stepTimer;
constexpr uint32_t STEP_PULSE_LOW_US = 5;

#define WIRE_PORT Wire 
#define AD0_VAL 1
ICM_20948_I2C myICM; 
HX711 pressure_sensor; 

// ==========================================
// 2. SIGNAL PROCESSING & FILTERS
// ==========================================
class MovingAverageFilter {
    private:
    float* buffer;
    int windowSize;
    int index = 0;
    float currentSum = 0;

    public:
    MovingAverageFilter(int size) : windowSize(size) {
        buffer = new float[size]{0}; 
    }

    float process(float newValue) {
        currentSum -= buffer[index];       // Remove oldest value
        buffer[index] = newValue;          // Add new value
        currentSum += buffer[index];       // Update sum
        index = (index + 1) % windowSize;  // Move window index
        return currentSum / windowSize;    // Return the new average
    }
};

MovingAverageFilter weightFilter(5);
MovingAverageFilter gyroFilter(5); 

float filteredAngle = 0.0;
unsigned long lastTime = 0;

// ==========================================
// 3. GAIT PHASE STATE MACHINE
// ==========================================
enum GaitPhase {
  INITIAL_CONTACT, 
  LOADING_RESPONSE, 
  MID_STANCE,      
  TERMINAL_STANCE, 
  PRE_SWING,       
  INITIAL_SWING,   
  MID_SWING,       
  TERMINAL_SWING   
};

GaitPhase currentPhase = MID_STANCE; 

float baselineWeight = 0.0;
float baselineAngle = 0.0;
float weightThresholdContact; 
float weightThresholdLiftoff; 

float maxStanceAngle = -999.0;
float minStanceAngle = 999.0;
float maxSwingAngle = -999.0;
float minSwingAngle = 999.0;

float currentMotorSpeed = 0.0; 
float currentRequiredTorque = 0.0;

// ==========================================
// 4. MOTOR CONTROL FUNCTIONS (Brennon's Code)
// ==========================================
void stepISR() {
  digitalWriteFast(STEP_N_PIN, DM_ON);
  delayMicroseconds(STEP_PULSE_LOW_US);
  digitalWriteFast(STEP_N_PIN, DM_OFF);
}

void setSpeedSPS(float sps) {
  if (sps <= 0) {
    stepTimer.end();
    digitalWriteFast(STEP_N_PIN, DM_OFF);
    return;
  }
  float period_us = 1e6f / sps;
  stepTimer.begin(stepISR, period_us);
}

void enableDriver(bool enable) {
  digitalWriteFast(EN_N_PIN, enable ? DM_ON : DM_OFF);
}

void setDirection(bool forward) {
  digitalWriteFast(DIR_N_PIN, forward ? DM_ON : DM_OFF);
}

// ==========================================
// 5. KNEE ACTUATION LOGIC
// ==========================================
void calculateMotorTarget(GaitPhase phase, float &targetSpeedSPS, float &targetTorque) {
    // Defines "When" and "How Much" power/speed is needed per phase
    switch(phase) {
        case INITIAL_CONTACT:
        case LOADING_RESPONSE:
            targetTorque = 80.0; // High stiffness
            targetSpeedSPS = -200.0; // Slight flex (negative) to yield
            break;
            
        case MID_STANCE:
            targetTorque = 100.0; // Max support
            targetSpeedSPS = 0.0; // Lock position
            break;
            
        case TERMINAL_STANCE:
        case PRE_SWING:
            targetTorque = 90.0; 
            targetSpeedSPS = 600.0; // Forward driving speed (push-off)
            break;
            
        case INITIAL_SWING:
        case MID_SWING:
            targetTorque = 30.0; // Free swing
            targetSpeedSPS = 800.0; // Fast flexion clearing ground
            break;
            
        case TERMINAL_SWING:
            targetTorque = 60.0; // Braking torque
            targetSpeedSPS = -400.0; // Decelerating extension
            break;
    }
}

void actuateMotor(float speedSPS, float torque) {
    // 1. Handle Enable/Disable based on required torque
    if (torque > 5.0) {
        enableDriver(true);
    } else {
        enableDriver(false);
    }

    // 2. Handle Direction and Timer Speed
    if (speedSPS > 0) {
        setDirection(true); // Forward
        setSpeedSPS(speedSPS);
    } else if (speedSPS < 0) {
        setDirection(false); // Reverse
        setSpeedSPS(abs(speedSPS)); // Timer only takes positive periods
    } else {
        setSpeedSPS(0);
    }
}

// ==========================================
// 6. PHASE DETECTION LOGIC
// ==========================================
void detectPhase(float weight, float angle, float velocity) {
  if (currentPhase >= INITIAL_CONTACT && currentPhase <= TERMINAL_STANCE) {
      if (angle > maxStanceAngle) maxStanceAngle = angle;
      if (angle < minStanceAngle) minStanceAngle = angle;
  } else {
      if (angle > maxSwingAngle) maxSwingAngle = angle;
      if (angle < minSwingAngle) minSwingAngle = angle;
  }

  switch (currentPhase) {
    case TERMINAL_SWING:
      if (weight > weightThresholdContact) {
          currentPhase = INITIAL_CONTACT;
          maxStanceAngle = angle; minStanceAngle = angle;
      } break;

    case INITIAL_CONTACT:
      if (abs(angle - baselineAngle) < 10.0) currentPhase = LOADING_RESPONSE;
      break;

    case LOADING_RESPONSE:
      if (abs(angle - baselineAngle) < 3.0) currentPhase = MID_STANCE;
      break;

    case MID_STANCE:
      if ((angle - baselineAngle) < -5.0) currentPhase = TERMINAL_STANCE;
      break;

    case TERMINAL_STANCE:
      if (weight < weightThresholdLiftoff && (angle - baselineAngle) < -10.0) {
          currentPhase = PRE_SWING;
          maxSwingAngle = angle; minSwingAngle = angle;
      } 
      break;

    case PRE_SWING:
      if (weight < (weightThresholdLiftoff * 0.2)) currentPhase = INITIAL_SWING;
      break;

    case INITIAL_SWING:
      if (velocity > 30.0) currentPhase = MID_SWING;
      break;

    case MID_SWING:
      if ((angle - baselineAngle) > 10.0 && velocity < 15.0) currentPhase = TERMINAL_SWING; 
      break;
  }
}

// ==========================================
// 7. SENSOR CALIBRATION
// ==========================================
void calibrateSensors() {
  Serial.println("--- STARTING CALIBRATION ---");
  Serial.println("Please stand still with full weight on the prosthetic...");
  delay(2000); 

  float tempWeight = 0;
  float tempAngle = 0;
  int samples = 50;

  for (int i = 0; i < samples; i++) {
    if (myICM.dataReady()) {
      myICM.getAGMT();
      float rawPitch = atan2(myICM.accX(), myICM.accZ()) * 57.2958;
      tempAngle += rawPitch; 
      
      float rawWeight = pressure_sensor.get_units(1);
      tempWeight = weightFilter.process(rawWeight); 
      delay(20);
    }
  }

  baselineAngle = tempAngle / samples;
  baselineWeight = tempWeight;
  filteredAngle = baselineAngle; 
  
  weightThresholdContact = baselineWeight * 0.15; 
  weightThresholdLiftoff = baselineWeight * 0.40; 

  Serial.println("--- CALIBRATION COMPLETE ---");
  Serial.print("Baseline Angle: "); Serial.println(baselineAngle);
  Serial.print("Baseline Weight: "); Serial.println(baselineWeight);
  
  lastTime = millis(); 
  delay(1000);
}

// ==========================================
// 8. SYSTEM SETUP
// ==========================================
void setup() {
  Serial.begin(115200);

  // Motor Pin Setup
  pinMode(STEP_N_PIN, OUTPUT);
  pinMode(DIR_N_PIN, OUTPUT);
  pinMode(EN_N_PIN, OUTPUT);

  digitalWriteFast(STEP_N_PIN, DM_OFF);
  digitalWriteFast(DIR_N_PIN, DM_OFF);
  digitalWriteFast(EN_N_PIN, DM_OFF);
  
  // Default driver to ON for holding torque
  enableDriver(true);

  // Pressure Sensor Setup
  pressure_sensor.begin(DATA_PIN, CLOCK_PIN);
  pressure_sensor.set_offset(8235729);
  pressure_sensor.set_scale(8.23);
  pressure_sensor.tare(); 
  
  // IMU Setup
  WIRE_PORT.begin();
  WIRE_PORT.setClock(400000);

  bool initialized = false;
  while (!initialized) {
      myICM.begin(WIRE_PORT, AD0_VAL);
    if (myICM.status != ICM_20948_Stat_Ok) {
      delay(500);
    } else {
      initialized = true;
    }
  }

  calibrateSensors();
}

// ==========================================
// 9. MAIN LOOP
// ==========================================
void loop() {
  if (myICM.dataReady() && pressure_sensor.is_ready()) {
    myICM.getAGMT(); 
    
    // Time tracking for Gyro
    unsigned long currentTime = millis();
    float dt = (currentTime - lastTime) / 1000.0; 
    lastTime = currentTime;

    // 1. Complementary Filter (Angle Fusion)
    float accPitch = atan2(myICM.accX(), myICM.accZ()) * 57.2958;
    float currentGyroY = gyroFilter.process(myICM.gyrY()); 
    filteredAngle = 0.96 * (filteredAngle + currentGyroY * dt) + 0.04 * accPitch;
    
    // 2. Load Cell Filtering
    float rawWeight = pressure_sensor.get_units(1);
    float smoothWeight = weightFilter.process(rawWeight);

    // 3. Phase Detection & State Machine
    detectPhase(smoothWeight, filteredAngle, currentGyroY);

    // 4. Calculate Motor Target based on Phase
    calculateMotorTarget(currentPhase, currentMotorSpeed, currentRequiredTorque);
    
    // 5. Actuate the Stepper (Timer handles pulses in the background)
    actuateMotor(currentMotorSpeed, currentRequiredTorque);

    // 6. Telemetry for Serial Plotter
    Serial.print("Phase:"); Serial.print(currentPhase * 10); 
    Serial.print(", Angle:"); Serial.print(filteredAngle);
    Serial.print(", MotorSPS:"); Serial.println(currentMotorSpeed);
    
  } else {
    // Do not block heavily here so loop stays responsive
    delay(2); 
  }
}
