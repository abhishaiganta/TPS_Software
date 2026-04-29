// ================================================================
//  TPS_BEDay_Final.ino
//  Triton Prosthetics Society — Bionic Knee Demo Firmware
//  UCSD Bioengineering Senior Design — BE Day 2026
//
//  MERGED FROM:
//    Abhishai : Blocks A2, A5, C, D3, D6, D7, D8, Setup, Main Loop
//               ICM-20948 IMU + HX711 sensor pipeline,
//               complementary filter, 8-phase sequential state machine,
//               User-weight-normalised phase thresholds,
//               auto-weight calibration, refined swing detection
//
//    Brennon  : Blocks A3, A4, B3, D1, D2
//               Position-based motor control (moveToSteps),
//               SensorData struct, PhaseOutput struct,
//               per-phase knee angle targets
//
//    Ben      : Blocks D4, D5
//               Median spike-resistant HX711 calibration
//
//  PHYSICAL SETUP (BE Day Demo):
//    IMU band   : wrapped around demonstrator's lower leg (shank)
//    Load cells : Wheatstone-bridge scale platform — person steps on it
//    Motor      : drives the 4-bar polycentric knee joint (bench-mounted)
//
//  LIBRARIES (Arduino Library Manager):
//    SparkFun ICM-20948 IMU  → search "SparkFun_ICM_20948_IMU"
//    HX711 by Rob Tillaart   → search "HX711_ADC Tillaart"
//
//  ★ COMPILE RULE: In Arduino/.ino files, every enum, struct, and
//    class MUST be declared before any function that uses it as a
//    parameter or return type. The auto-prototyper runs before your
//    code is processed, so unknown types in signatures = compile error.
//    All types are therefore at the TOP of this file.
// ================================================================

#include <Arduino.h>
#include <IntervalTimer.h>
#include "ICM_20948.h"
#include "HX711.h"

// ================================================================
// BLOCK A — ALL TYPE DECLARATIONS
//   Everything in this block (enums, structs, classes) must come
//   before the first function definition. Do not move anything
//   below here into a lower section.
// ================================================================

// ── A1. Hardware pin constants ───────────────────────────────────
constexpr uint8_t STEP_N_PIN = 5;
constexpr uint8_t DIR_N_PIN  = 6;
constexpr uint8_t EN_N_PIN   = 7;
constexpr uint8_t DATA_PIN   = 2;   // HX711 DT
constexpr uint8_t CLOCK_PIN  = 3;   // HX711 SCK

constexpr uint8_t DM_OFF = HIGH;    // active-LOW driver: OFF = HIGH
constexpr uint8_t DM_ON  = LOW;     // active-LOW driver: ON  = LOW

// ── A2. GaitPhase enum ──────────────────────────────────────────
//   uint8_t base keeps it small; explicit values make Serial output
//   unambiguous.
enum GaitPhase : uint8_t {
  INITIAL_CONTACT  = 0,
  LOADING_RESPONSE = 1,
  MID_STANCE       = 2,
  TERMINAL_STANCE  = 3,
  PRE_SWING        = 4,
  INITIAL_SWING    = 5,
  MID_SWING        = 6,
  TERMINAL_SWING   = 7
};

const char* phaseNames[] = {
  "INITIAL_CONTACT",
  "LOADING_RESPONSE",
  "MID_STANCE",
  "TERMINAL_STANCE",
  "PRE_SWING",
  "INITIAL_SWING",
  "MID_SWING",
  "TERMINAL_SWING"
};

// ── A3. SensorData struct ────────────────────────────────────────
//   Returned by readSensors() and consumed by detectPhase().
struct SensorData {
  float weightGrams;        // smoothed HX711 reading (grams)
  float weightNorm;         // weight / userWeight  (0.0 – 1.0+)
  float shankAngleDeg;      // complementary-filtered IMU pitch (deg)
  float shankVelDegPerSec;  // smoothed gyro Y rate  (deg/s)
};

// ── A4. PhaseOutput struct ───────────────────────────────────────
//   Returned by phaseToOutput() and consumed by the motor commands.
struct PhaseOutput {
  GaitPhase   phase;
  float       targetKneeAngleDeg;  // desired knee flexion for this phase
  float       targetSPS;           // motor step rate for this transition
  const char* modeLabel;           // human-readable label for Serial
};

// ── A5. MovingAverageFilter class ────────────────────────────────
//   Circular-buffer running average. O(1) per sample.
class MovingAverageFilter {
  float* buf;
  int    sz, idx;
  float  sum;
public:
  MovingAverageFilter(int size) : sz(size), idx(0), sum(0.0f) {
    buf = new float[size]();   // zero-initialised
  }
  float process(float v) {
    sum -= buf[idx];
    buf[idx] = v;
    sum += v;
    idx = (idx + 1) % sz;
    return sum / sz;
  }
};

// ================================================================
// BLOCK B — GLOBAL OBJECTS & STATE
// ================================================================

// ── B1. Peripheral objects ───────────────────────────────────────
#define WIRE_PORT Wire
#define AD0_VAL   1          // ICM-20948 I2C address select

ICM_20948_I2C myICM;
HX711         pressure_sensor;

// ── B2. Signal filters ───────────────────────────────────────────
MovingAverageFilter weightFilter(10);   // 10-sample load-cell smoother
MovingAverageFilter gyroFilter(5);      // 5-sample gyro smoother

// ── B3. Motor position state ──────────────────────────
constexpr long  MIN_STEPS     = 0;
constexpr long  MAX_STEPS     = 1200;   // 120° max knee flexion
constexpr float STEPS_PER_DEG = 10.0f; // 10 steps = 1°

IntervalTimer   stepTimer;
volatile long   currentSteps    = 0;
volatile long   targetSteps     = 0;
volatile bool   currentDirFwd   = true;
volatile bool   steppingEnabled = false;
volatile bool   stepState       = false;

// ── B4. IMU / angle state ────────────────────────────────────────
float         baselineAngle = 0.0f;
float         filteredAngle = 0.0f;
unsigned long lastTime      = 0;

// ── B5. Gait state ───────────────────────────────────────────────
GaitPhase currentPhase = MID_STANCE;
GaitPhase prevPhase    = MID_STANCE;

float userPaceFactor = 1.0f;
float peakSwingSpeed = 0.0f;

// ── B6. User weight calibration ────────────────────────
float userWeight      = 0.0f;
bool  weightCalibrated = false;

// ── B7. HX711 calibration values ────────────────────────────────
//   After running the calibration wizard once (press 'C' at boot),
//   paste the printed values here and re-flash.
int32_t hxOffset = 0;
float   hxScale  = -12.48f;   // negative wiring is inverted

// ================================================================
// BLOCK C — THRESHOLD CONSTANTS
//   Weight thresholds are fractions of userWeight.
//   Falls back to BENCH_CONTACT_G when no person is calibrated.
// ================================================================
constexpr float W_CONTACT_FRAC    = 0.10f;  // 10% bodyweight → contact
constexpr float W_LOADING_FRAC    = 0.20f;  // 20% → loading response
constexpr float W_MID_FRAC        = 0.40f;  // 40% → mid stance
constexpr float W_TERMINAL_FRAC   = 0.80f;  // 80% → terminal stance
constexpr float W_LIFTOFF_FRAC    = 0.08f;  //  8% → liftoff (hysteresis)

constexpr float BENCH_CONTACT_G   = 20.0f;  // bench demo: any object on plate

constexpr float ANGLE_LOADING_DEG  = 2.0f;  // min angle shift → loading (bench)
constexpr float ANGLE_TERMINAL_DEG = -5.0f; // forward lean → terminal stance (bench)
constexpr float MAX_SWING_ANGLE    = 65.0f; // max expected knee flexion

constexpr float VEL_SETTLE       = 5.0f;    // gyro quiet → stance settled
constexpr float VEL_SWING_ENTRY  = 30.0f;   // fast → enter swing
constexpr float VEL_SWING_PEAK   = 10.0f;   // slowing → terminal swing

// ================================================================
// BLOCK D — FUNCTION DEFINITIONS
//   All types are fully declared above, so the auto-prototyper
//   will never encounter an unknown type in any signature.
// ================================================================

// ── D1. Motor helpers ────────────────────────────────────────────

void enableDriver(bool on) {
  digitalWriteFast(EN_N_PIN, on ? DM_ON : DM_OFF);
}

void setDirection(bool forward) {
  digitalWriteFast(DIR_N_PIN, forward ? DM_ON : DM_OFF);
}

// ISR: toggle pattern — fires at 2× SPS, two fires = one full step
void stepISR() {
  if (!steppingEnabled) return;

  long cs = currentSteps;
  long ts = targetSteps;

  if (cs == ts) {
    steppingEnabled = false;
    stepState = false;
    digitalWriteFast(STEP_N_PIN, DM_OFF);
    return;
  }

  stepState = !stepState;
  digitalWriteFast(STEP_N_PIN, stepState ? DM_ON : DM_OFF);

  if (stepState) {
    currentSteps += currentDirFwd ? 1 : -1;
  }
}

void setSpeedSPS(float sps) {
  if (sps <= 0.0f) {
    stepTimer.end();
    noInterrupts();
    steppingEnabled = false;
    stepState       = false;
    interrupts();
    digitalWriteFast(STEP_N_PIN, DM_OFF);
    return;
  }
  // ISR fires at 2× SPS (toggle pattern)
  uint32_t period_us = (uint32_t)(1e6f / (sps * 2.0f));
  stepTimer.begin(stepISR, period_us);
}

long angleDegToSteps(float angleDeg) {
  return constrain((long)(angleDeg * STEPS_PER_DEG), MIN_STEPS, MAX_STEPS);
}

// Move motor to a step position at a given speed.
// Stops automatically when target is reached (ISR-driven).
void moveToSteps(long newTarget, float sps) {
  newTarget = constrain(newTarget, MIN_STEPS, MAX_STEPS);

  noInterrupts();
  long cs = currentSteps;
  interrupts();

  if (newTarget == cs) {
    setSpeedSPS(0.0f);
    enableDriver(true);   // hold torque at position
    return;
  }

  bool dirFwd = (newTarget > cs);
  setDirection(dirFwd);

  noInterrupts();
  targetSteps     = newTarget;
  currentDirFwd   = dirFwd;
  steppingEnabled = true;
  interrupts();

  enableDriver(true);
  setSpeedSPS(sps);
}

// Convenience: move to a knee angle in degrees
void moveToAngle(float angleDeg, float sps) {
  moveToSteps(angleDegToSteps(angleDeg), sps);
}

// ── D2. Phase → knee angle + speed table ──────────────
//   Maps each gait phase to a target knee angle (degrees) and
//   motor speed (SPS). Angles from biomechanics literature.
//   Swing phases scaled by userPaceFactor.

PhaseOutput phaseToOutput(GaitPhase phase) {
  PhaseOutput out;
  out.phase = phase;

  switch (phase) {
    case INITIAL_CONTACT:
      out.targetKneeAngleDeg = 5.0f;
      out.targetSPS          = 200.0f;
      out.modeLabel          = "INITIAL_CONTACT [5deg]";
      break;

    case LOADING_RESPONSE:
      out.targetKneeAngleDeg = 10.0f;
      out.targetSPS          = 250.0f;
      out.modeLabel          = "LOADING_RESPONSE [10deg]";
      break;

    case MID_STANCE:
      out.targetKneeAngleDeg = 5.0f;
      out.targetSPS          = 180.0f;
      out.modeLabel          = "MID_STANCE [5deg hold]";
      break;

    case TERMINAL_STANCE:
      out.targetKneeAngleDeg = 0.0f;
      out.targetSPS          = 220.0f;
      out.modeLabel          = "TERMINAL_STANCE [0deg extend]";
      break;

    case PRE_SWING:
      out.targetKneeAngleDeg = 20.0f;
      out.targetSPS          = 350.0f;
      out.modeLabel          = "PRE_SWING [20deg]";
      break;

    case INITIAL_SWING:
      out.targetKneeAngleDeg = 40.0f;
      out.targetSPS          = 500.0f * userPaceFactor;
      out.modeLabel          = "INITIAL_SWING [40deg]";
      break;

    case MID_SWING:
      out.targetKneeAngleDeg = 60.0f;
      out.targetSPS          = 550.0f * userPaceFactor;
      out.modeLabel          = "MID_SWING [60deg peak]";
      break;

    case TERMINAL_SWING:
      out.targetKneeAngleDeg = 15.0f;
      out.targetSPS          = 300.0f * userPaceFactor;
      out.modeLabel          = "TERMINAL_SWING [15deg return]";
      break;

    default:
      out.targetKneeAngleDeg = 0.0f;
      out.targetSPS          = 120.0f;
      out.modeLabel          = "SAFE_HOME [0deg]";
      break;
  }
  return out;
}

// ── D3. Gait phase state machine ────────────
//   Sequential: can only move to the next phase in the cycle.
//   Weight thresholds use bodyweight fractions when calibrated,
//   or absolute gram values for bench demo.

void detectPhase(const SensorData& s) {
  float contactThreshold, liftoffThreshold;

  if (weightCalibrated && userWeight > 50.0f) {
    contactThreshold = userWeight * W_CONTACT_FRAC;
    liftoffThreshold = userWeight * W_LIFTOFF_FRAC;
  } else {
    contactThreshold = BENCH_CONTACT_G;
    liftoffThreshold = BENCH_CONTACT_G * 0.4f;
  }

  bool  isContact = (s.weightGrams > contactThreshold);
  bool  isLiftoff = (s.weightGrams < liftoffThreshold);
  float relAngle  = s.shankAngleDeg - baselineAngle;

  switch (currentPhase) {

    case TERMINAL_SWING:
      if (isContact) currentPhase = INITIAL_CONTACT;
      break;

    case INITIAL_CONTACT:
      if (weightCalibrated) {
        if (s.weightGrams > userWeight * W_LOADING_FRAC)
          currentPhase = LOADING_RESPONSE;
      } else {
        if (fabsf(relAngle) > ANGLE_LOADING_DEG)
          currentPhase = LOADING_RESPONSE;
      }
      break;

    case LOADING_RESPONSE:
      if (weightCalibrated) {
        if (s.weightGrams > userWeight * W_MID_FRAC && isContact)
          currentPhase = MID_STANCE;
      } else {
        if (fabsf(s.shankVelDegPerSec) < VEL_SETTLE && isContact)
          currentPhase = MID_STANCE;
      }
      break;

    case MID_STANCE:
      if (weightCalibrated) {
        if (s.weightGrams > userWeight * W_TERMINAL_FRAC)
          currentPhase = TERMINAL_STANCE;
      } else {
        if (relAngle < ANGLE_TERMINAL_DEG)
          currentPhase = TERMINAL_STANCE;
      }
      break;

    case TERMINAL_STANCE:
      if (isLiftoff) {
        currentPhase   = PRE_SWING;
        peakSwingSpeed = 0.0f;
      }
      break;

    case PRE_SWING:
      if (fabsf(s.shankVelDegPerSec) > VEL_SWING_ENTRY && isLiftoff)
        currentPhase = INITIAL_SWING;
      break;

    case INITIAL_SWING: {
      if (fabsf(s.shankVelDegPerSec) > peakSwingSpeed)
        peakSwingSpeed = fabsf(s.shankVelDegPerSec);

      // Adapt motor speed to user's natural walking pace
      userPaceFactor = constrain(peakSwingSpeed / 100.0f, 0.5f, 1.5f);

      // swing angle = baseline − current (positive as leg swings forward)
      float swingAngle = baselineAngle - s.shankAngleDeg;
      if (swingAngle > (MAX_SWING_ANGLE * 0.8f) && isLiftoff)
        currentPhase = MID_SWING;
      break;
    }

    case MID_SWING: {
      float swingAngle = baselineAngle - s.shankAngleDeg;
      if (swingAngle > 20.0f && isLiftoff &&
          fabsf(s.shankVelDegPerSec) < VEL_SWING_PEAK)
        currentPhase = TERMINAL_SWING;
      break;
    }
  }
}

// ── D4. Median calibration helper ─────────────────────────
//   n raw HX711 readings → sorted → middle value returned.
//   A single spike cannot affect the median.

long getMedian(int n) {
  long* samples = new long[n];
  for (int i = 0; i < n; i++) {
    samples[i] = pressure_sensor.read();
    delay(5);
  }
  for (int i = 0; i < n - 1; i++) {
    for (int j = i + 1; j < n; j++) {
      if (samples[j] < samples[i]) {
        long tmp   = samples[i];
        samples[i] = samples[j];
        samples[j] = tmp;
      }
    }
  }
  long med = samples[n / 2];
  delete[] samples;
  return med;
}

// ── D5. HX711 calibration wizard ────────────────────────────────
//   Interactive Serial calibration. Run once. Paste output into B7.

void runHX711Calibration() {
  Serial.println("\n==============================");
  Serial.println("   HX711 CALIBRATION WIZARD   ");
  Serial.println("==============================");

  Serial.println("Step 1: Remove ALL weight from load cells.");
  Serial.println("        Press ENTER when platform is empty...");
  while (Serial.available()) Serial.read();
  while (!Serial.available());
  while (Serial.available()) Serial.read();

  Serial.println("  Measuring zero offset (20-sample median)...");
  hxOffset = getMedian(20);
  pressure_sensor.set_offset(hxOffset);
  Serial.print("  OFFSET = "); Serial.println(hxOffset);

  Serial.println("\nStep 2: Place a known weight on the platform.");
  Serial.println("        Type weight in GRAMS, then press ENTER:");

  uint32_t knownGrams = 0;
  while (Serial.peek() != '\n') {
    if (Serial.available()) {
      char c = Serial.read();
      if (isdigit(c)) knownGrams = knownGrams * 10 + (c - '0');
    }
  }
  while (Serial.available()) Serial.read();

  Serial.print("  Known weight: "); Serial.print(knownGrams); Serial.println(" g");
  Serial.println("  Measuring loaded value...");

  long loadedRaw = getMedian(20);
  long diff      = loadedRaw - hxOffset;

  if (diff == 0) {
    Serial.println("  ERROR: No change detected — check wiring!");
    return;
  }

  hxScale = (float)knownGrams / (float)diff;
  pressure_sensor.set_scale(hxScale);

  Serial.println("\n-- Paste these into Block B7 of the sketch --");
  Serial.print("int32_t hxOffset = "); Serial.print(hxOffset);   Serial.println(";");
  Serial.print("float   hxScale  = "); Serial.print(hxScale, 6); Serial.println(";");
  Serial.println("----------------------------------------------\n");
  delay(2000);
}

// ── D6. User weight auto-calibration ───────────────────
//   Person stands still on platform for 3 s.
//   All stance thresholds then auto-scale to their bodyweight.

void calibrateUserWeight() {
  Serial.println("\n-- USER WEIGHT CALIBRATION --");
  Serial.println("Stand STILL on platform for 3 seconds...");
  delay(500);

  float weightSum = 0.0f;
  int   count     = 0;
  unsigned long start = millis();

  while (millis() - start < 3000) {
    if (pressure_sensor.is_ready()) {
      float w = pressure_sensor.get_units(3);
      if (w > 0.0f) { weightSum += w; count++; }
    }
    delay(50);
  }

  if (count > 0 && (weightSum / count) > 100.0f) {
    userWeight       = weightSum / count;
    weightCalibrated = true;
    Serial.print("  User weight: "); Serial.print(userWeight, 1); Serial.println(" g");
    Serial.println("  Stance thresholds are now bodyweight-normalised.");
  } else {
    weightCalibrated = false;
    Serial.println("  Weight too low — using bench demo thresholds (20g contact).");
  }
}

// ── D7. IMU baseline calibration ─────────────────────

void calibrateIMU() {
  Serial.println("IMU Cal: Keep device STILL for 2 seconds...");
  delay(2000);

  float angleSum = 0.0f;
  int   count    = 0;

  for (int i = 0; i < 50; i++) {
    if (myICM.dataReady()) {
      myICM.getAGMT();
      angleSum += atan2f(myICM.accX(), myICM.accZ()) * 57.2958f;
      count++;
    }
    delay(20);
  }

  baselineAngle = (count > 0) ? (angleSum / count) : 0.0f;
  filteredAngle = baselineAngle;
  lastTime      = millis();

  Serial.print("  Baseline angle: "); Serial.print(baselineAngle, 2); Serial.println(" deg");
}

// ── D8. Sensor read + complementary filter ───────────
//   Returns a fully populated SensorData struct.
//   Complementary filter: 96% gyro integration + 4% accel correction.

SensorData readSensors() {
  SensorData s;
  s.weightGrams       = 0.0f;
  s.weightNorm        = 0.0f;
  s.shankAngleDeg     = filteredAngle;   // keep last value if IMU not ready
  s.shankVelDegPerSec = 0.0f;

  // ── IMU ──
  if (myICM.dataReady()) {
    myICM.getAGMT();

    unsigned long now = millis();
    float dt = (now - lastTime) / 1000.0f;
    if (dt < 0.001f) dt = 0.001f;
    lastTime = now;

    float accPitch = atan2f(myICM.accX(), myICM.accZ()) * 57.2958f;
    float gyroRate = gyroFilter.process(myICM.gyrY());

    filteredAngle = 0.96f * (filteredAngle + gyroRate * dt)
                  + 0.04f * accPitch;

    s.shankAngleDeg     = filteredAngle;
    s.shankVelDegPerSec = gyroRate;
  }

  // ── Load cell ──
  if (pressure_sensor.is_ready()) {
    float raw    = pressure_sensor.get_units(1);
    float smooth = weightFilter.process(raw);
    s.weightGrams = smooth;
    s.weightNorm  = (weightCalibrated && userWeight > 0.0f)
                    ? (smooth / userWeight)
                    : 0.0f;
  }

  return s;
}

// ================================================================
// SETUP
// ================================================================
void setup() {
  Serial.begin(115200);
  delay(500);

  // Motor pins — start with driver disabled for safety
  pinMode(STEP_N_PIN, OUTPUT);
  pinMode(DIR_N_PIN,  OUTPUT);
  pinMode(EN_N_PIN,   OUTPUT);
  digitalWriteFast(STEP_N_PIN, DM_OFF);
  digitalWriteFast(DIR_N_PIN,  DM_OFF);
  enableDriver(false);

  // I2C + IMU
  WIRE_PORT.begin();
  WIRE_PORT.setClock(400000);

  Serial.println("Connecting to ICM-20948...");
  while (true) {
    myICM.begin(WIRE_PORT, AD0_VAL);
    if (myICM.status == ICM_20948_Stat_Ok) break;
    Serial.println("  Not found — retrying...");
    delay(500);
  }
  Serial.println("  IMU OK.");

  // Load cell — apply stored calibration from Block B7
  pressure_sensor.begin(DATA_PIN, CLOCK_PIN);
  pressure_sensor.set_offset(hxOffset);
  pressure_sensor.set_scale(hxScale);

  // ── Startup menu (8-second window) ──────────────────────────
  Serial.println("\n================================");
  Serial.println("  TPS BIONIC KNEE  |  BE DAY   ");
  Serial.println("  Triton Prosthetics Society    ");
  Serial.println("================================");
  Serial.println("C     — HX711 calibration wizard");
  Serial.println("W     — calibrate user bodyweight");
  Serial.println("ENTER — skip, use stored values\n");

  unsigned long menuStart = millis();
  while (millis() - menuStart < 8000) {
    if (Serial.available()) {
      char ch = toupper((char)Serial.read());
      if (ch == 'C') { runHX711Calibration(); break; }
      if (ch == 'W') { calibrateUserWeight();  break; }
      if (ch == '\n' || ch == '\r') break;
    }
  }

  // IMU baseline
  calibrateIMU();

  // Enable driver and home the knee to full extension
  enableDriver(true);
  moveToAngle(0.0f, 200.0f);
  delay(500);

  Serial.println("\nSystem READY — main loop starting.");
  Serial.println("Columns: Weight(g),WeightNorm,Angle(deg),Gyro(dps),Phase*10,KneeTarget(deg),MotorSPS,Steps");
}

// ================================================================
// MAIN LOOP  (~50 Hz)
// ================================================================
void loop() {
  // 1. Read all sensors
  SensorData s = readSensors();

  // 2. Run state machine
  detectPhase(s);

  // 3. Get motor command for current phase
  PhaseOutput out = phaseToOutput(currentPhase);

  // 4. Command motor to target angle
  moveToAngle(out.targetKneeAngleDeg, out.targetSPS);

  // 5. Read motor position (interrupt-safe)
  noInterrupts();
  long cs = currentSteps;
  //long ts = targetSteps;
  interrupts();

  // 6. Print phase change notification
  if (currentPhase != prevPhase) {
    Serial.print("\n>>> PHASE: ");
    Serial.print(phaseNames[prevPhase]);
    Serial.print(" -> ");
    Serial.println(phaseNames[currentPhase]);
    Serial.print("    Knee target: "); Serial.print(out.targetKneeAngleDeg, 1); Serial.println(" deg");
    Serial.print("    Motor SPS:   "); Serial.print(out.targetSPS, 0);          Serial.println(" sps");
    Serial.print("    Mode:        "); Serial.println(out.modeLabel);
    prevPhase = currentPhase;
  }

  // 7. CSV telemetry — open Serial Plotter to visualise live
  Serial.print(s.weightGrams,       2); Serial.print(",");
  Serial.print(s.weightNorm,        3); Serial.print(",");
  Serial.print(s.shankAngleDeg,     2); Serial.print(",");
  Serial.print(s.shankVelDegPerSec, 2); Serial.print(",");
  Serial.print(currentPhase * 10);      Serial.print(",");
  Serial.print(out.targetKneeAngleDeg); Serial.print(",");
  Serial.print(out.targetSPS,       1); Serial.print(",");
  Serial.println(cs);

  delay(20);   // 50 Hz loop rate
}
