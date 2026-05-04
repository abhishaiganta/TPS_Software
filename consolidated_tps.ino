// =============================================================================
// BIONIC LEG PROTOTYPE — WALKING GAIT PHASE CONTROLLER
// =============================================================================
//
// HARDWARE SUMMARY:
//   - ICM-20948 IMU  : Strapped to user's calf via I2C (Wire)
//   - HX711 Load Cell: 4-cell plate under foot (DATA=pin 2, CLK=pin 3)
//   - Stepper Motor  : Controls knee joint angle (STEP=5, DIR=6, EN=7)
//   - Teensy (or compatible Arduino with IntervalTimer & digitalWriteFast)
//
// SYSTEM FLOW:
//   1. CALIBRATE  — Find the HX711 scale factor (run once, paste result into code)
//   2. STARTUP    — Record user weight baseline + IMU angle baseline
//   3. DEMO       — Press Enter to start live data; walk on/off plate to cycle phases
//
// GAIT PHASES DETECTED:
//   TERMINAL_SWING  → INITIAL_CONTACT → LOADING_RESPONSE → MID_STANCE
//   → TERMINAL_STANCE → PRE_SWING → INITIAL_SWING → MID_SWING → TERMINAL_SWING
//
// MOTOR BEHAVIOUR PER PHASE:
//   Stance phases  : Motor OFF (leg straight/locked under load)
//   Terminal/Pre-  : Low-speed flex (push-off assist)
//   Initial Swing  : Motor ON forward (knee flexes to ~60°)
//   Mid Swing      : Motor reverses direction (knee extends back)
//   Terminal Swing : Motor brakes to stop (leg straight for heel-strike)
//
// =============================================================================

#include <Arduino.h>
#include <IntervalTimer.h>
#include "ICM_20948.h"   // SparkFun ICM-20948 library
#include "HX711.h"       // HX711 load cell amplifier library

// =============================================================================
// SECTION 1 — PIN DEFINITIONS & HARDWARE CONSTANTS
// =============================================================================

// --- Stepper Motor Driver (e.g. DRV8825 / A4988) ---
constexpr uint8_t STEP_N_PIN = 5;   // Step pulse output
constexpr uint8_t DIR_N_PIN  = 6;   // Direction output
constexpr uint8_t EN_N_PIN   = 7;   // Enable (active LOW on most drivers)

// --- HX711 Load Cell Amplifier ---
constexpr uint8_t HX711_DATA_PIN  = 2;   // DT pin
constexpr uint8_t HX711_CLOCK_PIN = 3;   // SCK pin

// --- Driver logic polarity ---
// Most stepper drivers: EN LOW = driver ON, EN HIGH = driver OFF
constexpr uint8_t DM_OFF = HIGH;
constexpr uint8_t DM_ON  = LOW;

// --- Step pulse width (microseconds) ---
// DRV8825 minimum STEP HIGH pulse = 1.9 µs; 5 µs is safe
constexpr uint32_t STEP_PULSE_US = 5;

// =============================================================================
// SECTION 2 — MOTOR POSITION & SPEED LIMITS
// =============================================================================

// Mechanical travel: 0 steps = fully straight (0°), MAX_STEPS = fully flexed
// IMPORTANT: Measure how many steps take the knee from 0° to full flexion
// and update MAX_SAFE_STEPS before running with mechanical team on Friday.
// Current value: 70° × 10 steps/degree = 700 steps (PLACEHOLDER — verify!)
constexpr float STEPS_PER_DEG    = 10.0f;
constexpr float MAX_KNEE_DEG     = 70.0f;   // Do NOT exceed mechanical limit
constexpr float MIN_KNEE_DEG     = 0.0f;
constexpr long  MIN_STEPS        = 0;
constexpr long  MAX_SAFE_STEPS   = (long)(MAX_KNEE_DEG * STEPS_PER_DEG); // 700

// Speed cap — increase after verifying mechanical clearance with team
constexpr float MAX_SAFE_SPS     = 600.0f;  // Steps per second

// =============================================================================
// SECTION 3 — IMU SETUP
// =============================================================================

#define WIRE_PORT Wire        // I2C bus for ICM-20948
#define AD0_VAL   1           // IMU I2C address select (1 → 0x69, 0 → 0x68)

ICM_20948_I2C myICM;

// Complementary filter state (blends gyro integration with accelerometer)
// Coefficient: 0.96 = 96% gyro trust, 4% accel correction
float filteredAngle = 0.0f;
float baselineAngle = 0.0f;   // Captured during startup (standing still)
unsigned long lastImuTime = 0;

// =============================================================================
// SECTION 4 — LOAD CELL SETUP
// =============================================================================

HX711 loadCell;

// *** CALIBRATION: Replace -12.48 with YOUR scale factor from the 'c' menu ***
// Run the calibration sequence, read the printed value, paste it here, re-upload.
constexpr float LOAD_CELL_SCALE = -12.48f;

// User weight is captured live during startup — no hard-coding needed
float userWeightGrams = 70000.0f;  // fallback (never used after startup)

// =============================================================================
// SECTION 5 — MOVING-AVERAGE SMOOTHING FILTERS
// =============================================================================
// A small window (2 samples) removes single-sample noise without adding
// much latency. Increase the window if signals are noisy in testing.

class MovingAverageFilter {
private:
    float*  buf;
    int     windowSize;
    int     idx      = 0;
    float   sum      = 0.0f;
public:
    MovingAverageFilter(int size) : windowSize(size) {
        buf = new float[size]();   // zero-initialised
    }
    float process(float v) {
        sum    -= buf[idx];
        buf[idx] = v;
        sum    += v;
        idx     = (idx + 1) % windowSize;
        return sum / windowSize;
    }
};

MovingAverageFilter weightFilter(4);  // Smooth the load cell reading
MovingAverageFilter gyroFilter(2);    // Smooth raw gyro Y before integration

// Cache of last smoothed sensor values (updated in loop())
float smoothWeight = 0.0f;
float currentGyroY = 0.0f;

// =============================================================================
// SECTION 6 — GAIT PHASE ENUM & STATE
// =============================================================================

enum GaitPhase {
    INITIAL_CONTACT,    // 0 — heel touches plate; weight starts loading
    LOADING_RESPONSE,   // 1 — weight rapidly increases
    MID_STANCE,         // 2 — full body weight over foot
    TERMINAL_STANCE,    // 3 — weight shifts to forefoot; heel rising
    PRE_SWING,          // 4 — toe still on ground; knee starts to flex
    INITIAL_SWING,      // 5 — foot lifts; knee flexes to ~60°
    MID_SWING,          // 6 — peak knee flexion; leg swings forward
    TERMINAL_SWING      // 7 — knee extends; leg prepares for next heel-strike
};

GaitPhase currentPhase  = TERMINAL_SWING;  // Start in swing (foot lifted)
GaitPhase previousPhase = TERMINAL_SWING;

// =============================================================================
// SECTION 7 — STEPPER MOTOR INTERRUPT DRIVER
// =============================================================================
// Uses Teensy IntervalTimer to generate step pulses in the background.
// The ISR fires at a rate of (targetSPS × 2) so each on/off toggle =
// one complete step pulse.

IntervalTimer stepTimer;

// These are shared between ISR and main loop — volatile for safety
volatile long  stepPosition    = 0;     // Current step count (encoder substitute)
volatile long  stepTarget      = 0;     // Where we want to go
volatile bool  stepDirForward  = true;  // Current direction
volatile bool  steppingActive  = false; // Is the timer actively stepping?

// Called at 2× the desired step rate (one toggle per call)
void stepISR() {
    if (!steppingActive) return;

    // Stop when we reach the target
    if (stepPosition == stepTarget) {
        steppingActive = false;
        digitalWriteFast(STEP_N_PIN, DM_OFF);
        return;
    }

    // Generate one step pulse (HIGH → LOW)
    digitalWriteFast(STEP_N_PIN, DM_ON);
    delayMicroseconds(STEP_PULSE_US);
    digitalWriteFast(STEP_N_PIN, DM_OFF);

    // Track position
    if (stepDirForward) stepPosition++;
    else                stepPosition--;
}

// =============================================================================
// SECTION 8 — MOTOR CONTROL HELPERS
// =============================================================================

void enableDriver(bool on) {
    digitalWriteFast(EN_N_PIN, on ? DM_ON : DM_OFF);
}

void setDirection(bool forward) {
    noInterrupts();
    stepDirForward = forward;
    interrupts();
    digitalWriteFast(DIR_N_PIN, forward ? DM_ON : DM_OFF);
}

// Set ISR firing rate to achieve the requested steps-per-second
void setSpeedSPS(float sps) {
    if (sps <= 0.0f) {
        stepTimer.end();
        noInterrupts();
        steppingActive = false;
        interrupts();
        digitalWriteFast(STEP_N_PIN, DM_OFF);
        return;
    }
    sps = constrain(sps, 1.0f, MAX_SAFE_SPS);
    // Period in µs for ISR. ISR fires twice per step (HIGH + LOW).
    uint32_t period_us = (uint32_t)(1e6f / sps);
    stepTimer.begin(stepISR, period_us);
}

// Convert a desired knee angle (degrees) to a step position
long angleDegToSteps(float deg) {
    deg = constrain(deg, MIN_KNEE_DEG, MAX_KNEE_DEG);
    return constrain((long)(deg * STEPS_PER_DEG), MIN_STEPS, MAX_SAFE_STEPS);
}

// Move the motor to an absolute step position at the given speed
void moveToPosition(long targetPos, float sps) {
    targetPos = constrain(targetPos, MIN_STEPS, MAX_SAFE_STEPS);
    sps       = constrain(sps, 0.0f, MAX_SAFE_SPS);

    noInterrupts();
    long current = stepPosition;
    interrupts();

    if (current == targetPos) return;  // Already there

    bool forward = (targetPos > current);
    setDirection(forward);

    noInterrupts();
    stepTarget    = targetPos;
    steppingActive = true;
    interrupts();

    setSpeedSPS(sps);
}

// Convenience: move to a knee angle in degrees
void moveToAngle(float deg, float sps) {
    moveToPosition(angleDegToSteps(deg), sps);
}

// Hard stop — disables driver and kills timer (use for errors / end of demo)
void emergencyStop() {
    stepTimer.end();
    noInterrupts();
    steppingActive = false;
    interrupts();
    digitalWriteFast(STEP_N_PIN, DM_OFF);
    enableDriver(false);
    Serial.println("[EMERGENCY STOP]");
}

// =============================================================================
// SECTION 9 — PER-PHASE MOTOR BEHAVIOUR
// =============================================================================
// Maps each gait phase to a target knee angle and movement speed.
// These values are intentionally conservative for the first demo.
// Tune after the mechanical team verifies travel limits on Friday.
//
// Motor logic summary:
//   STANCE phases   → 0° (straight), driver stays OFF (passive load bearing)
//   PRE_SWING       → 20°, slow flex to initiate lift-off
//   INITIAL_SWING   → 60°, forward direction, knee bends fully
//   MID_SWING       → reverse direction, knee begins extending
//   TERMINAL_SWING  → 0°, brake back to straight for heel-strike

void applyMotorForPhase(GaitPhase phase) {
    switch (phase) {

        // ---- STANCE PHASES: motor OFF, leg passive under body weight ----
        case INITIAL_CONTACT:
            // Yield slightly under impact — very slow, driver ON
            enableDriver(true);
            moveToAngle(10.0f, 150.0f);  // small flex to absorb shock
            break;

        case LOADING_RESPONSE:
            // Resist further flexion — hold near 10°
            enableDriver(true);
            moveToAngle(10.0f, 100.0f);
            break;

        case MID_STANCE:
            // Full body weight — lock knee at 5° (nearly straight)
            enableDriver(true);
            moveToAngle(5.0f, 80.0f);
            break;

        case TERMINAL_STANCE:
            // Weight shifting off — hold straight
            enableDriver(true);
            moveToAngle(0.0f, 150.0f);
            break;

        // ---- PRE_SWING: begin flex before toe-off ----
        case PRE_SWING:
            enableDriver(true);
            setDirection(true);           // Forward = flex direction
            moveToAngle(20.0f, 300.0f);   // Gentle pre-flex
            break;

        // ---- INITIAL_SWING: knee bends to peak flexion (~60°) ----
        case INITIAL_SWING:
            enableDriver(true);
            setDirection(true);           // Forward = flex
            moveToAngle(60.0f, 500.0f);   // Drive to peak bend
            break;

        // ---- MID_SWING: reverse motor to extend knee ----
        case MID_SWING:
            enableDriver(true);
            setDirection(false);          // Reverse = extend
            moveToAngle(30.0f, 450.0f);   // Begin extending from 60° → 30°
            break;

        // ---- TERMINAL_SWING: brake to straight, ready for heel-strike ----
        case TERMINAL_SWING:
            enableDriver(true);
            setDirection(false);          // Reverse = extend
            moveToAngle(0.0f, 300.0f);    // Return to fully straight
            break;
    }
}

// =============================================================================
// SECTION 10 — GAIT PHASE DETECTION
// =============================================================================
// Uses only load cell weight and IMU angle (no time-based assumptions).
// Thresholds are expressed as fractions of the measured userWeightGrams.
//
// Weight thresholds (tuned for stepping on/off a plate in a demo):
//   >10% body weight  = foot is weight-bearing (stance)
//   >30% body weight  = load response active
//   >60% body weight  = mid-stance (full weight over foot)
//   <5%  body weight  = foot has lifted (swing)
//
// Angle thresholds use the *relative* angle (current − baseline):
//   >10° relative tilt = leg has swung back (initial swing)
//   >20° relative tilt = leg is at peak swing (mid swing)
//   <10° relative tilt = leg is returning (terminal swing)

const char* phaseName(GaitPhase p) {
    switch (p) {
        case INITIAL_CONTACT:   return "INITIAL_CONTACT";
        case LOADING_RESPONSE:  return "LOADING_RESPONSE";
        case MID_STANCE:        return "MID_STANCE";
        case TERMINAL_STANCE:   return "TERMINAL_STANCE";
        case PRE_SWING:         return "PRE_SWING";
        case INITIAL_SWING:     return "INITIAL_SWING";
        case MID_SWING:         return "MID_SWING";
        case TERMINAL_SWING:    return "TERMINAL_SWING";
        default:                return "UNKNOWN";
    }
}

void detectPhase(float weight, float angle) {
    bool isWeightBearing = weight > (userWeightGrams * 0.10f); // ≥10% BW
    bool isFullyLoaded   = weight > (userWeightGrams * 0.60f); // ≥60% BW
    bool isUnloaded      = weight < (userWeightGrams * 0.05f); // <5%  BW

    float relAngle = angle - baselineAngle;   // Angle relative to standing still

    switch (currentPhase) {

        case TERMINAL_SWING:
            // Heel strikes plate → any weight-bearing triggers Initial Contact
            if (isWeightBearing) currentPhase = INITIAL_CONTACT;
            break;

        case INITIAL_CONTACT:
            // Weight quickly rises past 30% → Loading Response
            if (weight > (userWeightGrams * 0.30f)) currentPhase = LOADING_RESPONSE;
            break;

        case LOADING_RESPONSE:
            // Weight continues rising past 60% → Mid Stance
            if (isFullyLoaded) currentPhase = MID_STANCE;
            break;

        case MID_STANCE:
            // Body shifts forward; weight drops below 60% → Terminal Stance
            if (weight < (userWeightGrams * 0.60f)) currentPhase = TERMINAL_STANCE;
            break;

        case TERMINAL_STANCE:
            // Foot lifts off completely → Pre-Swing
            if (isUnloaded) currentPhase = PRE_SWING;
            break;

        case PRE_SWING:
            // Shank tilts back ≥10° as leg swings → Initial Swing
            if (abs(relAngle) > 10.0f) currentPhase = INITIAL_SWING;
            break;

        case INITIAL_SWING:
            // Shank reaches ≥20° tilt = peak swing → Mid Swing
            if (abs(relAngle) > 20.0f) currentPhase = MID_SWING;
            break;

        case MID_SWING:
            // Shank returns within 10° of baseline → Terminal Swing (leg straightening)
            if (abs(relAngle) < 10.0f) currentPhase = TERMINAL_SWING;
            break;
    }

    // Print a clearly visible banner on every phase transition
    if (currentPhase != previousPhase) {
        Serial.println();
        Serial.println("╔══════════════════════════════════╗");
        Serial.print  ("║  PHASE: ");
        Serial.print  (phaseName(currentPhase));
        Serial.println("  ║");
        Serial.println("╚══════════════════════════════════╝");
        previousPhase = currentPhase;
    }
}

// =============================================================================
// SECTION 11 — SERIAL UTILITY
// =============================================================================

// Discard any bytes left in the serial receive buffer
void flushSerialInput() {
    while (Serial.available()) { Serial.read(); delay(2); }
}

// Block until a specific character arrives; flush everything else
void waitForChar(char c) {
    Serial.print("(waiting for '"); Serial.print(c); Serial.println("')");
    while (true) {
        if (Serial.available()) {
            char rx = Serial.read();
            if (rx == c) { flushSerialInput(); return; }
        }
    }
}

// =============================================================================
// SECTION 12 — CALIBRATION SEQUENCE
// =============================================================================
// PURPOSE: Find the HX711 scale factor so that get_units() returns grams.
//
// HOW TO USE:
//   1. Upload this code.
//   2. Open Serial Monitor at 9600 baud.
//   3. Type 'c' and press Enter.
//   4. Follow the on-screen instructions.
//   5. Copy the printed scale factor.
//   6. Paste it into LOAD_CELL_SCALE at the top of this file.
//   7. Re-upload — calibration complete!

void runLoadCellCalibration() {
    Serial.println();
    Serial.println("=== LOAD CELL CALIBRATION MODE ===");
    Serial.println("Step 1: Make sure the plate is EMPTY.");
    Serial.println("Press 'y' then Enter when ready to tare (zero) the sensor.");
    waitForChar('y');

    loadCell.tare();
    Serial.println("Tare complete — plate is now zeroed.");
    Serial.println();

    Serial.println("Step 2: Place a KNOWN weight on the plate.");
    Serial.println("Press 'y' then Enter when the weight is stable.");
    waitForChar('y');

    Serial.println("Reading raw values (20 samples)...");
    long rawAvg = loadCell.read_average(20);
    Serial.print("Raw average: "); Serial.println(rawAvg);
    Serial.println();

    Serial.println("Step 3: Enter the known weight in GRAMS, then press Enter.");
    Serial.println("(e.g. type  500  for a 500g calibration weight)");
    while (!Serial.available());
    float knownGrams = Serial.parseFloat();
    flushSerialInput();

    float scaleFactor = (float)rawAvg / knownGrams;

    Serial.println();
    Serial.println("╔══════════════════════════════════════════════════╗");
    Serial.println("║           CALIBRATION RESULT                     ║");
    Serial.print  ("║  Scale factor = ");
    Serial.print  (scaleFactor, 4);
    Serial.println("                              ║");
    Serial.println("╠══════════════════════════════════════════════════╣");
    Serial.println("║  1. Copy the number above.                       ║");
    Serial.println("║  2. Paste into LOAD_CELL_SCALE at top of file.  ║");
    Serial.println("║  3. Re-upload the code.                          ║");
    Serial.println("╚══════════════════════════════════════════════════╝");
    Serial.println();
    Serial.println("SYSTEM HALTED. Re-upload with new scale factor.");

    while (true);  // Halt — force re-upload with new constant
}

// =============================================================================
// SECTION 13 — USER WEIGHT CALIBRATION
// =============================================================================
// Called during startup (not during the demo).
// User stands still on the plate; we average 50 samples to get body weight.

float captureUserWeight() {
    Serial.println();
    Serial.println("=== USER WEIGHT CALIBRATION ===");
    Serial.println("Stand STILL on the plate with full body weight.");
    Serial.println("Press 'y' then Enter when you are steady.");
    waitForChar('y');

    delay(500);  // Let the user settle

    Serial.println("Measuring weight (50 samples)...");
    float sum = 0.0f;
    for (int i = 0; i < 50; i++) {
        if (loadCell.is_ready()) {
            sum += loadCell.get_units(1);
        } else {
            i--;  // Retry this sample
        }
        delay(20);
    }
    float weight = sum / 50.0f;

    Serial.print("User weight recorded: ");
    Serial.print(weight, 1);
    Serial.println(" grams");

    return weight;
}

// =============================================================================
// SECTION 14 — IMU BASELINE CALIBRATION
// =============================================================================
// Capture the standing-still angle so we know what "straight" looks like.

float captureBaselineAngle() {
    Serial.println();
    Serial.println("=== IMU BASELINE CALIBRATION ===");
    Serial.println("Stand STILL with IMU strapped to calf — leg straight.");
    Serial.println("Press 'y' then Enter when you are steady.");
    waitForChar('y');

    delay(500);

    float sum = 0.0f;
    int samples = 50;
    for (int i = 0; i < samples; i++) {
        if (myICM.dataReady()) {
            myICM.getAGMT();
            float pitch = atan2(myICM.accX(), myICM.accZ()) * 57.2958f;
            sum += pitch;
        } else {
            i--;
        }
        delay(20);
    }
    float baseline = sum / samples;

    Serial.print("IMU baseline angle: ");
    Serial.print(baseline, 2);
    Serial.println(" degrees");

    return baseline;
}

// =============================================================================
// SECTION 15 — STARTUP MENU
// =============================================================================

void startupMenu() {
    Serial.println();
    Serial.println("╔═══════════════════════════════════════════╗");
    Serial.println("║     BIONIC LEG PROTOTYPE — STARTUP        ║");
    Serial.println("╠═══════════════════════════════════════════╣");
    Serial.println("║  Type 'c' → Load Cell Calibration (once)  ║");
    Serial.println("║  Type 'w' → Live Demo Mode                ║");
    Serial.println("╚═══════════════════════════════════════════╝");

    while (!Serial.available());
    char choice = Serial.read();
    flushSerialInput();

    if (choice == 'c') {
        // --- CALIBRATION PATH ---
        // (load cell is already initialised in setup with the current constant)
        runLoadCellCalibration();
        // does not return
    }

    // --- DEMO PATH ---
    Serial.println();
    Serial.println("=== LIVE DEMO MODE ===");

    // Capture user weight (with IMU and load cell already up)
    userWeightGrams = captureUserWeight();

    // Capture IMU baseline angle
    baselineAngle  = captureBaselineAngle();
    filteredAngle  = baselineAngle;
    lastImuTime    = millis();

    Serial.println();
    Serial.println("All baselines captured.");
    Serial.println("Press Enter to START live data plotting and phase detection...");

    // Wait for Enter (newline or carriage return)
    while (true) {
        if (Serial.available()) {
            char c = Serial.read();
            if (c == '\n' || c == '\r') { flushSerialInput(); break; }
        }
    }

    Serial.println("Starting demo — walk on and off the plate!");
    Serial.println();
    delay(500);
}

// =============================================================================
// SECTION 16 — HARDWARE SETUP
// =============================================================================

void setup() {
    Serial.begin(9600);
    while (!Serial && millis() < 3000);  // Wait for Serial Monitor (USB boards)

    // --- Motor driver pins ---
    pinMode(STEP_N_PIN, OUTPUT);
    pinMode(DIR_N_PIN,  OUTPUT);
    pinMode(EN_N_PIN,   OUTPUT);
    digitalWriteFast(STEP_N_PIN, DM_OFF);
    digitalWriteFast(DIR_N_PIN,  DM_OFF);
    enableDriver(false);  // Driver OFF until needed

    // --- I2C + IMU ---
    WIRE_PORT.begin();
    WIRE_PORT.setClock(400000);  // 400 kHz fast mode

    Serial.println("Initialising IMU...");
    while (myICM.begin(WIRE_PORT, AD0_VAL) != ICM_20948_Stat_Ok) {
        Serial.println("  ICM-20948 not found — retrying...");
        delay(500);
    }
    Serial.println("IMU OK.");

    // --- Load Cell ---
    Serial.println("Initialising load cell...");
    loadCell.begin(HX711_DATA_PIN, HX711_CLOCK_PIN);
    loadCell.set_scale(LOAD_CELL_SCALE);  // ← paste your calibrated value here
    loadCell.tare();                       // Zero with empty plate
    Serial.println("Load cell OK.");

    // --- Interactive startup menu ---
    startupMenu();

    // Enable driver now that we are ready to run
    enableDriver(true);
}

// =============================================================================
// SECTION 17 — MAIN LOOP
// =============================================================================
// IMU and load cell are sampled at DIFFERENT times to avoid blocking:
//   • IMU  → checked every loop iteration (fast; non-blocking dataReady check)
//   • HX711→ checked only when is_ready() returns true (~10 Hz by default)
// This keeps the angle filter smooth while the weight filter is slower.

void loop() {
    bool gotNewData = false;

    // -------------------------------------------------------------------------
    // IMU UPDATE (fast — runs whenever dataReady())
    // -------------------------------------------------------------------------
    if (myICM.dataReady()) {
        myICM.getAGMT();

        unsigned long now = millis();
        float dt = (now - lastImuTime) / 1000.0f;
        lastImuTime = now;

        // Accelerometer-derived pitch (degrees)
        float accPitch = atan2(myICM.accX(), myICM.accZ()) * 57.2958f;

        // Smooth raw gyro Y
        currentGyroY = gyroFilter.process(myICM.gyrY());

        // Complementary filter: 96% gyro integration + 4% accelerometer
        filteredAngle = 0.96f * (filteredAngle + currentGyroY * dt)
                      + 0.04f * accPitch;

        gotNewData = true;
    }

    // -------------------------------------------------------------------------
    // LOAD CELL UPDATE (slow — ~10 Hz, non-blocking)
    // -------------------------------------------------------------------------
    if (loadCell.is_ready()) {
        float raw = loadCell.get_units(1);
        smoothWeight = weightFilter.process(raw);
        gotNewData = true;
    }

    // -------------------------------------------------------------------------
    // PHASE DETECTION + MOTOR CONTROL + TELEMETRY
    // (only when at least one sensor has new data)
    // -------------------------------------------------------------------------
    if (gotNewData) {
        // 1. Update gait phase based on latest weight and angle
        detectPhase(smoothWeight, filteredAngle);

        // 2. Command motor based on the detected phase
        applyMotorForPhase(currentPhase);

        // 3. Read back motor state for telemetry
        long curStep, tgtStep;
        noInterrupts();
        curStep = stepPosition;
        tgtStep = stepTarget;
        interrupts();

        // 4. Serial Plotter telemetry (tab-separated, labelled)
        //    Open Tools → Serial Plotter to see live waveforms.
        //    Phase is scaled by 10 so it is visible alongside other signals.
        Serial.print("Weight:");      Serial.print(smoothWeight, 1);
        Serial.print("\tPhase:");     Serial.print((int)currentPhase * 100);
        Serial.print("\tAngle:");     Serial.print(filteredAngle - baselineAngle, 2);
        Serial.print("\tSteps:");     Serial.print(curStep);
        Serial.print("\tTarget:");    Serial.println(tgtStep);
    } else {
        delay(2);   // Tiny yield so we don't spin-wait the CPU
    }
}
// =============================================================================
// END OF FILE
// =============================================================================
