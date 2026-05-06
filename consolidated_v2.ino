// =============================================================================
// BIONIC LEG PROTOTYPE — WALKING GAIT PHASE CONTROLLER
// Final merged version: Abishai (calibration/sensors) + Ben (loop timing) +
// Brennon (motor phases) — integrated and verified.
// =============================================================================
//
// ┌─────────────────────────────────────────────────────────────────────────┐
// │ HARDWARE                                                                │
// │  • ICM-20948 IMU   — strapped to calf, I2C via Wire                   │
// │  • HX711 + 4x load cells — foot plate, DATA=pin2 CLK=pin3             │
// │  • Stepper motor driver  — STEP=pin5, DIR=pin6, EN=pin7               │
// │  • Teensy 4.x (required for IntervalTimer + digitalWriteFast)          │
// └─────────────────────────────────────────────────────────────────────────┘
//
// ┌─────────────────────────────────────────────────────────────────────────┐
// │ HOW TO USE — READ THIS FIRST                                           │
// │                                                                         │
// │  FIRST TIME ONLY — Find your load cell scale factor:                   │
// │   1. Upload code. Open Serial Monitor (115200 baud, "Both NL & CR").   │
// │   2. Type  c  and press Enter.                                          │
// │   3. Follow the on-screen steps. Copy the printed scale factor.        │
// │   4. Paste it into LOAD_CELL_SCALE below. Re-upload.                   │
// │                                                                         │
// │  EVERY RUN — Live demo:                                                 │
// │   1. Open Serial Monitor (115200 baud, "Both NL & CR").                │
// │   2. Type  w  and press Enter.                                          │
// │   3. Stand on plate with full weight → press  y  → Enter.              │
// │   4. Stand straight with IMU on calf → press  y  → Enter.              │
// │   5. Press Enter to start live data + phase detection.                 │
// │   6. Step on/off the plate to cycle through gait phases.               │
// │                                                                         │
// │  SERIAL PLOTTER (live graphs):                                         │
// │   Tools → Serial Plotter. Uses same baud (115200).                     │
// │   NOTE: Close Serial Monitor first — only one can be open at a time.   │
// │   Signals: Weight | Phase (×100 so visible) | Angle | Steps | Target   │
// │                                                                         │
// │  SERIAL MONITOR (phase text + debug):                                  │
// │   Use this during calibration and to read phase-change banners.        │
// └─────────────────────────────────────────────────────────────────────────┘
//
// ┌─────────────────────────────────────────────────────────────────────────┐
// │ GAIT CYCLE — strict one-way order, no phase skipping                   │
// │                                                                         │
// │  TERMINAL_SWING → INITIAL_CONTACT → LOADING_RESPONSE → MID_STANCE     │
// │  → TERMINAL_STANCE → PRE_SWING → INITIAL_SWING → MID_SWING            │
// │  → TERMINAL_SWING (loops)                                              │
// │                                                                         │
// │ MOTOR BEHAVIOUR:                                                        │
// │  INITIAL_CONTACT / LOADING_RESPONSE / MID_STANCE / TERMINAL_STANCE    │
// │    → Driver OFF, knee held passive at 0° (stance = straight leg)       │
// │  PRE_SWING   → Driver ON, flex to 20° at 300 SPS                       │
// │  INITIAL_SWING → Flex to 60° at 500 SPS (peak bend, foot clearance)   │
// │  MID_SWING   → Reverse direction, extend back to 30° at 450 SPS        │
// │  TERMINAL_SWING → Extend to 0° at 300 SPS (straight for heel-strike)  │
// └─────────────────────────────────────────────────────────────────────────┘

#include <Arduino.h>
#include <IntervalTimer.h>
#include "ICM_20948.h"    // SparkFun ICM-20948 IMU library
#include "HX711.h"        // HX711 load cell amplifier library

// =============================================================================
// SECTION 1 — PIN DEFINITIONS
// =============================================================================

constexpr uint8_t STEP_N_PIN      = 5;   // Stepper: step pulse
constexpr uint8_t DIR_N_PIN       = 6;   // Stepper: direction
constexpr uint8_t EN_N_PIN        = 7;   // Stepper: enable (active LOW)
constexpr uint8_t HX711_DATA_PIN  = 2;   // HX711: DT
constexpr uint8_t HX711_CLOCK_PIN = 3;   // HX711: SCK

// Most stepper drivers: EN LOW = ON, EN HIGH = OFF
constexpr uint8_t DM_ON  = LOW;
constexpr uint8_t DM_OFF = HIGH;

// Minimum step-pulse HIGH width. DRV8825 needs ≥1.9 µs; 5 µs is safe.
constexpr uint32_t STEP_PULSE_US = 5;

// =============================================================================
// SECTION 2 — MECHANICAL LIMITS
// =============================================================================
// *** UPDATE THESE after Friday's mechanical test ***
// Measure how many steps move the knee from fully straight to fully bent.
// Current values are conservative placeholders — do NOT increase until verified.

constexpr float STEPS_PER_DEG  = 10.0f;  // motor steps per degree of knee angle
constexpr float MAX_KNEE_DEG   = 70.0f;  // absolute mechanical stop — never exceed
constexpr float MIN_KNEE_DEG   = 0.0f;   // fully straight
constexpr long  MIN_STEPS      = 0;
constexpr long  MAX_SAFE_STEPS = (long)(MAX_KNEE_DEG * STEPS_PER_DEG); // = 700

// Speed cap — raise only after team confirms no mechanical issues
constexpr float MAX_SAFE_SPS = 600.0f;   // steps per second

// =============================================================================
// SECTION 3 — LOAD CELL CALIBRATION CONSTANT
// =============================================================================
// *** PASTE YOUR SCALE FACTOR HERE after running the 'c' calibration menu ***
// The value below is a placeholder from previous testing. Yours will differ.
constexpr float LOAD_CELL_SCALE = -12.48f;

// =============================================================================
// SECTION 4 — IMU & I2C
// =============================================================================

#define WIRE_PORT Wire
#define AD0_VAL   1      // AD0 pin HIGH → I2C address 0x69

ICM_20948_I2C myICM;

// Complementary filter: blends gyro integration (fast) with accel pitch (slow drift correction)
// 0.96 = trust gyro 96%, correct with accel 4% each sample
float filteredAngle = 0.0f;
float baselineAngle = 0.0f;   // "straight leg standing still" reference angle
unsigned long lastImuTime = 0;

// =============================================================================
// SECTION 5 — LOAD CELL
// =============================================================================

HX711 loadCell;

// Populated during 'w' startup — never relies on a hard-coded default at runtime
float userWeightGrams = 0.0f;

// =============================================================================
// SECTION 6 — MOVING-AVERAGE FILTERS
// =============================================================================
// Smooths noisy sensor readings without introducing excessive lag.
// Weight: 4-sample window (~slow HX711 rate, so 4 samples ≈ 400 ms smoothing).
// Gyro:   2-sample window (very fast IMU, minimal smoothing needed).

class MovingAverageFilter {
private:
    float* buf;
    int    windowSize;
    int    idx = 0;
    float  sum = 0.0f;
public:
    MovingAverageFilter(int size) : windowSize(size) {
        buf = new float[size]();  // zero-initialised
    }
    float process(float v) {
        sum     -= buf[idx];
        buf[idx]  = v;
        sum      += v;
        idx       = (idx + 1) % windowSize;
        return sum / windowSize;
    }
};

MovingAverageFilter weightFilter(4);  // load cell smoothing
MovingAverageFilter gyroFilter(2);    // gyro Y smoothing

// Latest smoothed values — updated in loop(), used by detectPhase()
float smoothWeight = 0.0f;
float currentGyroY = 0.0f;

// =============================================================================
// SECTION 7 — GAIT PHASE STATE MACHINE
// =============================================================================

enum GaitPhase {
    INITIAL_CONTACT,    // 0 — heel touches plate
    LOADING_RESPONSE,   // 1 — weight loading rapidly
    MID_STANCE,         // 2 — full body weight on foot
    TERMINAL_STANCE,    // 3 — weight transferring to forefoot
    PRE_SWING,          // 4 — foot almost off, knee begins to flex
    INITIAL_SWING,      // 5 — foot lifts, knee bends to peak
    MID_SWING,          // 6 — peak flex, leg swings forward
    TERMINAL_SWING      // 7 — knee extends, leg straightens for heel-strike
};

// Start with foot in the air (TERMINAL_SWING) so the first step on the
// plate correctly triggers INITIAL_CONTACT
GaitPhase currentPhase  = TERMINAL_SWING;
GaitPhase previousPhase = TERMINAL_SWING;

// =============================================================================
// SECTION 8 — STEPPER MOTOR — INTERRUPT-DRIVEN DRIVER
// =============================================================================
// The IntervalTimer fires the ISR at a rate of (desired SPS) Hz.
// Each ISR call generates exactly one complete step pulse (HIGH then LOW).
// Position is tracked in steps so we can stop at exact targets.

IntervalTimer stepTimer;

volatile long stepPosition   = 0;     // current absolute position in steps
volatile long stepTarget     = 0;     // desired target position in steps
volatile bool stepDirForward = true;  // true = flex (bend), false = extend (straighten)
volatile bool steppingActive = false; // ISR does nothing when false

void stepISR() {
    if (!steppingActive) return;

    // Arrived at target — stop automatically
    if (stepPosition == stepTarget) {
        steppingActive = false;
        digitalWriteFast(STEP_N_PIN, DM_OFF);
        return;
    }

    // Pulse STEP pin
    digitalWriteFast(STEP_N_PIN, DM_ON);
    delayMicroseconds(STEP_PULSE_US);
    digitalWriteFast(STEP_N_PIN, DM_OFF);

    // Update position tracker
    if (stepDirForward) stepPosition++;
    else                stepPosition--;
}

// =============================================================================
// SECTION 9 — MOTOR CONTROL HELPERS
// =============================================================================

void enableDriver(bool on) {
    digitalWriteFast(EN_N_PIN, on ? DM_ON : DM_OFF);
}

// Call setDirection() BEFORE setSpeedSPS() / moveToPosition()
// Direction pin must settle before the first step pulse.
void setDirection(bool flexKnee) {
    noInterrupts();
    stepDirForward = flexKnee;
    interrupts();
    digitalWriteFast(DIR_N_PIN, flexKnee ? DM_ON : DM_OFF);
    delayMicroseconds(5);  // direction settle time per DRV8825 datasheet
}

// Start (or restart) the timer at the desired speed
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
    uint32_t period_us = (uint32_t)(1e6f / sps);
    stepTimer.begin(stepISR, period_us);
}

// Move to an absolute step position at a given speed.
// Safe: clamps position and speed to their declared limits.
void moveToPosition(long targetPos, float sps) {
    targetPos = constrain(targetPos, MIN_STEPS, MAX_SAFE_STEPS);
    sps       = constrain(sps, 0.0f, MAX_SAFE_SPS);

    noInterrupts();
    long current = stepPosition;
    interrupts();

    if (current == targetPos) return;  // already there — nothing to do

    bool flex = (targetPos > current); // moving toward more bend?
    setDirection(flex);

    noInterrupts();
    stepTarget     = targetPos;
    steppingActive = true;
    interrupts();

    setSpeedSPS(sps);
}

// Convenience: move to a knee angle in degrees
void moveToAngleDeg(float deg, float sps) {
    deg = constrain(deg, MIN_KNEE_DEG, MAX_KNEE_DEG);
    moveToPosition((long)(deg * STEPS_PER_DEG), sps);
}

// =============================================================================
// SECTION 10 — PER-PHASE MOTOR BEHAVIOUR
// =============================================================================
// Called every loop cycle with the current gait phase.
// Only changes motor state on phase TRANSITIONS — the position-tracking ISR
// handles the rest autonomously between calls.

void applyMotorForPhase(GaitPhase phase) {
    switch (phase) {

        // ── STANCE (4 phases) ─────────────────────────────────────────────
        // Leg is under body weight. Keep knee at 0° (straight).
        // Disable the driver so it doesn't fight gravity or heat up.
        // The motor holds position passively via mechanical friction / brake.
        case INITIAL_CONTACT:
        case LOADING_RESPONSE:
        case MID_STANCE:
        case TERMINAL_STANCE:
            moveToAngleDeg(0.0f, 200.0f);  // return to straight if not already
            enableDriver(false);            // passive hold — driver off
            break;

        // ── PRE_SWING ─────────────────────────────────────────────────────
        // Foot is lifting. Begin knee flex to clear the floor.
        case PRE_SWING:
            enableDriver(true);
            moveToAngleDeg(20.0f, 300.0f); // gentle early bend (20°)
            break;

        // ── INITIAL_SWING ─────────────────────────────────────────────────
        // Foot fully off ground. Drive to peak flex for foot clearance.
        case INITIAL_SWING:
            enableDriver(true);
            setDirection(true);             // flex direction
            moveToAngleDeg(60.0f, 500.0f); // peak bend (60°)
            break;

        // ── MID_SWING ─────────────────────────────────────────────────────
        // Leg swinging forward past peak. Reverse motor to begin extending.
        case MID_SWING:
            enableDriver(true);
            setDirection(false);            // extend direction
            moveToAngleDeg(30.0f, 450.0f); // begin straightening (60° → 30°)
            break;

        // ── TERMINAL_SWING ────────────────────────────────────────────────
        // Leg almost straight for heel-strike. Finish extending to 0°.
        case TERMINAL_SWING:
            enableDriver(true);
            setDirection(false);            // extend direction
            moveToAngleDeg(0.0f, 300.0f);  // fully straight
            break;
    }
}

// =============================================================================
// SECTION 11 — PHASE DETECTION
// =============================================================================
// Strict sequential state machine — can only advance ONE phase at a time.
// Thresholds expressed as fractions of measured userWeightGrams.
//
// WEIGHT THRESHOLDS (adjust if demo triggers too early/late):
//   >10% body weight  → initial contact confirmed
//   >30% body weight  → loading response (weight rising fast)
//   >60% body weight  → mid-stance (full weight over foot)
//   <60% body weight  → terminal stance (weight shifting off)
//   <5%  body weight  → foot lifted (swing begins)
//
// ANGLE THRESHOLDS (relative to baseline standing angle):
//   |relAngle| > 10° → leg has swung → initial swing
//   |relAngle| > 20° → peak swing    → mid swing
//   |relAngle| < 10° → leg returning → terminal swing
//
// If you see phases flickering at a boundary, increase the gap between
// the entry and exit thresholds (hysteresis). E.g. enter MID_STANCE at >60%,
// exit at <55%.

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

    // Pre-compute booleans for readability
    float relAngle = angle - baselineAngle;

    bool weightBearing = weight > (userWeightGrams * 0.10f);  // >10% BW
    bool loaded30      = weight > (userWeightGrams * 0.30f);  // >30% BW
    bool loaded60      = weight > (userWeightGrams * 0.60f);  // >60% BW
    bool unloaded      = weight < (userWeightGrams * 0.05f);  // <5%  BW (foot lifted)

    // Each case ONLY checks the condition to move to the NEXT phase.
    // It is impossible to skip a phase because only the current phase's case runs.
    switch (currentPhase) {

        case TERMINAL_SWING:
            // Waiting for heel-strike — any weight triggers contact
            if (weightBearing) currentPhase = INITIAL_CONTACT;
            break;

        case INITIAL_CONTACT:
            // Weight increasing past 30% → body loading onto foot
            if (loaded30) currentPhase = LOADING_RESPONSE;
            break;

        case LOADING_RESPONSE:
            // Full body weight (>60%) → mid-stance
            if (loaded60) currentPhase = MID_STANCE;
            break;

        case MID_STANCE:
            // Weight dropping below 60% → body shifting to forefoot
            if (!loaded60) currentPhase = TERMINAL_STANCE;
            break;

        case TERMINAL_STANCE:
            // Foot fully lifts off (<5% BW) → enter swing
            if (unloaded) currentPhase = PRE_SWING;
            break;

        case PRE_SWING:
            // Shank has tilted >10° from baseline → leg is in swing
            if (fabsf(relAngle) > 10.0f) currentPhase = INITIAL_SWING;
            break;

        case INITIAL_SWING:
            // Shank reaches >20° tilt → peak of swing arc
            if (fabsf(relAngle) > 20.0f) currentPhase = MID_SWING;
            break;

        case MID_SWING:
            // Shank returning within 10° of baseline → leg straightening
            if (fabsf(relAngle) < 10.0f) currentPhase = TERMINAL_SWING;
            break;
    }

    // ── Print a clearly visible banner on every phase transition ──────────
    if (currentPhase != previousPhase) {
        Serial.println();
        Serial.println(F("╔══════════════════════════════════════╗"));
        Serial.print  (F("  PHASE: "));
        Serial.println(phaseName(currentPhase));
        Serial.println(F("╚══════════════════════════════════════╝"));
        previousPhase = currentPhase;
    }
}

// =============================================================================
// SECTION 12 — SERIAL UTILITY FUNCTIONS
// =============================================================================

// Throw away any stray bytes sitting in the receive buffer
void flushSerialInput() {
    while (Serial.available()) {
        Serial.read();
        delay(1);
    }
}

// Block until the user sends the exact character  c  (case-insensitive)
void waitForChar(char expected) {
    while (true) {
        if (Serial.available()) {
            char rx = (char)Serial.read();
            // Accept lower or upper case
            if (rx == expected || rx == (expected - 32) || rx == (expected + 32)) {
                flushSerialInput();
                return;
            }
        }
    }
}

// =============================================================================
// SECTION 13 — LOAD CELL SCALE-FACTOR CALIBRATION
// =============================================================================
// Run this ONCE. Paste the printed number into LOAD_CELL_SCALE above.
// After re-uploading, use the 'w' path for every subsequent run.

void runLoadCellCalibration() {
    Serial.println();
    Serial.println(F("╔══════════════════════════════════════╗"));
    Serial.println(F("       LOAD CELL CALIBRATION            "));
    Serial.println(F("╚══════════════════════════════════════╝"));
    Serial.println();

    // Step 1: tare with empty plate
    Serial.println(F("STEP 1: Make sure the plate is completely EMPTY."));
    Serial.println(F("        Press 'y' then Enter when ready to tare."));
    waitForChar('y');
    loadCell.tare();
    Serial.println(F("  >> Tare complete. Plate is now zeroed."));
    Serial.println();

    // Step 2: place known weight
    Serial.println(F("STEP 2: Place a KNOWN calibration weight on the plate."));
    Serial.println(F("        (e.g. a 500 g dumbbell plate, or a known object)"));
    Serial.println(F("        Press 'y' then Enter when the reading is stable."));
    waitForChar('y');

    Serial.println(F("  Reading 20 samples — do not touch the plate..."));
    long rawAvg = loadCell.read_average(20);
    Serial.print(F("  >> Raw average = ")); Serial.println(rawAvg);
    Serial.println();

    // Step 3: user enters the known weight in grams
    Serial.println(F("STEP 3: Type the known weight in GRAMS and press Enter."));
    Serial.println(F("        Example: type  500  for a 500 g weight."));
    while (!Serial.available());
    float knownGrams = Serial.parseFloat();
    flushSerialInput();
    Serial.print(F("  You entered: ")); Serial.print(knownGrams, 1); Serial.println(F(" g"));
    Serial.println();

    float scaleFactor = (float)rawAvg / knownGrams;

    Serial.println(F("╔══════════════════════════════════════════════════════╗"));
    Serial.println(F("  CALIBRATION RESULT:"));
    Serial.print  (F("    Scale factor = ")); Serial.println(scaleFactor, 6);
    Serial.println(F(""));
    Serial.println(F("  NEXT STEPS:"));
    Serial.println(F("    1. Copy the scale factor number above."));
    Serial.println(F("    2. Paste it into LOAD_CELL_SCALE at the top of the file."));
    Serial.println(F("    3. Re-upload the code."));
    Serial.println(F("    4. Run the 'w' demo mode — no need to calibrate again."));
    Serial.println(F("╚══════════════════════════════════════════════════════╝"));
    Serial.println();
    Serial.println(F("  SYSTEM HALTED. Re-upload with the new scale factor."));

    while (true) { /* halt — force re-upload */ }
}

// =============================================================================
// SECTION 14 — USER WEIGHT CAPTURE
// =============================================================================
// User stands still on plate. Average 50 samples for a stable body-weight reading.
// This stored value drives all phase-detection thresholds at runtime.

float captureUserWeight() {
    Serial.println();
    Serial.println(F("╔══════════════════════════════════════╗"));
    Serial.println(F("       USER WEIGHT CALIBRATION          "));
    Serial.println(F("╚══════════════════════════════════════╝"));
    Serial.println(F("  Stand on the plate with your FULL body weight."));
    Serial.println(F("  Hold still. Press 'y' then Enter when steady."));
    waitForChar('y');

    delay(500);  // let the reading settle after they press Enter

    Serial.println(F("  Sampling weight (50 readings)..."));
    float total = 0.0f;
    int   count = 0;
    while (count < 50) {
        if (loadCell.is_ready()) {
            float sample = loadCell.get_units(1);
            total += sample;
            count++;
            // Print a progress dot every 10 samples so user knows it's working
            if (count % 10 == 0) {
                Serial.print(F("  ..."));
                Serial.print(count);
                Serial.println(F(" samples done"));
            }
            delay(10);
        }
    }

    float weight = total / 50.0f;
    Serial.println();
    Serial.print(F("  >> User weight recorded: "));
    Serial.print(weight, 1);
    Serial.println(F(" grams"));

    // Sanity check — warn if the value looks wrong
    if (weight < 500.0f) {
        Serial.println(F("  [WARNING] Weight seems very low (<500 g)."));
        Serial.println(F("            Check that LOAD_CELL_SCALE is set correctly."));
    }
    if (weight > 200000.0f) {
        Serial.println(F("  [WARNING] Weight seems very high (>200 kg)."));
        Serial.println(F("            Check that LOAD_CELL_SCALE is set correctly."));
    }

    return weight;
}

// =============================================================================
// SECTION 15 — IMU BASELINE ANGLE CAPTURE
// =============================================================================
// User stands straight with IMU strapped to calf.
// Average 50 accel-pitch samples to get the "leg straight" reference angle.

float captureBaselineAngle() {
    Serial.println();
    Serial.println(F("╔══════════════════════════════════════╗"));
    Serial.println(F("       IMU BASELINE CALIBRATION         "));
    Serial.println(F("╚══════════════════════════════════════╝"));
    Serial.println(F("  Strap IMU to calf. Stand with leg STRAIGHT."));
    Serial.println(F("  Hold perfectly still. Press 'y' then Enter when ready."));
    waitForChar('y');

    delay(500);

    Serial.println(F("  Sampling IMU angle (50 readings)..."));
    float total  = 0.0f;
    int   count  = 0;
    while (count < 50) {
        if (myICM.dataReady()) {
            myICM.getAGMT();
            float pitch = atan2f(myICM.accX(), myICM.accZ()) * 57.2958f;
            total += pitch;
            count++;
            delay(20);
        }
    }
    float baseline = total / 50.0f;

    Serial.print(F("  >> IMU baseline angle: "));
    Serial.print(baseline, 2);
    Serial.println(F(" degrees"));
    Serial.println(F("  (This is your 'leg straight standing still' reference.)"));

    return baseline;
}

// =============================================================================
// SECTION 16 — STARTUP MENU
// =============================================================================

void startupMenu() {
    Serial.println();
    Serial.println(F("╔══════════════════════════════════════════════════════╗"));
    Serial.println(F("         BIONIC LEG PROTOTYPE — STARTUP MENU           "));
    Serial.println(F("╠══════════════════════════════════════════════════════╣"));
    Serial.println(F("   Type  c  + Enter  →  Load Cell Calibration (once)   "));
    Serial.println(F("   Type  w  + Enter  →  Live Demo Mode                 "));
    Serial.println(F("╚══════════════════════════════════════════════════════╝"));

    // Wait for a valid choice
    while (true) {
        if (Serial.available()) {
            char choice = (char)Serial.read();
            flushSerialInput();

            if (choice == 'c' || choice == 'C') {
                runLoadCellCalibration();
                // does not return
            }
            if (choice == 'w' || choice == 'W') {
                break;  // proceed to demo setup below
            }
            // Any other character — re-prompt
            Serial.println(F("  Please type  c  or  w  then press Enter."));
        }
    }

    // ── DEMO SETUP ─────────────────────────────────────────────────────────
    Serial.println();
    Serial.println(F("  Starting demo setup..."));

    userWeightGrams = captureUserWeight();
    baselineAngle   = captureBaselineAngle();

    // Initialise the complementary filter at the baseline so it doesn't
    // take many seconds to converge from 0
    filteredAngle = baselineAngle;
    lastImuTime   = millis();

    // Tare the load cell again now that the user has stepped off
    // (baseline weight was captured while standing — now they step off briefly)
    Serial.println();
    Serial.println(F("  Step OFF the plate briefly to re-tare it, then step back on."));
    Serial.println(F("  Press 'y' then Enter when the plate is empty and still."));
    waitForChar('y');
    loadCell.tare();
    Serial.println(F("  >> Load cell re-tared (zeroed). You can step on now."));

    Serial.println();
    Serial.println(F("  ── All baselines captured ──────────────────────────────"));
    Serial.print  (F("     User weight  : ")); Serial.print(userWeightGrams, 1); Serial.println(F(" g"));
    Serial.print  (F("     Baseline angle: ")); Serial.print(baselineAngle, 2); Serial.println(F(" deg"));
    Serial.println(F("  ────────────────────────────────────────────────────────"));
    Serial.println();
    Serial.println(F("  Press Enter to START live data + phase detection..."));
    Serial.println(F("  (Switch to Serial Plotter for live graphs after pressing Enter)"));

    // Wait for Enter key
    while (true) {
        if (Serial.available()) {
            char c = (char)Serial.read();
            if (c == '\n' || c == '\r') {
                flushSerialInput();
                break;
            }
        }
    }

    Serial.println(F("  >> LIVE DEMO STARTED — step on and off the plate!"));
    Serial.println();
    delay(300);

    // Print Serial Plotter header — these labels appear as the legend in the plotter.
    // Must be printed ONCE, before the first data line.
    // Format: label1:value TAB label2:value ... newline
    Serial.println(F("Weight\tPhase_x100\tRelAngle\tSteps\tStepTarget"));
}

// =============================================================================
// SECTION 17 — HARDWARE SETUP (runs once at power-on)
// =============================================================================

void setup() {
    Serial.begin(115200);
    while (!Serial && millis() < 3000);  // wait up to 3 s for USB Serial on Teensy

    // ── Motor driver pins ───────────────────────────────────────────────────
    pinMode(STEP_N_PIN, OUTPUT);
    pinMode(DIR_N_PIN,  OUTPUT);
    pinMode(EN_N_PIN,   OUTPUT);
    digitalWriteFast(STEP_N_PIN, DM_OFF);  // step idle LOW
    digitalWriteFast(DIR_N_PIN,  DM_OFF);
    enableDriver(false);  // driver OFF at boot — enabled per phase in applyMotorForPhase()
    Serial.println(F("[SETUP] Motor driver pins configured."));

    // ── IMU ─────────────────────────────────────────────────────────────────
    WIRE_PORT.begin();
    WIRE_PORT.setClock(400000);  // 400 kHz I2C
    Serial.println(F("[SETUP] Initialising IMU..."));
    uint8_t imuTries = 0;
    while (myICM.begin(WIRE_PORT, AD0_VAL) != ICM_20948_Stat_Ok) {
        imuTries++;
        Serial.print(F("  ICM-20948 not found (try "));
        Serial.print(imuTries);
        Serial.println(F("). Check wiring. Retrying..."));
        delay(500);
        if (imuTries > 20) {
            Serial.println(F("  [FATAL] IMU not found after 20 tries. Check SDA/SCL and AD0."));
            while (true);  // halt
        }
    }
    Serial.println(F("[SETUP] IMU OK."));

    // ── Load cell ───────────────────────────────────────────────────────────
    Serial.println(F("[SETUP] Initialising load cell..."));
    loadCell.begin(HX711_DATA_PIN, HX711_CLOCK_PIN);
    loadCell.set_scale(LOAD_CELL_SCALE);  // ← your calibrated factor
    loadCell.tare();                       // zero with empty plate on startup
    Serial.println(F("[SETUP] Load cell OK (tared with empty plate)."));

    // ── Interactive startup ─────────────────────────────────────────────────
    startupMenu();  // blocks until user completes calibration + presses Enter

    // Enable motor driver now that we're ready to run
    enableDriver(true);
    Serial.println(F("[SETUP] Motor driver enabled. Starting main loop."));
}

// =============================================================================
// SECTION 18 — MAIN LOOP
// =============================================================================
// IMU and HX711 are sampled INDEPENDENTLY — non-blocking checks on every pass.
//
//   IMU updates   : every ~5–10 ms (ICM-20948 default ODR)
//   HX711 updates : every ~100 ms  (HX711 default ~10 Hz conversion rate)
//
// Phase detection and motor actuation run whenever EITHER sensor has new data,
// using whichever values are most recently available.
// This keeps the angle filter smooth at full IMU rate and doesn't stall waiting
// for the slower load cell.

void loop() {
    bool gotNewData = false;

    // ── IMU SAMPLE (fast, ~100–200 Hz) ───────────────────────────────────────
    if (myICM.dataReady()) {
        myICM.getAGMT();

        unsigned long now = millis();
        float dt = (now - lastImuTime) / 1000.0f;
        lastImuTime = now;

        // Accel-only pitch as a slow drift reference
        float accPitch = atan2f(myICM.accX(), myICM.accZ()) * 57.2958f;

        // Smooth gyro before integrating to reduce high-frequency noise
        currentGyroY = gyroFilter.process(myICM.gyrY());

        // Complementary filter — see SECTION 4 comment for rationale
        filteredAngle = 0.96f * (filteredAngle + currentGyroY * dt)
                      + 0.04f * accPitch;

        gotNewData = true;
    }

    // ── LOAD CELL SAMPLE (slow, ~10 Hz) ──────────────────────────────────────
    if (loadCell.is_ready()) {
        float raw = loadCell.get_units(1);
        smoothWeight = weightFilter.process(raw);
        gotNewData = true;
    }

    // ── PROCESS (runs at the rate of whichever sensor was just updated) ───────
    if (gotNewData) {

        // 1. Detect gait phase (may print a phase-change banner to Serial Monitor)
        detectPhase(smoothWeight, filteredAngle);

        // 2. Drive motor according to the current phase
        applyMotorForPhase(currentPhase);

        // 3. Snapshot motor state atomically for telemetry
        long curStep, tgtStep;
        noInterrupts();
        curStep = stepPosition;
        tgtStep = stepTarget;
        interrupts();

        // 4. Telemetry line — one line per update
        //
        // FORMAT FOR SERIAL PLOTTER:
        //   label:value<TAB>label:value<TAB>...<newline>
        //   The plotter reads the labels from the first line printed after opening.
        //   Values separated by TAB (\t) are plotted as separate traces.
        //
        // SIGNALS:
        //   Weight     — raw grams from load cell (0 when foot off, ~userWeightGrams when standing)
        //   Phase_x100 — gait phase index × 100  (0–700, steps of 100 for visibility)
        //   RelAngle   — filtered angle minus baseline (0 = straight leg, ±° when swinging)
        //   Steps      — actual motor step position (0 = straight, MAX_SAFE_STEPS = full bend)
        //   StepTarget — where motor is headed (matches Steps once move finishes)

        Serial.print(F("Weight:"));      Serial.print(smoothWeight, 1);
        Serial.print(F("\tPhase:"));     Serial.print((int)currentPhase * 100);
        Serial.print(F("\tRelAngle:")); Serial.print(filteredAngle - baselineAngle, 2);
        Serial.print(F("\tSteps:"));     Serial.print(curStep);
        Serial.print(F("\tTarget:"));    Serial.println(tgtStep);

    } else {
        delay(2);  // brief yield — prevents CPU spin while waiting for next sensor sample
    }
}

// =============================================================================
// END OF FILE
// =============================================================================
//
// THINGS TO UPDATE AFTER FRIDAY'S MECHANICAL TEST:
//   • STEPS_PER_DEG  — count actual steps from 0° to max bend, divide by degrees
//   • MAX_KNEE_DEG   — set to the real mechanical stop angle (do NOT exceed it)
//   • MAX_SAFE_SPS   — raise slowly only after confirming no mechanical stress
//   • LOAD_CELL_SCALE — confirm it gives correct grams (compare to a known weight)
//   • Phase angle thresholds in detectPhase() — tune to match your real IMU tilt range
// =============================================================================
