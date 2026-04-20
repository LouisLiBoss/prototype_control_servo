#include <Arduino.h>
#include <SPI.h>
#include <Servo.h>
#include <ICM42688.h>
#include <IWatchdog.h>

// ============================================================
//  CONFIGURATION MATERIELLE
//  Ajuster les pins selon le câblage.
// ============================================================
const uint8_t IMU_CS    = PA4;
const int     SERVO_PIN = PA15;
const int     LED_PIN   = PC13;  // LED embarquée — ajuster selon le PCB

ICM42688 imu(SPI, IMU_CS);
Servo finServo;

// ============================================================
//  MACHINE D'ETATS DE VOL
// ============================================================
enum FlightState : uint8_t {
    STATE_PAD,       // Sur le pas de tir
    STATE_BOOST,     // Moteur allumé
    STATE_COAST,     // Vol balistique (moteur éteint)
    STATE_DESCENT,   // Après apogée
    STATE_LANDED     // Au sol
};

FlightState flightState = STATE_PAD;

// ============================================================
//  GAINS PID — GAIN SCHEDULING
//
//  Convention d'unités :
//    Entrée PID  -> erreur roll rate (rad/s)
//    Sortie PID  -> déflexion aileron (degrés)
//
//    kp : deg/(rad/s)    ki : deg/rad    kd : deg/(rad/s²)
// ============================================================
struct PIDGains {
    float kp;
    float ki;
    float kd;
};

const PIDGains GAINS_BOOST = {0.0816f,   5.3373f, 0.0f};
const PIDGains GAINS_COAST = {0.016591f, 4.203f,  0.0f};

// Gains supplémentaires (à activer après tuning) :
// const PIDGains GAINS_HIGHSPEED = {1.3841e-07f, 3.4397f, 0.0f};
// const PIDGains GAINS_EXTREME  = {0.27429f,    1.0444f, 0.0f};
// const PIDGains GAINS_REENTRY  = {0.13683f,    0.49909f, 0.0f};

PIDGains currentGains = GAINS_BOOST;

// ============================================================
//  VARIABLES PID
// ============================================================
float setpoint   = 0.0f;  // Consigne roll rate (rad/s)
float rollRate   = 0.0f;  // Mesure  roll rate (rad/s)
float error      = 0.0f;  // rad/s
float lastError  = 0.0f;  // rad/s
float integral   = 0.0f;  // rad
float derivative = 0.0f;  // rad/s²
float output     = 0.0f;  // degrés de déflexion

// ============================================================
//  CONTRAINTES SERVO
// ============================================================
const float SERVO_CENTER   = 90.0f;  // Position neutre servo (degrés PWM)
const float DEFLECTION_MAX = 15.0f;  // Déflexion max +/- depuis le centre (degrés)
const float INTEGRAL_MAX   = 3.0f;   // Borne absolue sur |intégrale| (rad)

// ============================================================
//  TIMING
// ============================================================
const unsigned long LOOP_INTERVAL_US = 3333;  // ~300 Hz
const float dt = (float)LOOP_INTERVAL_US / 1000000.0f;

const int SERVO_DIVIDER       = 6;   // -> 50 Hz servo
const int SERIAL_DIVIDER      = 30;  // -> ~10 Hz debug (mode vol)
const int SERIAL_DIVIDER_TEST = 15;  // -> ~20 Hz debug (mode test)

// ============================================================
//  SEUILS DETECTION DE VOL
//
//  NOTE — Axe de poussée :
//  On suppose que l'axe X de l'IMU est aligné avec l'axe
//  longitudinal de la fusée (nez = +X), cohérent avec
//  gyrX() = roll.  Si le montage est différent, modifier
//  la ligne thrustAccel dans readIMU().
// ============================================================
const float LAUNCH_ACCEL_G       = 3.0f;   // Seuil lancement (g, sur magnitude)
const float BURNOUT_ACCEL_G      = 2.0f;   // Sous ce seuil -> fin de poussée (g, magnitude)
const float LANDED_DURATION      = 2.0f;   // Durée à ~1g pour déclarer atterri (s)
const float MIN_BOOST_TIME       = 0.5f;   // Durée min en BOOST avant transition (s)
const int   LAUNCH_CONFIRM_COUNT = 3;      // Échantillons consécutifs pour confirmer lancement

// Variables état de vol
float thrustAccel       = 0.0f;  // Accélération axiale (g)
float accelMag          = 0.0f;  // Magnitude accélération totale (g)
float estimatedVelocity = 0.0f;  // Estimation vitesse verticale (m/s)
float landedTimer       = 0.0f;
float boostTimer        = 0.0f;
int   launchConfirmCnt  = 0;

// ============================================================
//  MODE TEST
// ============================================================
bool    testMode    = false;
int     testGainSet = 0;      // 0 = boost, 1 = coast
char    cmdBuf[32];
uint8_t cmdIdx      = 0;

unsigned long lastTime;

// ============================================================
//  PROTOTYPES
// ============================================================
void readIMU();
void applyOutput(float out);
void updateFlightState();
void selectGains();
void resetPID();
void setTestMode(bool on);
void checkSerialCmd();
void executeCmd(const char* cmd);
void loopTest();
void servoSweep();
void printTestStatus();

// ============================================================
//  SETUP
// ============================================================
void setup() {
    Serial.begin(115200);
    pinMode(LED_PIN, OUTPUT);
    digitalWrite(LED_PIN, LOW);

    // --- SPI (config spécifique HGLRC F405) ---
    SPI.setSCLK(PA5);
    SPI.setMISO(PA6);
    SPI.setMOSI(PA7);
    SPI.begin();

    // --- IMU ---
    if (imu.begin() < 0) {
        Serial.println("ERREUR : ICM42688 introuvable !");
        finServo.attach(SERVO_PIN);
        finServo.write((int)SERVO_CENTER);
        while (1) {
            digitalWrite(LED_PIN, !digitalRead(LED_PIN));
            delay(200);
        }
    }

    Serial.println("Calibration Gyro... NE PAS BOUGER");
    imu.calibrateGyro();
    Serial.println("IMU prete.");

    // --- Servo ---
    finServo.attach(SERVO_PIN);
    finServo.write((int)SERVO_CENTER);

    // Auto-test : droite -> gauche -> centre
    Serial.println("Test servo...");
    finServo.write((int)(SERVO_CENTER + 20));
    delay(300);
    finServo.write((int)(SERVO_CENTER - 20));
    delay(300);
    finServo.write((int)SERVO_CENTER);
    delay(200);

    lastTime = micros();

    // --- Watchdog (4 secondes) ---
    IWatchdog.begin(4000000);

    // --- Détection mode test (3 secondes) ---
    Serial.println("Envoyer 't' dans 3s pour MODE TEST...");
    unsigned long waitStart = millis();
    while (millis() - waitStart < 3000) {
        IWatchdog.reload();
        if (Serial.available()) {
            char c = Serial.read();
            if (c == 't' || c == 'T') {
                setTestMode(true);
                break;
            }
        }
    }

    if (!testMode) {
        Serial.println("Mode vol. Attente lancement...");
    }
}

// ============================================================
//  BOUCLE PRINCIPALE — 300 Hz
// ============================================================
void loop() {
    unsigned long now = micros();
    if (now - lastTime < LOOP_INTERVAL_US) return;
    lastTime += LOOP_INTERVAL_US;

    IWatchdog.reload();

    // Toujours vérifier les commandes série (test ET vol)
    checkSerialCmd();

    // ========== MODE TEST ==========
    if (testMode) {
        loopTest();
        return;
    }

    // ========== MODE VOL ==========
    readIMU();
    updateFlightState();

    if (flightState == STATE_BOOST || flightState == STATE_COAST) {
        selectGains();

        error = setpoint - rollRate;

        // Dérivée filtrée (EMA alpha=0.2, tau ~17 ms a 300 Hz)
        float rawDerivative = (error - lastError) / dt;
        derivative = 0.2f * rawDerivative + 0.8f * derivative;
        lastError = error;

        // Sortie PID (degrés de déflexion)
        output = currentGains.kp * error
               + currentGains.ki * integral
               + currentGains.kd * derivative;

        // Anti-windup : n'accumule l'intégrale que si la sortie
        // courante n'est pas saturée dans la direction de l'erreur
        bool satHigh = (output >=  DEFLECTION_MAX) && (error > 0.0f);
        bool satLow  = (output <= -DEFLECTION_MAX) && (error < 0.0f);
        if (!satHigh && !satLow) {
            integral += error * dt;
            integral = constrain(integral, -INTEGRAL_MAX, INTEGRAL_MAX);
        }

        // Envoi servo à 50 Hz
        static int servoCnt = 0;
        if (++servoCnt >= SERVO_DIVIDER) {
            applyOutput(output);
            servoCnt = 0;
        }
    } else {
        // Hors contrôle actif — reset PID, servo au neutre
        error = lastError = integral = derivative = output = 0.0f;
        finServo.write((int)SERVO_CENTER);
    }

    // Debug série (~10 Hz)
    static int serialCnt = 0;
    if (++serialCnt >= SERIAL_DIVIDER) {
        Serial.print(">state:");    Serial.println((int)flightState);
        Serial.print(">roll:");     Serial.println(rollRate, 4);
        Serial.print(">output:");   Serial.println(output, 2);
        Serial.print(">integral:"); Serial.println(integral, 4);
        Serial.print(">accel:");    Serial.println(accelMag, 2);
        Serial.print(">vel:");      Serial.println(estimatedVelocity, 1);
        serialCnt = 0;
    }
}

// ============================================================
//  LECTURE IMU
// ============================================================
void readIMU() {
    imu.getAGT();

    rollRate = imu.gyrX() * (PI / 180.0f);  // deg/s -> rad/s

    // Accélération axiale (axe de poussée = X)
    // Modifier ici si l'IMU est montée différemment.
    thrustAccel = imu.accX();

    // Magnitude totale (indépendante de l'orientation, pour détection)
    float ax = imu.accX(), ay = imu.accY(), az = imu.accZ();
    accelMag = sqrtf(ax * ax + ay * ay + az * az);
}

// ============================================================
//  APPLIQUER LA SORTIE AU SERVO
//  Entrée : déflexion en degrés [-DEFLECTION_MAX, +DEFLECTION_MAX]
//  Sortie : angle servo [75, 105] centré sur 90
// ============================================================
void applyOutput(float out) {
    out = constrain(out, -DEFLECTION_MAX, DEFLECTION_MAX);
    finServo.write((int)(SERVO_CENTER + out));
}

// ============================================================
//  MISE A JOUR ETAT DE VOL
// ============================================================
void updateFlightState() {
    switch (flightState) {

    case STATE_PAD:
        if (accelMag > LAUNCH_ACCEL_G) {
            launchConfirmCnt++;
            if (launchConfirmCnt >= LAUNCH_CONFIRM_COUNT) {
                flightState = STATE_BOOST;
                estimatedVelocity = 0.0f;
                boostTimer = 0.0f;
                Serial.println(">>> LANCEMENT <<<");
            }
        } else {
            launchConfirmCnt = 0;
        }
        break;

    case STATE_BOOST:
        boostTimer += dt;
        estimatedVelocity += (thrustAccel - 1.0f) * 9.81f * dt;

        if (boostTimer > MIN_BOOST_TIME && accelMag < BURNOUT_ACCEL_G) {
            flightState = STATE_COAST;
            Serial.println(">>> COAST <<<");
        }
        break;

    case STATE_COAST:
        estimatedVelocity += (thrustAccel - 1.0f) * 9.81f * dt;

        if (estimatedVelocity <= 0.0f) {
            flightState = STATE_DESCENT;
            Serial.println(">>> DESCENTE <<<");
        }
        break;

    case STATE_DESCENT:
        if (fabsf(accelMag - 1.0f) < 0.15f) {
            landedTimer += dt;
            if (landedTimer >= LANDED_DURATION) {
                flightState = STATE_LANDED;
                Serial.println(">>> ATTERRI <<<");
            }
        } else {
            landedTimer = 0.0f;
        }
        break;

    case STATE_LANDED:
        break;
    }
}

// ============================================================
//  SELECTION DES GAINS (gain scheduling)
// ============================================================
void selectGains() {
    switch (flightState) {
    case STATE_BOOST: currentGains = GAINS_BOOST; break;
    case STATE_COAST: currentGains = GAINS_COAST; break;
    default:          currentGains = GAINS_BOOST; break;
    }
}

// ============================================================
//  RESET PID
// ============================================================
void resetPID() {
    error = lastError = integral = derivative = output = 0.0f;
}

// ============================================================
//  MODE TEST — BOUCLE PRINCIPALE
// ============================================================
void loopTest() {
    readIMU();

    // PID identique au mode vol
    error = setpoint - rollRate;

    float rawDerivative = (error - lastError) / dt;
    derivative = 0.2f * rawDerivative + 0.8f * derivative;
    lastError = error;

    output = currentGains.kp * error
           + currentGains.ki * integral
           + currentGains.kd * derivative;

    bool satHigh = (output >=  DEFLECTION_MAX) && (error > 0.0f);
    bool satLow  = (output <= -DEFLECTION_MAX) && (error < 0.0f);
    if (!satHigh && !satLow) {
        integral += error * dt;
        integral = constrain(integral, -INTEGRAL_MAX, INTEGRAL_MAX);
    }

    // Servo à 50 Hz
    static int servoCnt = 0;
    if (++servoCnt >= SERVO_DIVIDER) {
        applyOutput(output);
        servoCnt = 0;
    }

    // Debug série à 20 Hz
    static int serialCnt = 0;
    if (++serialCnt >= SERIAL_DIVIDER_TEST) {
        Serial.print(">roll:");     Serial.println(rollRate, 4);
        Serial.print(">output:");   Serial.println(output, 2);
        Serial.print(">integral:"); Serial.println(integral, 4);
        Serial.print(">error:");    Serial.println(error, 4);
        Serial.print(">deriv:");    Serial.println(derivative, 4);
        Serial.print(">setpt:");    Serial.println(setpoint, 4);
        serialCnt = 0;
    }
}

// ============================================================
//  COMMANDES SERIE
// ============================================================
void checkSerialCmd() {
    while (Serial.available()) {
        char c = Serial.read();
        if (c == '\n' || c == '\r') {
            if (cmdIdx > 0) {
                cmdBuf[cmdIdx] = '\0';
                executeCmd(cmdBuf);
                cmdIdx = 0;
            }
        } else if (cmdIdx < sizeof(cmdBuf) - 1) {
            cmdBuf[cmdIdx++] = c;
        }
    }
}

void executeCmd(const char* cmd) {
    switch (cmd[0]) {

    case 't':
    case 'T':
        setTestMode(!testMode);
        break;

    case 's':
    case 'S':
        if (testMode) servoSweep();
        break;

    case 'c':
    case 'C':
        if (testMode) {
            resetPID();
            finServo.write((int)SERVO_CENTER);
            Serial.println("PID reset, servo centre.");
        }
        break;

    case 'g':
    case 'G':
        if (testMode) {
            testGainSet = (testGainSet + 1) % 2;
            currentGains = (testGainSet == 0) ? GAINS_BOOST : GAINS_COAST;
            resetPID();
            Serial.print("Gains -> ");
            Serial.println(testGainSet == 0 ? "BOOST" : "COAST");
            Serial.print("  kp="); Serial.print(currentGains.kp, 6);
            Serial.print(" ki=");  Serial.print(currentGains.ki, 4);
            Serial.print(" kd=");  Serial.println(currentGains.kd, 4);
        }
        break;

    case 'p':
    case 'P':
        if (testMode) {
            char* ptr = (char*)(cmd + 1);
            char* end;
            float kp = strtof(ptr, &end);
            if (end == ptr) { Serial.println("Usage: p <kp> <ki> <kd>"); break; }
            ptr = end;
            float ki = strtof(ptr, &end);
            if (end == ptr) { Serial.println("Usage: p <kp> <ki> <kd>"); break; }
            ptr = end;
            float kd = strtof(ptr, &end);
            if (end == ptr) { Serial.println("Usage: p <kp> <ki> <kd>"); break; }
            currentGains.kp = kp;
            currentGains.ki = ki;
            currentGains.kd = kd;
            resetPID();
            Serial.print("Gains custom: kp="); Serial.print(kp, 6);
            Serial.print(" ki=");              Serial.print(ki, 4);
            Serial.print(" kd=");              Serial.println(kd, 4);
        }
        break;

    case '0':
        if (testMode) {
            setpoint = 0.0f;
            resetPID();
            Serial.println("Setpoint = 0");
        }
        break;

    case '+':
        if (testMode) {
            setpoint += 10.0f * (PI / 180.0f);  // +10 deg/s en rad/s
            Serial.print("Setpoint = ");
            Serial.print(setpoint * (180.0f / PI), 1);
            Serial.println(" deg/s");
        }
        break;

    case '-':
        if (testMode) {
            setpoint -= 10.0f * (PI / 180.0f);  // -10 deg/s en rad/s
            Serial.print("Setpoint = ");
            Serial.print(setpoint * (180.0f / PI), 1);
            Serial.println(" deg/s");
        }
        break;

    case 'i':
    case 'I':
        printTestStatus();
        break;

    default:
        Serial.print("? ");
        Serial.println(cmd);
        Serial.println("Cmds: t s c g p<kp ki kd> 0 + - i");
        break;
    }
}

// ============================================================
//  ACTIVATION / DESACTIVATION MODE TEST
// ============================================================
void setTestMode(bool on) {
    testMode = on;
    if (on) {
        currentGains = GAINS_BOOST;
        testGainSet = 0;
        setpoint = 0.0f;
        resetPID();
        digitalWrite(LED_PIN, HIGH);
        Serial.println(">>> MODE TEST <<<");
        Serial.println("Cmds: t s c g p<kp ki kd> 0 + - i");
    } else {
        resetPID();
        setpoint = 0.0f;
        flightState = STATE_PAD;
        launchConfirmCnt = 0;
        estimatedVelocity = 0.0f;
        boostTimer = 0.0f;
        landedTimer = 0.0f;
        finServo.write((int)SERVO_CENTER);
        digitalWrite(LED_PIN, LOW);
        Serial.println(">>> MODE VOL <<<");
    }
}

// ============================================================
//  SWEEP SERVO
//  Balayage 0 -> +max -> -max -> 0 par pas de 5°
// ============================================================
void servoSweep() {
    Serial.println("Sweep...");

    for (float d = 0.0f; d <= DEFLECTION_MAX; d += 5.0f) {
        finServo.write((int)(SERVO_CENTER + d));
        Serial.print("  +"); Serial.println((int)d);
        delay(200);
        IWatchdog.reload();
    }
    for (float d = DEFLECTION_MAX; d >= -DEFLECTION_MAX; d -= 5.0f) {
        finServo.write((int)(SERVO_CENTER + d));
        Serial.print("  "); Serial.println((int)d);
        delay(200);
        IWatchdog.reload();
    }
    for (float d = -DEFLECTION_MAX; d <= 0.0f; d += 5.0f) {
        finServo.write((int)(SERVO_CENTER + d));
        Serial.print("  "); Serial.println((int)d);
        delay(200);
        IWatchdog.reload();
    }

    finServo.write((int)SERVO_CENTER);
    Serial.println("Sweep ok.");
}

// ============================================================
//  DUMP ETAT COMPLET
// ============================================================
void printTestStatus() {
    Serial.println("==== ETAT ====");
    Serial.print("Mode:   "); Serial.println(testMode ? "TEST" : "VOL");
    Serial.print("State:  "); Serial.println((int)flightState);
    Serial.println("-- Gains --");
    Serial.print("  set=");  Serial.println(testGainSet == 0 ? "BOOST" : "COAST");
    Serial.print("  kp=");  Serial.println(currentGains.kp, 6);
    Serial.print("  ki=");  Serial.println(currentGains.ki, 4);
    Serial.print("  kd=");  Serial.println(currentGains.kd, 4);
    Serial.println("-- PID --");
    Serial.print("  setpt=");  Serial.print(setpoint * (180.0f / PI), 2); Serial.println(" deg/s");
    Serial.print("  roll=");   Serial.print(rollRate * (180.0f / PI), 2); Serial.println(" deg/s");
    Serial.print("  err=");    Serial.println(error, 4);
    Serial.print("  int=");    Serial.println(integral, 4);
    Serial.print("  drv=");    Serial.println(derivative, 4);
    Serial.print("  out=");    Serial.print(output, 2); Serial.println(" deg");
    Serial.println("-- IMU --");
    Serial.print("  gX="); Serial.print(imu.gyrX(), 2); Serial.println(" d/s");
    Serial.print("  gY="); Serial.print(imu.gyrY(), 2); Serial.println(" d/s");
    Serial.print("  gZ="); Serial.print(imu.gyrZ(), 2); Serial.println(" d/s");
    Serial.print("  aX="); Serial.print(imu.accX(), 3); Serial.println(" g");
    Serial.print("  aY="); Serial.print(imu.accY(), 3); Serial.println(" g");
    Serial.print("  aZ="); Serial.print(imu.accZ(), 3); Serial.println(" g");
    Serial.print("  mag="); Serial.print(accelMag, 3);  Serial.println(" g");
    Serial.println("==============");
}
