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

const int SERVO_DIVIDER  = 6;   // -> 50 Hz servo
const int SERIAL_DIVIDER = 30;  // -> ~10 Hz debug

// ============================================================
//  SEUILS DETECTION DE VOL
//
//  NOTE — Axe de poussée :
//  On suppose que l'axe X de l'IMU est aligné avec l'axe
//  longitudinal de la fusée (nez = +X), cohérent avec
//  gyrX() = roll.  Si le montage est différent, modifier
//  la ligne thrustAccel dans readIMU().
// ============================================================
const float LAUNCH_ACCEL_G      = 3.0f;   // Seuil lancement (g, sur magnitude)
const float BURNOUT_ACCEL_G     = 2.0f;   // Sous ce seuil -> fin de poussée (g, magnitude)
const float LANDED_DURATION     = 2.0f;   // Durée à ~1g pour déclarer atterri (s)
const float MIN_BOOST_TIME      = 0.5f;   // Durée min en BOOST avant transition (s)
const int   LAUNCH_CONFIRM_COUNT = 3;     // Échantillons consécutifs pour confirmer lancement

// Variables état de vol
float thrustAccel       = 0.0f;  // Accélération axiale (g)
float accelMag          = 0.0f;  // Magnitude accélération totale (g)
float estimatedVelocity = 0.0f;  // Estimation vitesse verticale (m/s)
float landedTimer       = 0.0f;
float boostTimer        = 0.0f;
int   launchConfirmCnt  = 0;

unsigned long lastTime;

// ============================================================
//  PROTOTYPES
// ============================================================
void readIMU();
void applyOutput(float out);
void updateFlightState();
void selectGains();

// ============================================================
//  SETUP
// ============================================================
void setup() {
    Serial.begin(115200);
    pinMode(LED_PIN, OUTPUT);

    // --- SPI (config spécifique HGLRC F405) ---
    SPI.setSCLK(PA5);
    SPI.setMISO(PA6);
    SPI.setMOSI(PA7);
    SPI.begin();

    // --- IMU ---
    if (imu.begin() < 0) {
        Serial.println("ERREUR : ICM42688 introuvable !");
        // Fail-safe : servo au neutre, LED clignotante
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
    // Si le code plante, le MCU redémarre automatiquement.
    IWatchdog.begin(4000000);

    Serial.println("Systeme pret. Attente lancement...");
}

// ============================================================
//  BOUCLE PRINCIPALE — 300 Hz
// ============================================================
void loop() {
    unsigned long now = micros();
    if (now - lastTime < LOOP_INTERVAL_US) return;
    lastTime += LOOP_INTERVAL_US;

    IWatchdog.reload();

    // 1. Lecture capteurs
    readIMU();

    // 2. Machine d'états de vol
    updateFlightState();

    // 3. Contrôle PID (actif uniquement en BOOST / COAST)
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

    // 4. Debug série (~10 Hz)
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
        // Lancement : forte accélération confirmée sur plusieurs échantillons
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
        // Estimation vitesse (intégration accéléromètre - 1g gravité)
        estimatedVelocity += (thrustAccel - 1.0f) * 9.81f * dt;

        // Fin de poussée : accélération retombe (après temps minimum)
        if (boostTimer > MIN_BOOST_TIME && accelMag < BURNOUT_ACCEL_G) {
            flightState = STATE_COAST;
            Serial.println(">>> COAST <<<");
        }
        break;

    case STATE_COAST:
        estimatedVelocity += (thrustAccel - 1.0f) * 9.81f * dt;

        // Apogée : vitesse estimée passe sous zéro
        if (estimatedVelocity <= 0.0f) {
            flightState = STATE_DESCENT;
            Serial.println(">>> DESCENTE <<<");
        }
        break;

    case STATE_DESCENT:
        // Atterrissage : accélération stable ~1g pendant LANDED_DURATION
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
