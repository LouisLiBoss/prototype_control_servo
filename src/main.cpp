#include <Arduino.h>
#include <SPI.h>
#include <Servo.h>
#include <ICM42688.h>

// --- Configuration Matérielle ---
const uint8_t IMU_CS = PA4;
const int SERVO_PIN = PA15;
ICM42688 imu(SPI, IMU_CS);
Servo finServo;

// --- Structure et Paramètres du Gain Scheduling ---
struct PIDGains { 
    float kp;
    float ki;
    float kd;
};

// Tableau de gains
const PIDGains lowSpeed     = {0.0816f, 5.3373f, 0.0}; 
const PIDGains midSpeed     = {0.016591f, 4.203f, 0.0f};  
const PIDGains highSpeed    = {1.3841e-07f, 3.4397f, 0.0f}; 
const PIDGains extremeSpeed = {0.27429f, 1.0444f, 0.0f}; 
const PIDGains lightSpeed   = {0.13683f, 0.49909f, 0.0f};

PIDGains currentGains = lowSpeed;

// --- Variables de contrôle et de la vitesse verticale (dérivée de l'altitude) ---
float error, lastError, integral, derivative, output; // Variables pour le PID (voir ligne 74 à 81) )
float setpoint = 0.0f; // consigne de vitesse angulaire (rad/s)
float rollRate = 0.0f;


// --- Contraintes Servo (output min/max) ---
const float OUT_MIN = -15.0f;
const float OUT_MAX = 15.0f;

void applyOutput(float out); 

// --- Timing
unsigned long lastTime;
float dt = 3333.0f / 1000000.0f;  
void setup() {
    Serial.begin(115200);

    // 1. Initialisation Bus SPI (Config spécifique HGLRC F405)
    SPI.setSCLK(PA5);
    SPI.setMISO(PA6);
    SPI.setMOSI(PA7);
    SPI.begin();

    // 2. Initialisation IMU
    if (imu.begin() < 0) {
        Serial.println("ERREUR : ICM42688 introuvable !");
        while(1);
    }
    
    Serial.println("Calibration Gyro... NE PAS BOUGER");
    imu.calibrateGyro();
    Serial.println("IMU prête !");

    // 3. Initialisation Servo
    finServo.attach(SERVO_PIN);
    
    lastTime = micros();
    
    Serial.println("Test survie servo...");
    finServo.write(110); // Bouge un peu
    delay(500);
    finServo.write(90);  // Revient au centre
}

void loop() { // boucle principale optimisée pour 300 Hz, avec contrôle strict du timing et fonctions déportées pour la lisibilité
    unsigned long currentTime = micros();
    
if (currentTime - lastTime >= 3333) {
    lastTime += 3333;

    // --- 1. gyro ---
    imu.getAGT();
    rollRate = imu.gyrX() * (PI / 180.0f); // conversion en rad/s

    // 2. Calcul de l'erreur pour le PID
    error = setpoint - rollRate;

    // 3. Intégral
    if (!(output >= OUT_MAX && error > 0) && !(output <= OUT_MIN && error < 0)) { // clamping
        integral += error * dt; // Calcul de l'intégrale
    }

    // 4. Dérivée + FILTRE (pour éviter les sauts brusques)
        float raw_derivative = (error - lastError) / dt;
        // On ne prend que 1% de la nouvelle valeur pour lisser le mouvement (à voir)
        derivative = (0.01f * raw_derivative) + (0.99f * derivative); 
        lastError = error;

    // 5. Output
    output = (currentGains.kp * error) + (currentGains.ki * integral) + (currentGains.kd * derivative);

    static int servoDivider = 0;
        servoDivider++;
        if (servoDivider >= 6) {
            applyOutput(output); 
            servoDivider = 0;
        }
    // 6. Saturation et Envoi au Servo
    static int count = 0;
    count++;
    if (count >= 15) { // Environ 10 fois par seconde (300 / 30)
    Serial.print(">rollRate (rad/s):"); Serial.println(rollRate);
    Serial.print(">Output: "); Serial.println(output);
    Serial.print(">rawDerivative:"); Serial.println(raw_derivative);
    Serial.print(">Derivative:"); Serial.println(derivative);
    Serial.print(">Integral:"); Serial.println(integral);
    count = 0;
    }
}
}

void applyOutput(float out) {
    out = out * (180/PI); // conversion en degrés

    // limitation de u dans [-15;15]
    if (out > OUT_MAX) {
    out = OUT_MAX;
    } else if (out < OUT_MIN) {
    out = OUT_MIN;
    }

    finServo.write(out);
}