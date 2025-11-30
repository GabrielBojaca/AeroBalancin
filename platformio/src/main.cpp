#include <Arduino.h>
#include <Wire.h>
#include <AS5600.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <BasicLinearAlgebra.h>

using namespace BLA; // Para LinealAgebra

// ---------------- PINES ----------------
#define PWM_PIN 18
#define SDA_PIN 21
#define SCL_PIN 22

#define BTN_DOWN 19 // Bajar referencia
#define BTN_UP 23   // Subir referencia

// ----------------- MODO DE CONTROL --------------
#define PID 0
#define SMC 1
#define MRAC 2
#define HINF 3
#define PI_D 4 // PI+D

// ---------------- PROTOTIPOS -------------------
float computePID(float angle);
float computePI_D(float angle, float wz);
void calibrateGyroZ(int samples);
float hinf_control(float ref);
float getGyroZ();

// ---------------- OBJETO ENCODER IMU ----------------
AS5600 encoder;
Adafruit_MPU6050 mpu;
sensors_event_t accel, gyro, temp;

// ---------------- VARIABLES ENCODER IMU ----------------
float gyroZ_bias = 0.0;

// ----------------- Controlador -----------------
int controlMode = HINF;

// ---------------- PWM ----------------
const int PWM_CHANNEL = 1;
const int PWM_FREQ = 20000;
const int PWM_RES = 10;

// ---------------- PID ----------------
float Kp = 7.80e-1;
float Ki = 3.9979;
float Kd = 6.214e-2;

float errorPrev = 0.0;
float integral = 0.0;
unsigned long tPID_prev = 0;
// -------------------------------------

// --- HINF ---
BLA::Matrix<4, 1> xk = {0, 0, 0, 0};
float a = 5.2070e-08;
float b = 2.5241e-05;
float c = 0.0063;

// ------------- FRECUENCIA CONTROL MUESTREO ------------
const unsigned long Ts_us = 10 * 1e3; // 1 ms = 1000 us -> 100ms
unsigned long lastMicros = 0;

const unsigned long Ts_buttons_ms = 120; // tiempo de lectura de botones (Ojo está en ms, no us)
unsigned long t_buttons_prev = 0;

float setpoint = 80.0;
const float step_ref = 5.0;

void setup()
{

    pinMode(BTN_DOWN, INPUT_PULLUP);
    pinMode(BTN_UP, INPUT_PULLUP);
    ledcSetup(PWM_CHANNEL, PWM_FREQ, PWM_RES);
    ledcAttachPin(PWM_PIN, PWM_CHANNEL);
    ledcWrite(PWM_CHANNEL, 0);

    Serial.begin(115200);
    delay(2000);

    // --- I2C ---
    Wire.begin(SDA_PIN, SCL_PIN);
    Wire.setTimeout(1);
    Wire.setClock(400000);

    // --- AS5600 ---
    if (!encoder.begin())
        Serial.println("No se detecta AS5600");
    else
        Serial.println("AS5600 detectado");

    // --- MPU6050 ---
    if (!mpu.begin(0x68, &Wire))
    {
        Serial.println("No se detecta el MPU6050");
    }
    else
    {
        Serial.println("MPU6050 detectado correctamente");
    }

    // Configuración del MPU6050
    mpu.setAccelerometerRange(MPU6050_RANGE_2_G);
    mpu.setGyroRange(MPU6050_RANGE_2000_DEG);
    mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);

    calibrateGyroZ(500);

    Serial.println("Sistema listo");
}

void loop()
{
    unsigned long now_us = micros();
    unsigned long now_ms = millis();

    if (now_us - lastMicros >= Ts_us)
    {
        lastMicros = now_us;

        // --- Lectura encoder ---
        uint16_t raw = encoder.readAngle();
        // float angle = 45 - ((raw * 360.0) / 4096.0);
        //float angle = 138.82 - ((raw * 360.0) / 4096.0);
        float angle = ((raw * 360.0) / 4096.0); //Singularidad eliminada
        if (angle <= 180){
            angle = 138.82-angle;
        } else{
            angle = (498.82-angle);
            //angle = (718.82-angle);
        }


        
        // --- Lectura giroscopio ---
        float wz = getGyroZ(); // velocidad angular calibrada

        mpu.getEvent(&accel, &gyro, &temp);

        float pidOut = 0;
        int pwm = 0;
        float u = 0;

        switch (controlMode)
        {
        case PID:
            pidOut = computePID(angle);
            break;
        case HINF:
            pwm = (int)hinf_control((setpoint - angle)); // DEG_TO_RAD);
            break;
        case SMC:
            pidOut = computePID(angle);
            break;
        case MRAC:
            pidOut = computePID(angle);
            break;
        case PI_D:
            pidOut = computePI_D(angle, wz);
            break;
        default:
            pidOut = 0;
            break;
        }
        // int pwm = (int)pidOut;

        Serial.printf("%06lu", millis());
        Serial.print(" Ref: ");
        Serial.print(setpoint);
        Serial.print("  Ang: ");
        Serial.print(angle);
        Serial.print("  PWM: ");
        Serial.print(pwm);
        Serial.print(" GyroZ: ");
        Serial.println(wz, 4);

        if (pwm < 0)
            pwm = 0;
        if (pwm > 1023)
            pwm = 1023;

        ledcWrite(PWM_CHANNEL, pwm);
    }

    //---------------------------------------------------------------------------

    if (now_ms - t_buttons_prev >= Ts_buttons_ms)
    {
        t_buttons_prev = now_ms;

        bool down = digitalRead(BTN_DOWN) == LOW;
        bool up = digitalRead(BTN_UP) == LOW;

        if (down)
        {
            setpoint -= step_ref;
            if (setpoint < -180)
                setpoint = -180;
        }

        if (up)
        {
            setpoint += step_ref;
            if (setpoint > 180)
                setpoint = 180;
        }
    }
}

// -------- PID --------
float computePID(float angle)
{

    unsigned long now = millis();
    float dt = (now - tPID_prev) / 1000.0;
    if (dt <= 0)
        dt = 0.001;

    tPID_prev = now;

    float error = setpoint - angle;

    // Integral con anti-windup
    integral += error * dt;
    if (integral > 200)
        integral = 200;
    if (integral < -200)
        integral = -200;

    float derivative = (error - errorPrev) / dt;
    errorPrev = error;

    float out = Kp * error + Ki * integral + Kd * derivative;

    return out;
}

float computePI_D(float angle, float wz)
{
    unsigned long now = millis();
    float dt = (now - tPID_prev) / 1000.0;
    if (dt <= 0)
        dt = 0.001;

    tPID_prev = now;

    float error = setpoint - angle;

    integral += error * dt;
    if (integral > 200)
        integral = 200;
    if (integral < -200)
        integral = -200;

    // --- PI + D (gyro) ---
    float out = Kp * error + Ki * integral - Kd * wz;

    return out;
}

void calibrateGyroZ(int samples = 500)
{
    Serial.println("🔧 Calibrando Gyro Z...");
    float sum = 0;

    for (int i = 0; i < samples; i++)
    {
        mpu.getEvent(&accel, &gyro, &temp);
        sum += gyro.gyro.z;
        delay(2); // 500 muestras son 1 segundo
    }

    gyroZ_bias = sum / samples;

    Serial.print("Bias Gyro Z: ");
    Serial.println(gyroZ_bias, 5);
}

float getGyroZ()
{
    mpu.getEvent(&accel, &gyro, &temp);
    return gyro.gyro.z - gyroZ_bias;
}

float hinf_control(float ref)
{
    static const BLA::Matrix<4, 4> Ad = {
        0.9982, 1.68e-25, -1.348e-21, 3.178e-23,
        0.04555, -3.879e-09, 1.907e-06, -7.352e-07,
        395.8, -3.355e-05, 0.0165, -0.006359,
        1257.0, 0.00347, -1.706, 0.6578};

    static const BLA::Matrix<4, 1> Bd = {
        0.01998,
        -0.008256,
        -71.72,
        558.2};

    static const BLA::Matrix<1, 4> Cd = {
        70.92, 0.0159, -0.1519, -0.006245};

    static const float Dd = 0.0f;

    // ---- Control H∞ discreto ----
    xk = Ad * xk + Bd * ref;

    // Anti overflow

    for (int i = 0; i < 4; i++)
    {
        // if (xk(i,0) > 1023) xk(i,0) = 1023;
        // if (xk(i,0) < -1023) xk(i,0) = -1023;
        Serial.print(" ");
        Serial.print(xk(i, 0));
        Serial.print(" ");
    }

    float u = (Cd * xk)(0, 0) + Dd * ref;

    // u = (u + 0.009739) / (8.6928e-5);
    // u = (11503.77) * u + 112.0;
    // u = (u) / (8.6928e-5);

    return u + 650;
}
