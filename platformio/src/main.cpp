#include <Arduino.h>
#include <Wire.h>
#include <AS5600.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>

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

// ---------------- PROTOTIPOS -------------------
float computePID(float angle);
void calibrateGyroZ(int samples);
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

// ------------- FRECUENCIA CONTROL MUESTREO ------------
const unsigned long Ts_us = 100 * 1e3; // 1 ms = 1000 us -> 100ms
unsigned long lastMicros = 0;

const unsigned long Ts_buttons_ms = 120; // tiempo de lectura de botones (Ojo está en ms, no us)
unsigned long t_buttons_prev = 0;

float setpoint = -10.0;
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
    Wire.setTimeout(50);
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
        float angle = 45 - ((raw * 360.0) / 4096.0);

        // --- Lectura giroscopio ---
        float wz = getGyroZ();  // velocidad angular calibrada

        mpu.getEvent(&accel, &gyro, &temp);

        float pidOut = 0;

        switch (controlMode)
        {
        case PID:
            pidOut = computePID(angle);
            break;
        case HINF:
            pidOut = computePID(angle);
            break;
        case SMC:
            pidOut = computePID(angle);
            break;
        case MRAC:
            pidOut = computePID(angle);
            break;
        default:
            pidOut = 0;
            break;
        }

        int pwm = (int)pidOut;

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
            if (setpoint < -90)
                setpoint = -90;
        }

        if (up)
        {
            setpoint += step_ref;
            if (setpoint > 40)
                setpoint = 40;
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

void calibrateGyroZ(int samples = 500)
{
    Serial.println("🔧 Calibrando Gyro Z...");
    float sum = 0;

    for (int i = 0; i < samples; i++)
    {
        mpu.getEvent(&accel, &gyro, &temp);
        sum += gyro.gyro.z;
        delay(2);   // 500 muestras son 1 segundo
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
