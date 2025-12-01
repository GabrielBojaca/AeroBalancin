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
#define PI_D 4    // PI+D
#define PI_SLOW 5 // PID
#define OFF_CONTROL 80

// ---------------- PROTOTIPOS -------------------
float computePID(float angle, bool reset);
float computePI_D(float angle, float wz, bool reset);
void calibrateGyroZ(int samples);
float hinf_control(float ref);
float getGyroZ();
float computePISlow(float angle, float setpoint, bool reset);
void leerComandosSerial();

// ---------------- OBJETO ENCODER IMU ----------------
AS5600 encoder;
Adafruit_MPU6050 mpu;
sensors_event_t accel, gyro, temp;

// ---------------- VARIABLES ENCODER IMU ----------------
float gyroZ_bias = 0.0;

// ----------------- Controlador -----------------
int controlMode = PI_D;
int controlModeActual = controlMode;

bool run = 0;

bool enableAprox = true;
bool aproximacion = true;
unsigned long tAproximacion = 0;
int pwmEq = 0;

// ---------------- PWM ----------------
const int PWM_CHANNEL = 1;
const int PWM_FREQ = 20000;
const int PWM_RES = 10;

// ---------------- PID ----------------
float Kp = 15;
float Ki = 3.9979e-1;
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

// --- SALIDA ---
int pwm = 0;

// --- DEBUG ---
bool graficarArd = false;

// ------------- FRECUENCIA CONTROL MUESTREO ------------
const unsigned long Ts_us = 10 * 1e3; // 1 ms = 1000 us -> 100ms
unsigned long lastMicros = 0;

const unsigned long Ts_buttons_ms = 120; // tiempo de lectura de botones (Ojo está en ms, no us)
unsigned long t_buttons_prev = 0;

float p_operacion = 80;
float setpoint = 10.0;
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
        if (!graficarArd)
            Serial.println("No se detecta AS5600");
        else if (!graficarArd)
            Serial.println("AS5600 detectado");

    // --- MPU6050 ---
    if (!mpu.begin(0x68, &Wire))
    {
        if (!graficarArd)
            Serial.println("No se detecta el MPU6050");
    }
    else
    {
        if (!graficarArd)
            Serial.println("MPU6050 detectado correctamente");
    }

    // Configuración del MPU6050
    mpu.setAccelerometerRange(MPU6050_RANGE_2_G);
    mpu.setGyroRange(MPU6050_RANGE_2000_DEG);
    mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);

    calibrateGyroZ(500);

    if (!graficarArd)
        Serial.println("Sistema listo");
    aproximacion = true;
    tAproximacion = millis();
}

void loop()
{
    unsigned long now_us = micros();
    unsigned long now_ms = millis();

    leerComandosSerial();

    if (now_us - lastMicros >= Ts_us)
    {
        lastMicros = now_us;

        // --- Lectura encoder ---
        uint16_t raw = encoder.readAngle();
        float angle = ((raw * 360.0) / 4096.0); // Singularidad eliminada
        if (angle <= 180)
        {
            angle = 135.82 - angle;
        }
        else
        {
            angle = (495.82 - angle);
        }

        // --- Lectura giroscopio ---
        float wz = getGyroZ(); // velocidad angular calibrada

        mpu.getEvent(&accel, &gyro, &temp);
        if (run)
        {
            // --- Rutina aproximación ---
            if (aproximacion && enableAprox) // enableAprox habilitia o deshabilita el uso de la rutina de aprox.
            {
                controlModeActual = controlMode;
                setpoint += 0.02;
                if(abs(setpoint-p_operacion)< 0.1){
                    aproximacion = false;
                    controlModeActual = controlMode;
                }
            }
            else
            {
                controlModeActual = controlMode;
            }

            // --- Modo de control ---

            switch (controlModeActual)
            {
            case PID:
                pwm = computePID(angle, false);
                break;
            case HINF:
                pwm = (int)hinf_control((setpoint - angle));
                break;
            case SMC:
                pwm = computePID(angle,false);
                break;
            case MRAC:
                pwm = computePID(angle,false);
                break;
            case PI_D:
                pwm = computePI_D(angle, wz,false);
                break;
            case PI_SLOW:
                pwm = computePISlow(angle, setpoint, false);
                break;
            case OFF_CONTROL:
                pwm = 0;
                aproximacion = false;
                break;
            default:
                pwm = 0;
                break;
            }
        }

        // int pwm = (int)pidOut;
        if (!graficarArd)
        {
            Serial.printf("%06lu", millis());
            Serial.print(" Ref: ");
            Serial.print(setpoint);
            Serial.print("  Ang: ");
            Serial.print(angle);
            Serial.print("  PWM: ");
            Serial.print(pwm);
            Serial.print("  MODO_ACT: ");
            if(controlModeActual == 0) {Serial.print("PID");} //PID
            else if(controlModeActual == 1){Serial.print("SMC");}
            else if(controlModeActual == 2){Serial.print("MRAC");}
            else if(controlModeActual == 3){Serial.print("HINF");}
            else if(controlModeActual == 4){Serial.print("PI_D");}
            else if(controlModeActual == 5){Serial.print("PI_SLOW");}
            else if(controlModeActual == 80){Serial.print("80 STOP");}
            Serial.print("  MODO_CONT: ");
            if(controlMode == 0) {Serial.print("PID");} //PID
            else if(controlMode == 1){Serial.print("SMC");}
            else if(controlMode == 2){Serial.print("MRAC");}
            else if(controlMode == 3){Serial.print("HINF");}
            else if(controlMode == 4){Serial.print("PI_D");}
            else if(controlMode == 5){Serial.print("PI_SLOW");}
            else if(controlMode == 80){Serial.print("80 STOP");}
            Serial.print("  RUN: ");
            Serial.print(run);
            Serial.print("  APROX: ");
            Serial.print(aproximacion);
            Serial.print(" GyroZ: ");
            Serial.println(wz, 4);

        }
        else
        {
            Serial.print("");
            Serial.print(setpoint);
            Serial.print(",");
            Serial.print(angle);
            Serial.print(",");
            Serial.print(pwm / 10); // escalado /10
            Serial.print(",0, 180,");
            Serial.println(wz, 4); // escalado x10
        }

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
            if (setpoint < -120)
                setpoint = -120;
        }

        if (up)
        {
            setpoint += step_ref;
            if (setpoint > 120)
                setpoint = 120;
        }
    }
}

// -------- PID --------
float computePID(float angle, bool reset)
{
    if(reset){
        float errorPrev = 0.0;
        float integral = 0.0;
    }
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

float computePI_D(float angle, float wz, bool reset)
{
    if(reset){
       integral = 0; 
    }
    unsigned long now = millis();
    float dt = (now - tPID_prev) / 1000.0;
    if (dt <= 0)
        dt = 0.001;

    tPID_prev = now;

    float error = setpoint - angle;

    integral += error * dt;
    if (integral > 500)
        integral = 500.0;
    if (integral < -500)
        integral = -500.0;

    // --- PI + D (gyro) ---
    float out = Kp * error + Ki * integral - Kd * wz;

    return out;
}

void calibrateGyroZ(int samples = 500)
{
    if (!graficarArd)
        Serial.println("Calibrando Gyro Z...");
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
        0.9991, 0.0, 0.0, 0.0,
        0.269, -0.0003531, 2.427e-05, -4.32e-06,
        278.5, -0.3545, 0.02438, -0.004338,
        1393, 40.99, -2.818, 0.5017};

    static const BLA::Matrix<4, 1> Bd = {
        0.03998,
        -0.03136,
        -32.21,
        860.1};

    static const BLA::Matrix<1, 4> Cd = {
        151, 42.79, -0.5241, -0.01239};

    static const float Dd = 0.0f;

    //||S||_∞  = 1.198
    //||T||_∞  = 1.23447
    //||KS||_∞ = 17.559

    // ---- Control H∞ discreto ----
    xk = Ad * xk + Bd * ref;

    // Anti overflow
    /*
    for (int i = 0; i < 4; i++)
    {
        // if (xk(i,0) > 1023) xk(i,0) = 1023;
        // if (xk(i,0) < -1023) xk(i,0) = -1023;
        //Serial.print(" ");
        //Serial.print(xk(i, 0));
        //Serial.print(" ");
    }
    */

    float u = (Cd * xk)(0, 0) + Dd * ref;

    if (pwm == 0)
    {
        u = u + 660;
    }
    else
    {
        u = u + 660;//pwmEq;
    }

    return u;
}

float computePISlow(float angle, float setpoint, bool reset)
{
    static float integral_aprox = 0.0;
    static float errorPrev_aprox = 0.0;
    static unsigned long lastTime_aprox = 0;

    if (reset)
    {
        integral_aprox = 0.0;
        errorPrev_aprox = 0.0;
        lastTime_aprox = 0;
    }

    const float Kp_aprox = 1.0;
    const float Ki_aprox = 2; // 1.2
    const float Kd_aprox = 1; // 0.8

    unsigned long now_aprox = millis();
    float dt_aprox = (now_aprox - lastTime_aprox) / 1000.0;
    if (dt_aprox <= 0)
        dt_aprox = 0.001;
    lastTime_aprox = now_aprox;

    float error_aprox = setpoint - angle;

    integral_aprox += error_aprox * dt_aprox;

    if (integral_aprox > 1000)
        integral_aprox = 1000;
    if (integral_aprox < -1000)
        integral_aprox = -1000;

    float derivative_aprox = (error_aprox - errorPrev_aprox) / dt_aprox;
    errorPrev_aprox = error_aprox;

    float u_aprox = Kp_aprox * error_aprox + Ki_aprox * integral_aprox + Kd_aprox * derivative_aprox;

    return u_aprox;
}

void leerComandosSerial()
{
    static String buffer = "";

    // Leer todo lo disponible sin bloquear
    while (Serial.available())
    {
        char c = Serial.read();

        // Si recibimos <enter> o coma, procesamos el número
        if (c == '\n' || c == '\r' || c == ',')
        {
            if (buffer.length() > 0)
            {
                long numero = buffer.toInt();
                buffer = ""; // limpiar

                // --- Rango 0 a 180 -> cambiar setpoint ---
                if (numero == 0)
                {
                    controlMode = OFF_CONTROL;
                    aproximacion = false;
                    computePISlow(0, 0, true); // reset PID
                }
                else if (numero > 0 && numero <= 180)
                {
                    setpoint = numero;
                    //Serial.println("Setpoint actualizado");
                }

                // --- Rango 1000 a 2000 -> hacer otra acción ---
                else if (numero >= 200 && numero <= 300)
                {
                    // ejemplo: cambiar modo de control
                    //computePISlow(0, 0, true); // reset PID
                    aproximacion = true;
                    tAproximacion = millis();
                    setpoint = 10;
                    //Serial.println("aproximacion true");
                    //delay(1000);
                }

                // --- Rango 3000 a 4000 ---
                else if (numero >= 300 && numero <= 400)
                {
                    controlMode = PI_D;
                    computePI_D(0,0,true);
                    //Serial.println("PID");
                }
                else if (numero >= 400 && numero <= 500)
                {
                    controlMode = HINF;
                    //Serial.println("HINF");
                }

                // --- Número fuera de todos los rangos ---
                else
                {
                    run = !run;
                    tAproximacion = millis();
                    computePISlow(0, 0, true); // reset PID
                }
            }
        }
        else if (isDigit(c))
        {
            buffer += c; // solo guardar números
        }
        else if (c == '-') // permitir números negativos
        {
            buffer += c;
        }
    }
}
