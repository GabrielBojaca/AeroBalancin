#include <Arduino.h>
#include <Wire.h>
#include <AS5600.h>

// ---------------- PINES ----------------
#define PWM_PIN   18
#define SDA_PIN   21
#define SCL_PIN   22

#define BTN_DOWN  19   // Bajar referencia
#define BTN_UP    23   // Subir referencia

// ---------------- OBJETO ENCODER ----------------
AS5600 encoder;

// ---------------- PWM ----------------
const int PWM_CHANNEL = 1;
const int PWM_FREQ    = 20000;
const int PWM_RES     = 10;

// ---------------- PID ----------------
float Kp = 7.80e-1;
float Ki = 3.9979;
float Kd = 6.214e-2;

float setpoint = -10.0;        
const float step_ref = 5.0;    

float errorPrev = 0.0;
float integral = 0.0;
unsigned long tPID_prev = 0;

// -------- PID --------
float computePID(float angle) {

    unsigned long now = millis();
    float dt = (now - tPID_prev) / 1000.0;
    if (dt <= 0) dt = 0.001;

    tPID_prev = now;

    float error = setpoint - angle;

    // Integral con anti-windup
    integral += error * dt;
    if (integral > 200) integral = 200;
    if (integral < -200) integral = -200;

    float derivative = (error - errorPrev) / dt;
    errorPrev = error;

    float out = Kp * error + Ki * integral + Kd * derivative;

    //Serial.print("0 1023 Kpu: ");
    //Serial.print("1023,0,");
    //Serial.print(error*Kp);
    //Serial.print(" Kiu: ");
    //Serial.print(",");
    //Serial.print(integral*Ki);
    //Serial.print(",");
    //Serial.print(" Kdu: ");
    //Serial.print(Kd * derivative);
    return out;
}


void setup() {
    Serial.begin(115200);

    ledcSetup(PWM_CHANNEL, PWM_FREQ, PWM_RES);
    ledcAttachPin(PWM_PIN, PWM_CHANNEL);

    Wire.begin(SDA_PIN, SCL_PIN);
    encoder.begin();

    pinMode(BTN_DOWN, INPUT_PULLUP);
    pinMode(BTN_UP, INPUT_PULLUP);

    delay(500);
}



void loop() {

    bool down = digitalRead(BTN_DOWN) == LOW;
    bool up   = digitalRead(BTN_UP)   == LOW;

    if (down) {
        setpoint -= step_ref;
        if (setpoint < -90) setpoint = -90;
        delay(150); 
    }

    if (up) {
        setpoint += step_ref;
        if (setpoint > 40) setpoint = 40;
        delay(150); 
    }

    uint16_t raw = encoder.readAngle();
    float angle = 45 - ((raw * 360.0) / 4096.0);

    float pidOut = computePID(angle);

    int pwm = (int)pidOut;


    Serial.print(" Ref:");
    //Serial.print(",");
    Serial.print(setpoint);
    Serial.print("  Ang:");
    Serial.print(",");
    Serial.print(angle);
    Serial.print("  PWM:");
    Serial.print(",");
    Serial.println(pwm);

    if (pwm < 0) pwm = 0;
    if (pwm > 1023) pwm = 1023;

    ledcWrite(PWM_CHANNEL, pwm);


    delay(10);
}
