#include <Arduino.h>
#include <Wire.h>
#include <AS5600.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>

// --- Pines ---
#define PWM_PIN   18
#define SDA_PIN   21
#define SCL_PIN   22

// --- Objetos ---
AS5600 encoder;
Adafruit_MPU6050 mpu;

// --- Configuración PWM ---
const int PWM_CHANNEL = 1;
const int PWM_FREQ = 20000;   // 20 kHz
const int PWM_RES  = 10;      // 10 bits → 0–1023

// --- Variables de rampa ---
int pwmValue = 0;
const int PWM_MIN = 0;
const int PWM_MAX = 871;
const int PWM_STEP = 50;        // incremento fijo por paso
const unsigned long T_STEP = 2000; // tiempo entre pasos (ms)
unsigned long t_prev = 0;

bool subiendo = true;  // dirección actual de la rampa

void setup() {
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
  if (!encoder.begin()) Serial.println("❌ No se detecta AS5600");
  else Serial.println("✅ AS5600 detectado");

  // --- MPU6050 ---
  if (!mpu.begin(0x68, &Wire)) {
    Serial.println("❌ No se detecta el MPU6050");
  } else {
    Serial.println("✅ MPU6050 detectado correctamente");
  }

  // Configuración del MPU6050
  mpu.setAccelerometerRange(MPU6050_RANGE_2_G);
  mpu.setGyroRange(MPU6050_RANGE_2000_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);

  Serial.println("✅ Sistema listo");
}

void loop() {
  unsigned long t_now = millis();

  // --- Leer ángulo ---
  uint16_t rawAngle = encoder.readAngle();
  float angulo = (rawAngle * 360.0f) / 4096.0f;
  angulo = 139.0 - angulo;

  // --- Leer giroscopio ---
  sensors_event_t accel, gyro, temp;
  mpu.getEvent(&accel, &gyro, &temp);

  // --- Actualizar rampa ---
  if (t_now - t_prev >= T_STEP) {
    t_prev = t_now;

    if (subiendo) {
      pwmValue += PWM_STEP;
      if (pwmValue >= PWM_MAX) {
        pwmValue = PWM_MAX;
        subiendo = false;
        Serial.println("🔻 Bajando");
      }
    } else {
      pwmValue -= PWM_STEP;
      if (pwmValue <= PWM_MIN) {
        pwmValue = PWM_MIN;
        subiendo = true;
        Serial.println("🔺 Subiendo");
      }
    }

    ledcWrite(PWM_CHANNEL, pwmValue);
  }


      // --- Debug ---
    Serial.print(millis());
    Serial.print(",");
    Serial.print(pwmValue);
    Serial.print(",");
    Serial.print(angulo, 2);
    Serial.print(",");
    Serial.println(gyro.gyro.z, 2);

  delay(100);
}
