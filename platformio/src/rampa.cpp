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
int pwmValue = 500;
const int PWM_MIN = 500;
const int PWM_MAX = 1023;
bool subiendo = true;

// --- Parámetros de velocidad de rampa ---
const int PWM_STEP_LENTO = 1;   // incremento cuando el ángulo es pequeño
const int PWM_STEP_RAPIDO = 1;  // incremento cuando el ángulo es grande
int PWM_STEP_ACTUAL = PWM_STEP_RAPIDO;

const float ANGULO_UMBRAL = -10.0; // grados para cambiar la velocidad

// --- Tiempo fijo de actualización ---
const unsigned long T_STEP = 1000;  // ms entre pasos
unsigned long t_prev = 0;

void setup() {
  ledcSetup(PWM_CHANNEL, PWM_FREQ, PWM_RES);
  ledcAttachPin(PWM_PIN, PWM_CHANNEL);
  ledcWrite(PWM_CHANNEL, 0*1023);

  Serial.begin(115200);
  delay(3000);

  // --- I2C ---
  Wire.begin(SDA_PIN, SCL_PIN);
  Wire.setTimeout(50);
  Wire.setClock(400000);

  // --- AS5600 ---
  if (!encoder.begin()) Serial.println("❌ No se detecta AS5600");
  else Serial.println("✅ AS5600 detectado");

  Serial.println("✅ Sistema listo");
}

void loop() {
  unsigned long t_now = millis();

  // --- Leer ángulo ---
  uint16_t rawAngle = encoder.readAngle();
  float angulo = (rawAngle * 360.0f) / 4096.0f;
  angulo = 139.0 - angulo;
  // --- Seleccionar tamaño del paso según ángulo ---
  if (angulo < ANGULO_UMBRAL) {
    PWM_STEP_ACTUAL = PWM_STEP_RAPIDO ;
    //Serial.println("RAPIDO");
  } else {
    PWM_STEP_ACTUAL = PWM_STEP_LENTO;
    //Serial.println("LNETO");
  }

  // --- Actualizar rampa ---
  if (t_now - t_prev >= T_STEP) {
    t_prev = t_now;

    if (subiendo) {
      pwmValue += PWM_STEP_ACTUAL;
      if (pwmValue >= PWM_MAX) {
        pwmValue = PWM_MAX;
        subiendo = false;
      }
    } else {
      pwmValue -= PWM_STEP_ACTUAL;
      if (pwmValue <= PWM_MIN) {
        pwmValue = PWM_MIN;
        subiendo = true;
      }
    }

    ledcWrite(PWM_CHANNEL, pwmValue);

      // --- Imprimir datos para debug ---
  Serial.print(millis());
  Serial.print(",");
  Serial.print(pwmValue);
  Serial.print(",");
  Serial.print(angulo, 2);
  Serial.print(",");
  Serial.println(PWM_STEP_ACTUAL);
  }



  delay(20); // frecuencia de
}