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

// --- Valores definidos por el usuario ---
int pwmVals[] = {400, 500, 600};   // <- define tus tres valores aquí
const int numVals = sizeof(pwmVals) / sizeof(pwmVals[0]);

// --- Tiempos ---
const unsigned long T_STEP_PWM = 1000;   // ms entre cambios de PWM
const unsigned long T_STEP_SENSORES = 20; // ms entre lecturas de sensores

unsigned long t_prev_pwm = 0;
unsigned long t_prev_sens = 0;

// --- Índices de las combinaciones ---
int i = 0, j = 0, k = 0;

// --- Variables de lectura ---
float angulo = 0.0;
int pwmActual = 0;

void setup() {
  Serial.begin(115200);
  delay(3000);
  Serial.println("🚀 Iniciando sistema...");

  // --- PWM ---
  ledcSetup(PWM_CHANNEL, PWM_FREQ, PWM_RES);
  ledcAttachPin(PWM_PIN, PWM_CHANNEL);
  ledcWrite(PWM_CHANNEL, 0);

  // --- I2C ---
  Wire.begin(SDA_PIN, SCL_PIN);
  Wire.setTimeout(50);
  Wire.setClock(400000);

  // --- AS5600 ---
  if (!encoder.begin()) Serial.println("❌ No se detecta AS5600");
  else Serial.println("✅ AS5600 detectado");

  // --- MPU6050 ---
  if (!mpu.begin()) Serial.println("⚠️ No se detecta MPU6050");
  else Serial.println("✅ MPU6050 detectado");

  Serial.println("✅ Sistema listo");
}

void loop() {
  unsigned long t_now = millis();

  // --- Lectura de sensores cada 20 ms ---
  if (t_now - t_prev_sens >= T_STEP_SENSORES) {
    t_prev_sens = t_now;

    // Leer ángulo del AS5600
    uint16_t rawAngle = encoder.readAngle();
    angulo = (rawAngle * 360.0f) / 4096.0f;

    // Leer acelerómetro (opcional)
    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);

    // Imprimir lecturas y PWM actual
    Serial.print("PWM actual: ");
    Serial.print(pwmActual);
    Serial.print(" | Ángulo: ");
    Serial.print(angulo, 2);
    Serial.print(" | Aceleración Z (m/s²): ");
    Serial.println(a.acceleration.z, 2);
  }

  // --- Cambio de PWM cada 1 s ---
  if (t_now - t_prev_pwm >= T_STEP_PWM) {
    t_prev_pwm = t_now;

    // Generar combinación actual
    int val1 = pwmVals[i];
    int val2 = pwmVals[j];
    int val3 = pwmVals[k];

    // Aplicar PWM (puedes cambiar qué valor usar)
    pwmActual = val1;
    ledcWrite(PWM_CHANNEL, pwmActual);

    // Imprimir combinación y PWM
    Serial.print("🧩 Combinación: ");
    Serial.print(val1); Serial.print(", ");
    Serial.print(val2); Serial.print(", ");
    Serial.print(val3);
    Serial.print(" | PWM aplicado: ");
    Serial.println(pwmActual);

    // Avanzar combinación
    k++;
    if (k >= numVals) {
      k = 0;
      j++;
      if (j >= numVals) {
        j = 0;
        i++;
        if (i >= numVals) {
          i = 0;
          Serial.println("🔁 Ciclo completo, reiniciando combinaciones...");
        }
      }
    }
  }
}