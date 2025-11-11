#include <Arduino.h>
#include <Wire.h>
#include <AS5600.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>

#define PWM_PIN   18   // PWM al TB6612
#define SDA_PIN   21   // SDA I2C
#define SCL_PIN   22   // SCL I2C
#define BTN1_PIN  19   // Botón bajar PWM
#define BTN2_PIN  23   // Botón subir PWM

AS5600 encoder;
Adafruit_MPU6050 mpu;

// Configuración PWM
const int PWM_CHANNEL = 1;
const int PWM_FREQ = 20000;  // 20 kHz
const int PWM_RES  = 10;      // 8 bits (0–1023)
const int PWM_CYCLES = 3;  // número de ciclos por valor

long t_prev = 0;
bool activar = 0;
bool next_step = 0;

uint16_t pwmValue = 850;
const uint16_t PWM_STEP = 1; // Incremento/decremento por pulsación
int cicloCount = 0;    

void setup() {
  Serial.begin(115200);
  delay(1000);

  // --- PWM ---
  ledcSetup(PWM_CHANNEL, PWM_FREQ, PWM_RES);
  ledcAttachPin(PWM_PIN, PWM_CHANNEL);
  ledcWrite(PWM_CHANNEL, pwmValue);

  // --- I2C ---
  Wire.begin(SDA_PIN, SCL_PIN);
  Wire.setTimeout(50); // 50 ms de timeout en operaciones I2C
  Wire.setClock(400000); // 40 kHz I2C

  // --- AS5600 ---
  if (!encoder.begin()) {
    Serial.println("❌ No se detecta el AS5600");
  } else {
    Serial.println("✅ AS5600 detectado correctamente");
  }

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

  // --- Botones ---
  pinMode(BTN1_PIN, INPUT_PULLUP);
  pinMode(BTN2_PIN, INPUT_PULLUP);

  Serial.println("✅ Sistema inicializado correctamente");
    delay(5000);
}

void loop() {
  // --- Leer ángulo del AS5600  ---
 
  uint16_t rawAngle = encoder.readAngle(); // valor 0–4095 
  float angulo = (rawAngle * 360.0) / 4096.0; // conversión a grados



  // --- Leer datos del MPU6050 ---
  sensors_event_t accel, gyro, temp;
  mpu.getEvent(&accel, &gyro, &temp);

  // --- Leer botones ---
  bool btnBajar = (digitalRead(BTN1_PIN) == LOW);
  bool btnSubir = (digitalRead(BTN2_PIN) == LOW);
  
  // cada 10 segundos alterna entre encendido y apagado
  if (millis() - t_prev >= 12000) {
    t_prev = millis();
    activar = !activar;

    // si termina un ciclo completo (apagado → encendido)
    if (activar) {
      cicloCount++;

      // después de 10 ciclos, aumenta el PWM
      if (cicloCount >= PWM_CYCLES) {
        cicloCount = 0;
        pwmValue += PWM_STEP;
        if (pwmValue > 480) pwmValue = 480;
      }
    }
  }

  // aplicar PWM
  if (activar) {
    ledcWrite(PWM_CHANNEL, pwmValue);
  } else {
    ledcWrite(PWM_CHANNEL, 0);
  }

  /*
  
  // --- Control PWM mejorado ---
  if (btnBajar && pwmValue > 0) {
    if (pwmValue >= PWM_STEP) pwmValue -= PWM_STEP;
    else pwmValue = 0;
    ledcWrite(PWM_CHANNEL, pwmValue);
    //Serial.printf("⬇️ PWM bajó a: %d\n", pwmValue);
    //delay(150); // anti-rebote
  }

  if (btnSubir && pwmValue < 1024) {
    if (pwmValue <= 1024 - PWM_STEP) pwmValue += PWM_STEP;
    else pwmValue = 1024;
    ledcWrite(PWM_CHANNEL, pwmValue);
    //Serial.printf("⬆️ PWM subió a: %d\n", pwmValue);
    //delay(150); // anti-rebote
  }
*/
  // --- Imprimir datos ---
  Serial.print(" time: ");
  Serial.print(millis());
  Serial.print(" Angle: ");
  Serial.print(",");
  Serial.print(angulo, 2);
  Serial.print("° | Giro (°/s): X=");
  Serial.print(gyro.gyro.x, 2);
  Serial.print(" Y=");
  Serial.print(gyro.gyro.y, 2);
  Serial.print(" Z=");
  Serial.print(gyro.gyro.z, 2);
  Serial.print("° | PWM=");
  Serial.print(",");
  Serial.print(activar);
  Serial.print(",");
  Serial.println(pwmValue);

  delay(200);
}
