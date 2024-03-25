#include <Adafruit_MPU6050.h>  
#include <Adafruit_BMP280.h>   
#include <SPI.h>
#include <SD.h>

// Objeto arquivo.txt
File myFile; 

// Objeto BMP
Adafruit_BMP280 bmp;

// Objeto MPU
Adafruit_MPU6050 mpu;

// Armazenam leitura do MPU
sensors_event_t a, g, temp;

void setup() {
  Serial.begin(115200);
  Serial.println("Iniciando serial");

  while (!Serial) {
    delay(100); // Small delay for Serial to initialize 
  }

  /* Inicializacao BMP280 */
  Serial.println("Inicializando BMP280...");  
  bmp.begin(0x76);

  /* Configuracoes padrao do datasheet */
  bmp.setSampling(Adafruit_BMP280::MODE_NORMAL,     
                  Adafruit_BMP280::SAMPLING_X2,     
                  Adafruit_BMP280::SAMPLING_X16,   
                  Adafruit_BMP280::FILTER_X16,      
                  Adafruit_BMP280::STANDBY_MS_500); 

  /* Inicializacao MPU6050 */
  Serial.println("Inicializando MPU6050..."); 
  mpu.begin();

  /* Configuracoes de parametros MPU6050 */
  mpu.setAccelerometerRange(MPU6050_RANGE_8_G); 
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);    
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ); 


}

void loop() {
  Serial.begin(115200); // Initialize serial for the loop

  // Leitura do BMP
  Serial.print(F("Altitude aproximada = "));
  Serial.print(bmp.readAltitude(1013.25));  
  Serial.println(" m");
  delay(2000);

  // Leitura do MPU: aceleracao, rotacao e temperatura 
  mpu.getEvent(&a, &g, &temp);

  // Imprime medidas de aceleracao
  Serial.print("Aceleracao X: ");
  Serial.print(a.acceleration.x);
  Serial.print(", Y: ");
  Serial.print(a.acceleration.y);
  Serial.print(", Z: ");
  Serial.print(a.acceleration.z);
  Serial.println(" m/s^2");

  // Imprime medidas de rotacao (velocidade angular)
  Serial.print("Rotacao X: ");
  Serial.print(g.gyro.x);
  Serial.print(", Y: ");
  Serial.print(g.gyro.y);
  Serial.print(", Z: ");
  Serial.print(g.gyro.z);
  Serial.println(" rad/s");


}
