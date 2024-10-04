/*
Implementacao da rotina de execucao Avionica Aurora III

Conexoes de fiacao {
  BMP 280 {
    VCC - 3V3;
    GND - GND;
    SCL - A5;
    SDA - A4;
  }

  MPU 6050 {
    VCC - 3V3 ou 5V (se alimentacao USB);
    GND - GND;
    SCL - A5;
    SDA - A4;
  }

  Cartao microSD {
    VCC - 3V3 ou 5V;
    CS - ~10 (mas pode-se alterar para usar 4 ou 8);
    MOSI - ~11;
    CLK ou SCK - 13;
    MISO - 12;
    GND - GND;
  }
}

Pendencias {
  Sleep mode
  Implementar rotina de voo
}

Autores {
  Gabriel Rugeri;
  Henrique mago dos Bzoide;
}

Antares, 2024

Ultima modificacao: 27 09 2024, Henrique
*/

#include <Wire.h>
#include <SD.h>
#include <Adafruit_BMP280.h>
#include <SPI.h>

//Enderecos internos MPU6050 (extraidos da lib)
#define ADDR_MPU 0x68
#define MPU6050_CONFIG 0x1A      ///< General configuration register
#define MPU6050_GYRO_CONFIG 0x1B ///< Gyro specfic configuration register
#define MPU6050_ACCEL_CONFIG 0x1C ///< Accelerometer specific configration register
#define MPU6050_TEMP_H 0x41     ///< Temperature data high byte register
#define MPU6050_TEMP_L 0x42     ///< Temperature data low byte register
#define MPU6050_ACCEL_OUT 0x3B  ///< base address for sensor data reads
#define MPU6050_PWR_MGMT_1 0x6B        ///< Primary power/sleep control register
#define MPU6050_SELF_TEST_A 0x10 ///< Self test factory calibrated values register
#define ACCEL_SCALE 4096 // Fator de escala para MPU6050_RANGE_8_G
#define GYRO_SCALE 65.5 // Fator de escala para MPU6050_RANGE_500_DEG

#define SD_CS_PIN 5 // Pin onde se conecta pino CS do leitor SD

// Objeto arquivo.txt+
File dataFrame;

// Objeto BMP
Adafruit_BMP280 bmp;

//Contador de tempo
unsigned long tempoInicial;
unsigned long tempoAtual;

// Armazena dado da leitura atual
float altitude;
float altitude_anterior = 0;
float height;
float pressure;
float tempBMP;
float tempMPU;
float acceleration[3];
float gyro[3];
bool led_aceso = false;

// Auxiliares de calibracao
float calibrationHeight = 0;
float calibrationAccel[3] = {0, 0, 0};
float calibrationGyro[3] = {0, 0, 0};


void writes_on_SD(const char *grandeza, float dado, const char *unidade, char fim) {
  // Escrever no arquivo
  dataFrame.print(grandeza);
  dataFrame.print(dado); 
  dataFrame.print(" ");
  dataFrame.print(unidade);
  dataFrame.print(fim);
}


void reads_MPU() {
  
  Wire.beginTransmission(ADDR_MPU);
  Wire.write(MPU6050_CONFIG);
  Wire.write(0x05);
  Wire.endTransmission();

  // Leitura giroscopio
  Wire.beginTransmission(ADDR_MPU);
  Wire.write(MPU6050_GYRO_CONFIG);
  Wire.write(0x8);
  Wire.endTransmission();

  Wire.beginTransmission(ADDR_MPU);
  Wire.write(0x43);
  Wire.endTransmission();

  Wire.requestFrom(ADDR_MPU, 6, true);
  
  gyro[0] = Wire.read() << 8 | Wire.read();  //0x43 (GYRO_XOUT_H) & 0x44 (GYRO_XOUT_L)     
  gyro[1] = Wire.read() << 8 | Wire.read();  //0x45 (GYRO_YOUT_H) & 0x46 (GYRO_YOUT_L)
  gyro[2] = Wire.read() << 8 | Wire.read();  //0x47 (GYRO_ZOUT_H) & 0x48 (GYRO_ZOUT_L)

  gyro[0] /= GYRO_SCALE;
  gyro[1] /= GYRO_SCALE;
  gyro[2] /= GYRO_SCALE;

  // Leitura aceleracao e temperatura
  Wire.beginTransmission(ADDR_MPU);
  Wire.write(MPU6050_ACCEL_CONFIG);
  Wire.write(MPU6050_SELF_TEST_A);
  Wire.endTransmission();

  Wire.beginTransmission(ADDR_MPU);
  Wire.write(MPU6050_ACCEL_OUT);
  Wire.endTransmission();

  Wire.requestFrom(ADDR_MPU, 6, true);

  acceleration[0] = Wire.read() << 8 | Wire.read();  //0x3B (ACCEL_XOUT_H) & 0x3C (ACCEL_XOUT_L)  
  acceleration[1] = Wire.read() << 8 | Wire.read();  //0x3D (ACCEL_YOUT_H) & 0x3E (ACCEL_YOUT_L)  
  acceleration[2] = Wire.read() << 8 | Wire.read();  //0x3F (ACCEL_ZOUT_H) & 0x40 (ACCEL_ZOUT_L) 

  acceleration[0] /= ACCEL_SCALE;
  acceleration[1] /= ACCEL_SCALE;
  acceleration[2] /= ACCEL_SCALE; 

  tempMPU = Wire.read() << 8 | Wire.read();  //0x41 (TEMP_OUT_H) & 0x42 (TEMP_OUT_L)
  tempMPU = tempMPU / 340.00 + 36.53;
}


void calibrates_MPU() {
  Serial.println("Calibrando MPU6050...");
  for (int i = 0; i < 2000; i++) {
    reads_MPU();

    calibrationAccel[0] += acceleration[0];
    calibrationAccel[1] += acceleration[1];
    calibrationAccel[2] += acceleration[2];

    calibrationGyro[0] += gyro[0];
    calibrationGyro[1] += gyro[1];
    calibrationGyro[2] += gyro[2];
  }

  calibrationAccel[0] /= 2000;
  calibrationAccel[1] /= 2000;
  calibrationAccel[2] /= 2000;

  calibrationGyro[0] /= 2000;
  calibrationGyro[1] /= 2000;
  calibrationGyro[2] /= 2000;

  Serial.println("MPU6050 calibrado.");
  writes_on_SD("Aceleracao X", calibrationAccel[0], "m/s^2", '\n');
  writes_on_SD("Aceleracao Y", calibrationAccel[1], "m/s^2", '\n');
  writes_on_SD("Aceleracao Z", calibrationAccel[2], "m/s^2", '\n');
  writes_on_SD("Gyro X", calibrationAccel[0], "rad/s", '\n');
  writes_on_SD("Gyro Y", calibrationAccel[1], "rad/s", '\n');
  writes_on_SD("Gyro Z", calibrationAccel[2], "rad/s", '\n');
}


void starts_up_MPU() {
  Serial.println("Inicializando MPU6050...");

  pinMode(21, OUTPUT);
  digitalWrite(21, HIGH);

  Wire.setClock(400000);
  Wire.begin();
  delay(250);
  
  Wire.beginTransmission(ADDR_MPU);
  Wire.write(MPU6050_PWR_MGMT_1);
  Wire.write(0x00);

  Wire.endTransmission();

  calibrates_MPU();
}


void calibrates_BMP() {
  for (int i = 0; i < 2000; i++) {
    calibrationHeight += bmp.readAltitude(1013.25);
  }
  calibrationHeight /= 2000;
  Serial.println("BMP280 calibrado.");
  writes_on_SD("Altitude", calibrationHeight, "m", '\n');
}


void starts_up_BMP() {
  if (!bmp.begin()) {
    Serial.println("Falha ao inicializar BMP280.");
    while (1)
      delay(10);
  } else
    Serial.println("Sensor BMP280 inicializado com sucesso!");

    // Configuracoes padrao do datasheet
  bmp.setSampling(Adafruit_BMP280::MODE_NORMAL,     
                  Adafruit_BMP280::SAMPLING_X2,     
                  Adafruit_BMP280::SAMPLING_X16,    
                  Adafruit_BMP280::FILTER_X16,      
                  Adafruit_BMP280::STANDBY_MS_500); 

  calibrates_BMP();
}


void starts_up_SD() {
  if (!SD.begin(SD_CS_PIN)) {
    Serial.println("Falha ao inicializar leitor microSD");
    while (1)
      delay(10);
  } else
    Serial.println("Leitor microSD inicializado com sucesso.");
}


void starts_up_serial(int baud) {
  Serial.begin(baud);
  while (!Serial)
    delay(100);
  Serial.println("Monitor Serial inicializado com sucesso.");
}


void setup() {
  pinMode(26, OUTPUT);
  
  tempoInicial = millis();

  // Inicializacao dos componentes
  starts_up_serial(9600);
  starts_up_BMP();
  starts_up_MPU();
  starts_up_SD();
}


void loop() {

  dataFrame = SD.open("dados.txt", FILE_WRITE);

  if (dataFrame) {
    Serial.println("Arquivo dataframe.txt aberto. Escrevendo em dataframe.txt...");

    // Contador de tempo
    tempoAtual = millis() - tempoInicial;
    writes_on_SD("Tempo", tempoAtual, "ms", '\t');

      /* Leitura do BMP */

    // Altitude
    altitude = bmp.readAltitude(1013.25); // Pressao padrao ao nivel de mar
    writes_on_SD("Altitude", altitude, "m", '\t');

    // Altura
    height = altitude - calibrationHeight;
    writes_on_SD("Altura", height, "m", '\t');

    // Pressao atmosferica
    pressure = bmp.readPressure();
    writes_on_SD("Pressao", pressure, "Pa", '\t');

    // Temperatura BMP
    tempBMP = bmp.readTemperature();
    writes_on_SD("Temperatura (BMP)", tempBMP, "oC", '\t');

    delay(100);

      /* Leitura do MPU: aceleracao, rotacao e temperatura */
    reads_MPU();

    // Temperatura
    writes_on_SD("Temperatura (MPU)", tempMPU, "oC", '\t');

    // Aceleracao (x)
    acceleration[0] -= calibrationAccel[0];
    writes_on_SD("Aceleracao (x)", acceleration[0], "m/s^2", '\t');

    // Aceleracao (y)
    acceleration[1] -= calibrationAccel[1];
    writes_on_SD("Aceleracao (y)", acceleration[1], "m/s^2", '\t');

    // Aceleracao (z)
    acceleration[2] -= calibrationAccel[2];
    writes_on_SD("Aceleracao (z)", acceleration[2], "m/s^2", '\t');

    // Gyro (x)
    gyro[0] -= calibrationAccel[0];
    writes_on_SD("Gyro (x)", gyro[0], "rad/s", '\t');

    // Gyro (y)
    gyro[1] -= calibrationAccel[1];
    writes_on_SD("Gyro (y)", gyro[1], "rad/s", '\t');

    // Gyro (z)
    gyro[2] -= calibrationAccel[2];
    writes_on_SD("Gyro (z)", gyro[2], "rad/s", '\n'); //Considerar usar \r\n

    // Fecha arquivo dataframe.txt
    dataFrame.close();
  } else
    Serial.println("Erro ao abrir arquivo dataframe.txt");
  
  if(!led_aceso) {
    if(altitude < altitude_anterior) {
      digitalWrite(26, HIGH);
      led_aceso = true;
    }
    else {
      altitude_anterior = altitude;
    }
  }
  
  delay(100);
}
