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
  Identificar inicio do acionamento para iniciar leituras;
  Identificar queda no solo para cessar leituras;
  ...
}

Autores {
  Gabriel Rugeri;
  Henrique mago dos Bzoide;
}

Antares, 2024

Ultima modificacao: 01 05 2024, Rugeri
*/

#include <Wire.h>
#include <SD.h>
#include <Adafruit_BMP280.h>
#include <SPI.h>

//Enderecos internos MPU6050 (extraidos da lib)
#define ADDR_BMP 0x76
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
#define BAUD 9600

// Objeto arquivo.txtPCB_AVIONICA_PROTOTIPO
File dataFrame;

// Objeto BMP
Adafruit_BMP280 bmp;

//Contador de tempo
unsigned long tempoInicial;
unsigned long tempoAtual;

// Armazena dado da leitura atual
float altitude;
float altura;
float pressao;
float tempBMP;
float tempMPU;
float aceleracao[3];
float gyro[3];

// Auxiliares de calibracao
float calibraAltura = 0;
float calibraAcel[3] = {0, 0, 0};
float calibraGyro[3] = {0, 0, 0};


void imprime(const char *grandeza, float dado, const char *unidade, char end) {


  // Imprimir no monitor serial
  Serial.print(grandeza);
  Serial.print(" = ");
  Serial.print(dado);
  Serial.print(" ");
  Serial.println(unidade);

  // Escrever no arquivo
  dataFrame.print(dado); 
  dataFrame.print(" ");
  dataFrame.print(unidade);
  dataFrame.print(end);
}


void leituraMPU() {
  
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

  aceleracao[0] = Wire.read() << 8 | Wire.read();  //0x3B (ACCEL_XOUT_H) & 0x3C (ACCEL_XOUT_L)  
  aceleracao[1] = Wire.read() << 8 | Wire.read();  //0x3D (ACCEL_YOUT_H) & 0x3E (ACCEL_YOUT_L)  
  aceleracao[2] = Wire.read() << 8 | Wire.read();  //0x3F (ACCEL_ZOUT_H) & 0x40 (ACCEL_ZOUT_L) 

  aceleracao[0] /= ACCEL_SCALE;
  aceleracao[1] /= ACCEL_SCALE;
  aceleracao[2] /= ACCEL_SCALE; 

  tempMPU = Wire.read() << 8 | Wire.read();  //0x41 (TEMP_OUT_H) & 0x42 (TEMP_OUT_L)
  tempMPU = tempMPU / 340.00 + 36.53;
}


void calibraMPU() {
  Serial.println("Calibrando MPU6050...");
  Serial.println();
  for (int i = 0; i < 2000; i++) {
    leituraMPU();

    calibraAcel[0] += aceleracao[0];
    calibraAcel[1] += aceleracao[1];
    calibraAcel[2] += aceleracao[2];

    calibraGyro[0] += gyro[0];
    calibraGyro[1] += gyro[1];
    calibraGyro[2] += gyro[2];
  }

  calibraAcel[0] /= 2000;
  calibraAcel[1] /= 2000;
  calibraAcel[2] /= 2000;

  calibraGyro[0] /= 2000;
  calibraGyro[1] /= 2000;
  calibraGyro[2] /= 2000;

  Serial.println("MPU6050 calibrado. Calibracao:");
  imprime("Aceleracao X", calibraAcel[0], "m/s^2", '\n');
  imprime("Aceleracao Y", calibraAcel[1], "m/s^2", '\n');
  imprime("Aceleracao Z", calibraAcel[2], "m/s^2", '\n');
  imprime("Gyro X", calibraAcel[0], "rad/s", '\n');
  imprime("Gyro Y", calibraAcel[1], "rad/s", '\n');
  imprime("Gyro Z", calibraAcel[2], "rad/s", '\n');
  Serial.println();
}


void inicializaMPU() {
  Serial.println("Inicializando MPU6050...");

  pinMode(13, OUTPUT);
  digitalWrite(13, HIGH);

  Wire.setClock(400000);
  Wire.begin();
  delay(250);
  
  Wire.beginTransmission(ADDR_MPU);
  Wire.write(MPU6050_PWR_MGMT_1);
  Wire.write(0x00);

  Wire.endTransmission();

  calibraMPU();
}


void calibraBMP() {
  Serial.println("Calibrando BMP280...");
  Serial.println();
  for (int i = 0; i < 2000; i++) {
    calibraAltura += bmp.readAltitude(1013.25);
  }
  calibraAltura /= 2000;
  Serial.println("BMP280 calibrado. Calibracao:");
  imprime("Altitude", calibraAltura, "m", '\n');
  Serial.println();
}


void inicializaBMP() {
  Serial.println("Inicializando BMP280...");
  Serial.println();
  Wire.beginTransmission(ADDR_BMP);
  if (!bmp.begin()) {
    Serial.println("Falha ao inicializar BMP280.");
    while (1)
      delay(10);
  } else
    Serial.println("Sensor BMP280 inicializado com sucesso!");
  Serial.println();

    // Configuracoes padrao do datasheet
  bmp.setSampling(Adafruit_BMP280::MODE_NORMAL,     
                  Adafruit_BMP280::SAMPLING_X2,     
                  Adafruit_BMP280::SAMPLING_X16,    
                  Adafruit_BMP280::FILTER_X16,      
                  Adafruit_BMP280::STANDBY_MS_500); 

  calibraBMP();
}


void inicializaSD() {
  Serial.println("Inicializando leitor microSD...");
  Serial.println();
  if (!SD.begin(SD_CS_PIN)) {
    Serial.println("Falha ao inicializar leitor microSD");
    while (1)
      delay(10);
  } else
    Serial.println("Leitor microSD inicializado com sucesso.");
  Serial.println();
}


void inicializaSerialMonitor() {
  Serial.begin(BAUD);
  while (!Serial)
    delay(100);
  Serial.println("Monitor Serial inicializado com sucesso.");
  Serial.println();
}


void setup() {
  //pinMode(27, INPUT_PULLUP);

  // Inicializacao dos componentes
  inicializaSerialMonitor();
  inicializaBMP();
  inicializaMPU();
  inicializaSD();

  tempoInicial = millis();

  // esp_sleep_enable_ext0_wakeup(GPIO_NUM_27, 1);
  // Serial.println("Entrando modo Sleep");
  // esp_deep_sleep_start();
}


void loop() {

  dataFrame = SD.open("data.txt", FILE_WRITE);

   if (dataFrame) {
    Serial.println("Arquivo data.txt aberto. Escrevendo em data.txt...");
    Serial.println();

    // Contador de tempo
    tempoAtual = millis() - tempoInicial;
    imprime("Tempo", tempoAtual, "ms", '\t');

      /* Leitura do BMP */

    // Altitude
    altitude = bmp.readAltitude(1013.25); // Pressao padrao ao nivel de mar
    imprime("Altitude", altitude, "m", '\t');

    // Altura
    altura = altitude - calibraAltura;
    imprime("Altura", altura, "m", '\t');

    // Pressao atmosferica
    pressao = bmp.readPressure();
    imprime("Pressao", pressao, "Pa", '\t');

    // Temperatura BMP
    tempBMP = bmp.readTemperature();
    imprime("Temperatura (BMP)", tempBMP, "oC", '\t');

    delay(100);

      /* Leitura do MPU: aceleracao, rotacao e temperatura */
    leituraMPU();

    // Temperatura
    imprime("Temperatura (MPU)", tempMPU, "oC", '\t');

    // Aceleracao (x)
    aceleracao[0] -= calibraAcel[0];
    imprime("Aceleracao (x)", aceleracao[0], "m/s^2", '\t');

    // Aceleracao (y)
    aceleracao[1] -= calibraAcel[1];
    imprime("Aceleracao (y)", aceleracao[1], "m/s^2", '\t');

    // Aceleracao (z)
    aceleracao[2] -= calibraAcel[2];
    imprime("Aceleracao (z)", aceleracao[2], "m/s^2", '\t');

    // Gyro (x)
    gyro[0] -= calibraAcel[0];
    imprime("Gyro (x)", gyro[0], "rad/s", '\t');

    // Gyro (y)
    gyro[1] -= calibraAcel[1];
    imprime("Gyro (y)", gyro[1], "rad/s", '\t');

    // Gyro (z)
    gyro[2] -= calibraAcel[2];
    imprime("Gyro (z)", gyro[2], "rad/s", '\n'); //Considerar usar \r\n

    // Fecha arquivo dataframe.txt
    dataFrame.close();
    Serial.println("Arquivo data.txt fechado.");
    Serial.println();

  } else
    Serial.println("Erro ao abrir arquivo data.txt");  // } else
  
  delay(100);
}