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

// Endereços internos MPU6050 (extraídos da lib)
#define ADDR_BMP 0x76
#define ADDR_MPU 0x68
#define MPU6050_CONFIG 0x1A
#define MPU6050_GYRO_CONFIG 0x1B
#define MPU6050_ACCEL_CONFIG 0x1C
#define MPU6050_TEMP_H 0x41
#define MPU6050_TEMP_L 0x42
#define MPU6050_ACCEL_OUT 0x3B
#define MPU6050_PWR_MGMT_1 0x6B
#define MPU6050_SELF_TEST_A 0x10
#define ACCEL_SCALE 4096
#define GYRO_SCALE 65.5

// Pinagens atualizadas para o microSD
#define SD_CS_PIN 5   // D5 -> CS
#define SD_MOSI_PIN 23 // D23 -> MOSI
#define SD_CLK_PIN 19  // D19 -> CLK
#define SD_MISO_PIN 18 // D18 -> MISO
#define BAUD 9600

// SPI object
SPIClass spiSD(VSPI);  // VSPI is the default SPI bus on ESP32

// Objeto arquivo.txtPCB_AVIONICA_PROTOTIPO
File dataFrame;

// Objeto BMP
Adafruit_BMP280 bmp;

// Contador de tempo
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

// Auxiliares de calibração
float calibraAltura = 0;
float calibraAcel[3] = {0, 0, 0};
float calibraGyro[3] = {0, 0, 0};

void imprime(const char *grandeza, float dado, const char *unidade, char end) {
  Serial.print(grandeza);
  Serial.print(" = ");
  Serial.print(dado);
  Serial.print(" ");
  Serial.println(unidade);

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

  // Leitura giroscópio
  Wire.beginTransmission(ADDR_MPU);
  Wire.write(MPU6050_GYRO_CONFIG);
  Wire.write(0x8);
  Wire.endTransmission();

  Wire.beginTransmission(ADDR_MPU);
  Wire.write(0x43);
  Wire.endTransmission();

  Wire.requestFrom(ADDR_MPU, 6, true);

  gyro[0] = Wire.read() << 8 | Wire.read();
  gyro[1] = Wire.read() << 8 | Wire.read();
  gyro[2] = Wire.read() << 8 | Wire.read();

  gyro[0] /= GYRO_SCALE;
  gyro[1] /= GYRO_SCALE;
  gyro[2] /= GYRO_SCALE;

  // Leitura aceleração e temperatura
  Wire.beginTransmission(ADDR_MPU);
  Wire.write(MPU6050_ACCEL_CONFIG);
  Wire.write(MPU6050_SELF_TEST_A);
  Wire.endTransmission();

  Wire.beginTransmission(ADDR_MPU);
  Wire.write(MPU6050_ACCEL_OUT);
  Wire.endTransmission();

  Wire.requestFrom(ADDR_MPU, 6, true);

  aceleracao[0] = Wire.read() << 8 | Wire.read();
  aceleracao[1] = Wire.read() << 8 | Wire.read();
  aceleracao[2] = Wire.read() << 8 | Wire.read();

  aceleracao[0] /= ACCEL_SCALE;
  aceleracao[1] /= ACCEL_SCALE;
  aceleracao[2] /= ACCEL_SCALE;

  tempMPU = Wire.read() << 8 | Wire.read();
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

  Serial.println("MPU6050 calibrado. Calibração:");
  imprime("Aceleração X", calibraAcel[0], "m/s^2", '\n');
  imprime("Aceleração Y", calibraAcel[1], "m/s^2", '\n');
  imprime("Aceleração Z", calibraAcel[2], "m/s^2", '\n');
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
  Serial.println("BMP280 calibrado. Calibração:");
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

  // Configurações padrão do datasheet
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

  // Configurando os pinos SPI manualmente
  spiSD.begin(SD_CLK_PIN, SD_MISO_PIN, SD_MOSI_PIN, SD_CS_PIN);

  // Inicializa o SD card usando o SPI personalizado
  if (!SD.begin(SD_CS_PIN, spiSD)) {
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
  // Inicialização dos componentes
  inicializaSerialMonitor();
  inicializaBMP();
  inicializaMPU();
  inicializaSD();

  tempoInicial = millis();
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
    altitude = bmp.readAltitude(1013.25);  // Pressão padrão ao nível do mar
    imprime("Altitude", altitude, "m", '\t');

    // Altura
    altura = altitude - calibraAltura;
    imprime("Altura", altura, "m", '\t');

    // Pressão atmosférica
    pressao = bmp.readPressure();
    imprime("Pressão", pressao, "Pa", '\t');

    // Temperatura BMP
    tempBMP = bmp.readTemperature();
    imprime("Temperatura (BMP)", tempBMP, "°C", '\t');

    delay(100);

    /* Leitura do MPU: aceleração, rotação e temperatura */
    leituraMPU();

    // Temperatura
    imprime("Temperatura (MPU)", tempMPU, "°C", '\t');

    // Aceleração (x)
    aceleracao[0] -= calibraAcel[0];
    imprime("Aceleração (x)", aceleracao[0], "m/s^2", '\t');

    // Aceleração (y)
    aceleracao[1] -= calibraAcel[1];
    imprime("Aceleração (y)", aceleracao[1], "m/s^2", '\t');

    // Aceleração (z)
    aceleracao[2] -= calibraAcel[2];
    imprime("Aceleração (z)", aceleracao[2], "m/s^2", '\t');

    // Gyro (x)
    gyro[0] -= calibraAcel[0];
    imprime("Gyro (x)", gyro[0], "rad/s", '\t');

    // Gyro (y)
    gyro[1] -= calibraAcel[1];
    imprime("Gyro (y)", gyro[1], "rad/s", '\t');

    // Gyro (z)
    gyro[2] -= calibraAcel[2];
    imprime("Gyro (z)", gyro[2], "rad/s", '\n');  // Considerar usar \r\n

    // Fecha arquivo dataframe.txt
    dataFrame.close();
    Serial.println("Arquivo data.txt fechado.");
    Serial.println();

  } else {
    Serial.println("Erro ao abrir arquivo data.txt");
  }

  delay(100);
}
