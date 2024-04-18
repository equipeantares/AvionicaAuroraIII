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
    VCC - 5V;
    CS - ~10;
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

Ultima modificacao: 18 04 2024, Rugeri
*/

#include <SD.h> // Lib ja inclui Arduino.h
#include <Adafruit_MPU6050.h> // Lib ja inclui Wire.h
#include <Antares_BMP280.h>
#include <SPI.h>

// Objeto arquivo.txt
File dataFrame;

// Objeto BMP
Adafruit_BMP280 bmp;

// Objeto MPU
Adafruit_MPU6050 mpu;

// Armazenam leitura do MPU
sensors_event_t a, g, temp;

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
float calibraAltitude = 0;
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

void setup() {

  tempoInicial = millis();

  int baud = 9600;
  Serial.begin(baud);
  Serial.print("Iniciando serial: baud ");
  Serial.println(baud);
  while (!Serial)
    delay(100);

    /* Inicializacao BMP280 */
  Serial.println("Inicializando BMP280...");
  if (!bmp.begin()) {
    Serial.println("Falha ao inicializar BMP280.");
    while (1)
      delay(10);
  } else
    Serial.println("Sensor BMP280 inicializado com sucesso!");

  for (int i = 0; i < 2000; i++) {
    calibraAltitude += bmp.readAltitude(1013.25);
  }

  calibraAltitude /= 2000;


  /* Configuracoes padrao do datasheet */
  bmp.setSampling(Adafruit_BMP280::MODE_NORMAL,     
                  Adafruit_BMP280::SAMPLING_X2,     
                  Adafruit_BMP280::SAMPLING_X16,    
                  Adafruit_BMP280::FILTER_X16,      
                  Adafruit_BMP280::STANDBY_MS_500); 


    /* Inicializacao MPU6050 */
  Serial.println("Inicializando MPU6050...");
  if (!mpu.begin()) {
    Serial.println("Falha ao inicializar MPU6050.");
    while (1)
      delay(10);
  } else
    Serial.println("MPU6050 inicializado com sucesso!");
  
  /* Configuracoes de parametros MPU6050 */
  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);

  for (int i = 0; i < 2000; i++) {
    mpu.getEvent(&a, &g, &temp);

    calibraAcel[0] += a.acceleration.x;
    calibraAcel[1] += a.acceleration.y;
    calibraAcel[2] += a.acceleration.z;

    calibraGyro[0] += g.gyro.x;
    calibraGyro[1] += g.gyro.y;
    calibraGyro[2] += g.gyro.z;
  }

  calibraAcel[0] /= 2000;
  calibraAcel[1] /= 2000;
  calibraAcel[2] /= 2000;

  calibraGyro[0] /= 2000;
  calibraGyro[1] /= 2000;
  calibraGyro[2] /= 2000;

    /* Inicializacao Leitor microSD */
  Serial.println("Inicializando leitor microSD...");
  if (!SD.begin(4)) {
    Serial.println("Falha ao inicializar leitor microSD");
    while (1)
      delay(10);
  } else
    Serial.println("Leitor microSD inicializado com sucesso.");
}


void loop() {

  dataFrame = SD.open("dataframe.txt", FILE_WRITE);

  if (dataFrame) {
    Serial.println("Arquivo dataframe.txt aberto. Escrevendo em dataframe.txt...");

    // Contador de tempo
    tempoAtual = millis() - tempoInicial;
    imprime("Tempo", tempoAtual, "ms", '\t');

      /* Leitura do BMP */

    // Altitude
    altitude = bmp.readAltitude(1013.25); // Pressao padrao ao nivel de mar
    imprime("Altitude", altitude, "m", '\t');

    // Altura
    altura = altitude - calibraAltitude;
    imprime("Altura", altura, "m", '\t');

    // Pressao atmosferica
    pressao = bmp.readPressure();
    imprime("Pressao", pressao, "Pa", '\t');

    // Temperatura BMP
    tempBMP = bmp.readTemperature();
    imprime("Temperatura (BMP)", tempBMP, "oC", '\t');

    delay(100);

      /* Leitura do MPU: aceleracao, rotacao e temperatura */
    mpu.getEvent(&a, &g, &temp);

    // Temperatura
    tempMPU = temp.temperature;
    imprime("Temperatura (MPU)", tempMPU, "oC", '\t');

    // Aceleracao (x)
    aceleracao[0] = a.acceleration.x - calibraAcel[0];
    imprime("Aceleracao (x)", aceleracao[0], "m/s^2", '\t');

    // Aceleracao (y)
    aceleracao[1] = a.acceleration.y - calibraAcel[1];
    imprime("Aceleracao (y)", aceleracao[1], "m/s^2", '\t');

    // Aceleracao (z)
    aceleracao[2] = a.acceleration.z - calibraAcel[2];
    imprime("Aceleracao (z)", aceleracao[2], "m/s^2", '\t');

    // Gyro (x)
    gyro[0] = g.gyro.x - calibraAcel[0];
    imprime("Gyro (x)", gyro[0], "rad/s", '\t');

    // Gyro (y)
    gyro[1] = g.gyro.y - calibraAcel[1];
    imprime("Gyro (y)", gyro[1], "rad/s", '\t');

    // Gyro (z)
    gyro[2] = g.gyro.z - calibraAcel[2];
    imprime("Gyro (z)", gyro[2], "rad/s", '\n'); //Considerar usar \r\n

    // Fecha arquivo dataframe.txt
    dataFrame.close();
    Serial.println("Arquivo dataframe.txt fechado.");
    Serial.println();

  } else
    Serial.println("Erro ao abrir arquivo dataframe.txt");
  
  delay(100);
}