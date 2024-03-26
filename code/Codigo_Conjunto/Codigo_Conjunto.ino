/*
Implementacao da rotina de execucao Avionica Aurora III

Pendencias:
Identificar inicio do acionamento para iniciar leituras
Identificar queda no solo para {
  cessar leituras
  fechar arquivo.txt (a ideia eh abrir abrir ao iniciar e fechar somente após identif. queda)
} 
...

Autores:
Gabriel Rugeri
Henrique mago dos Bzoide

Antares, 2024

Ultima modificacao: 25 03 2024, Rugeri

*/

#include <Adafruit_MPU6050.h> // Lib ja inclui Wire.h
#include <Antares_BMP280.h> // Lib ja inclui Adafruit_Sensor.h
#include <SPI.h>
#include <SD.h>

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
float calibraPressao = 0;
float calibraTempBMP = 0;
float calibraTempMPU = 0;
float calibraAcel[3] = [0, 0, 0];
float calibraGyro[3] = [0, 0, 0];

void imprime(const char *medida, const char *dado, const char *unidade, char end) {

  // Imprimir no monitor serial
  Serial.print(medida);
  Serial.print(" = ");
  Serial.print(dado);
  Serial.print(" ");
  Serial.println(unidade);

  // Escrever no arquivo
  dataFrame.print(dado); 
  dataFrame.print(" ");
  dataFrame.print(unidade);
  dataFrame.print(end);
  
  // Verificar se houve algum erro na escrita do arquivo
  if (dataFrame.fail())
    Serial.println("Erro ao escrever no arquivo!");
}

void setup() {

  // Armazena o tempo em que o arduino primeiro registra
  tempoInicial = millis();

    /* Inicializa monitor serial */
  int baud = 9600;
  Serial.begin(baud);
  Serial.println("Iniciando serial: baud " + String(baud));

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
    //calibraPressao += bmp.readPressure();
    //calibraTempBMP += bmp.readTemperature();
  }

  calibraAltitude /= 2000;
  //calibraPressao /= 2000;
  //calibraTempBMP /= 2000;


  /* Configuracoes padrao do datasheet */
  bmp.setSampling(Adafruit_BMP280::MODE_NORMAL,     /* Operating Mode. */
                  Adafruit_BMP280::SAMPLING_X2,     /* Temp. oversampling */
                  Adafruit_BMP280::SAMPLING_X16,    /* Pressure oversampling */
                  Adafruit_BMP280::FILTER_X16,      /* Filtering. */
                  Adafruit_BMP280::STANDBY_MS_500); /* Standby time. */


    /* Inicializacao MPU6050 */
  Serial.println("Inicializando MPU6050...");
  if (!mpu.begin()) {
    Serial.println("Falha ao inicializar MPU6050.");
    while (1)
      delay(10);
  } else
    Serial.println("MPU6050 inicializado com sucesso!");
  
  /* Configuracoes de parametros MPU6050 */
  mpu.setAccelerometerRange(MPU6050_RANGE_8_G); // Range de acelerometro == 8g
  mpu.setGyroRange(MPU6050_RANGE_500_DEG); // Range de giroscopio == 500 graus/s
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ); // Range de filtro == 21 Hz

  for (int i = 0; i < 2000; i++) {
    mpu.getEvent(&a, &g, &temp);

    calibraAcel[0] += a.acceleration.x;
    calibraAcel[1] += a.acceleration.y;
    calibraAcel[2] += a.acceleration.z;

    calibraGyro[0] += g.gyro.x;
    calibraGyro[1] += g.gyro.y;
    calibraGyro[2] += g.gyro.z;

    //calibraTempMPU += temp.temperature;
  }

  calibraAcel[0] /= 2000;
  calibraAcel[1] /= 2000;
  calibraAcel[2] /= 2000;

  calibraGyro[0] /= 2000;
  calibraGyro[1] /= 2000;
  calibraGyro[2] /= 2000;

  //calibraTempMPU /= 2000;

    /* Inicializacao Leitor microSD */
  Serial.println("Inicializando leitor microSD...");
  if (!SD.begin(4)) {
    Serial.println("Falha ao inicializar leitor microSD");
    while (1)
      delay(10);
  } else
    Serial.println("Leitor microSD inicializado com sucesso.");

  /* Só é possível ter um arquivo aberto por vez. Fechar antes de abrir outro */
  // Parâmetros: <nome do arquivo>.txt, FILE_WRITE (permite ler e escrever no arquivo)
  // Se arquivo não existir, cria um novo automaticamente
  dataFrame = SD.open("dataframe.txt", FILE_WRITE);

  // Se arquivo abriu com sucesso, pode-se-lhe escrever:
  if (dataFrame) {
    Serial.println("Arquivo dataframe.txt aberto. Escrevendo em dataframe.txt...");

    /* As duas formas de escrever são:
    dataFrame.println("testing 1, 2, 3.");
    dataFrame.write("testando 1,2,3."); */

    // read from the file until there's nothing else in it:
    while (dataFrame.available()) {
      Serial.write(dataFrame.read());
    }


    // As duas formas de fechar o arquivo são:
    dataFrame.close();
    //SD.close("<nome do arquivo>.txt");

    Serial.println("Arquivo test.txt fechado.");
  } else {
    // if the file didn't open, print an error:
    Serial.println("Erro ao abrir arquivo test.txt");
  }

  while (!dataFrame)
    delay(1);

}


void loop() {

  // Contador de tempo
  tempoAtual = millis() - tempoInicial;
  imprime("Tempo", String(tempoAtual), "ms", '\t');

    /* Leitura do BMP */

  // Altitude
  altitude = bmp.readAltitude(1013.25); // Pressao padrao ao nivel de mar
  imprime("Altitude", String(altitude), "m", '\t');

  // Altura
  altura = altitude - calibraAltitude;
  imprime("Altura", String(altura), "m", '\t');

  // Pressao atmosferica
  pressao = bmp.readPressure();
  imprime("Pressao", String(pressao), "Pa", '\t');

  // Temperatura BMP
  tempBMP = bmp.readTemperature();
  imprime("Temperatura (BMP)", String(tempBMP), "oC", '\t');

  delay(100);

    /* Leitura do MPU: aceleracao, rotacao e temperatura */
  mpu.getEvent(&a, &g, &temp);

  // Temperatura
  tempMPU = temp.temperature;
  imprime("Temperatura (MPU)", String(tempMPU), "oC", '\t');

  // Aceleracao (x)
  aceleracao[0] = a.acceleration.x - calibraAcel[0];
  imprime("Aceleracao (x)", String(aceleracao[0]), "m/s^2", '\t');

  // Aceleracao (y)
  aceleracao[1] = a.acceleration.y - calibraAcel[1];
  imprime("Aceleracao (y)", String(aceleracao[1]), "m/s^2", '\t');

  // Aceleracao (z)
  aceleracao[2] = a.acceleration.z - calibraAcel[2];
  imprime("Aceleracao (z)", String(aceleracao[2]), "m/s^2", '\t');

  // Gyro (x)
  gyro[0] = g.gyro.x - calibraAcel[0];
  imprime("Gyro (x)", String(gyro[0]), "rad/s", '\t');

  // Gyro (y)
  gyro[1] = g.gyro.y - calibraAcel[1];
  imprime("Gyro (y)", String(gyro[1]), "rad/s", '\t');

  // Gyro (z)
  gyro[2] = g.gyro.z - calibraAcel[2];
  imprime("Gyro (z)", String(gyro[2]), "rad/s", '\n'); //Considerar usar \r\n

  Serial.println();
  
  delay(100);
}