#include <Adafruit_MPU6050.h> // Lib ja inclui Wire.h
#include <Antares_BMP280.h> // Lib ja inclui Adafruit_Sensor.h
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
  Serial.begin(9600);
  Serial.println("Iniciando serial");

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

  // Leitura do BMP
  Serial.print(F("Altitude aproximada = "));
  Serial.print(bmp.readAltitude(1013.25)); /* Pressao padrao ao nivel de mar */
  Serial.println(" m");

  delay(50);

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




  /* Só é possível ter um arquivo aberto por vez. Fechar antes de abrir outro */
  // Parâmetros: <nome do arquivo>.txt, FILE_WRITE (permite ler e escrever no arquivo)
  // Se arquivo não existir, cria um novo automaticamente
  myFile = SD.open("test.txt", FILE_WRITE);

  // Se arquivo abriu com sucesso, pode-se-lhe escrever:
  if (myFile) {
    Serial.println("Arquivo test.txt aberto. Escrevendo em test.txt...");

    // As duas formas de escrever são:
    myFile.println("testing 1, 2, 3.");
    myFile.write("testando 1,2,3.");

    // read from the file until there's nothing else in it:
    while (myFile.available()) {
      Serial.write(myFile.read());
    }


    // As duas formas de fechar o arquivo são:
    myFile.close();
    //SD.close("<nome do arquivo>.txt");

    Serial.println("Arquivo test.txt fechado.");
  } else {
    // if the file didn't open, print an error:
    Serial.println("Erro ao abrir arquivo test.txt");
  }

  Serial.println();
  
  delay(50);
}