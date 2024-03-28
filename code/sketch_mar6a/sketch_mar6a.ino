#include <Wire.h>
#include <Adafruit_BMP280.h>
#include <SPI.h>
#include <SD.h>

Adafruit_BMP280 bmp; // I2C

//Endereço em hexadecimal do sensor MPU 6050
const int ENDERECO_SENSOR=0x68;  

File dataFrame;

float altitude;
float altitudeAnterior;
float altura;
float pressao;
float tempBMP;

float acelX, acelY, acelZ;
float temperatura
int16_t girX, girY, girZ;
float eixoX, eixoY, eixoZ;
float eixoXCalibre, eixoYCalibre, eixoZCalibre;
int NumeroDeCalibre;

void gyro(void)
{
  
  Wire.beginTransmission(ENDERECO_SENSOR);
  Wire.write(0x1A);
  Wire.write(0x05);
  Wire.endTransmission();

  Wire.beginTransmission(ENDERECO_SENSOR);
  Wire.write(0x1B);
  Wire.write(0x8);
  Wire.endTransmission();

  Wire.beginTransmission(ENDERECO_SENSOR);
  Wire.write(0x43);
  Wire.endTransmission(false);

  Wire.requestFrom(0x68, 6, true);
  
  girX = Wire.read()<<8|Wire.read();  //0x43 (GYRO_XOUT_H) & 0x44 (GYRO_XOUT_L)     
  girY = Wire.read()<<8|Wire.read();  //0x45 (GYRO_YOUT_H) & 0x46 (GYRO_YOUT_L)
  girZ = Wire.read()<<8|Wire.read();  //0x47 (GYRO_ZOUT_H) & 0x48 (GYRO_ZOUT_L)

  eixoX = (float)girX/65.6;
  eixoY = (float)girY/65.6;
  eixoZ = (float)girZ/65.6;

}

void setup() {

//MPU
  Serial.begin(57600);
  pinMode(13, OUTPUT);
  digitalWrite(13, HIGH);

  Wire.setClock(400000);
  Wire.begin();
  delay(250);
  
  Wire.beginTransmission(0x68);
  Wire.write(0x6B);
  Wire.write(0x00);

  Wire.endTransmission(true);
  
  for (NumeroDeCalibre = 0;  NumeroDeCalibre < 2000; NumeroDeCalibre++) {
      gyro();
      eixoXCalibre += eixoX;
      eixoYCalibre += eixoY;
      eixoZCalibre += eixoZ;
      delay(1);
    }
    eixoXCalibre/=2000;
    eixoYCalibre/=2000;
    eixoZCalibre/=2000; 

//BMP
  Serial.begin(9600);

  while ( !Serial ) delay(100);   // wait for native usb
  Serial.println(F("BMP280 test"));

  unsigned status;
  status = bmp.begin();

  if (!status) {
    Serial.println(F("Could not find a valid BMP280 sensor, check wiring or "
                      "try a different address!"));
    Serial.print("SensorID was: 0x"); Serial.println(bmp.sensorID(),16);
    Serial.print("        ID of 0xFF probably means a bad address, a BMP 180 or BMP 085\n");
    Serial.print("   ID of 0x56-0x58 represents a BMP 280,\n");
    Serial.print("        ID of 0x60 represents a BME 280.\n");
    Serial.print("        ID of 0x61 represents a BME 680.\n");
    while (1) delay(10);
  }

  /* Default settings from datasheet. */
  bmp.setSampling(Adafruit_BMP280::MODE_NORMAL,     /* Operating Mode. */
                  Adafruit_BMP280::SAMPLING_X2,     /* Temp. oversampling */
                  Adafruit_BMP280::SAMPLING_X16,    /* Pressure oversampling */
                  Adafruit_BMP280::FILTER_X16,      /* Filtering. */
                  Adafruit_BMP280::STANDBY_MS_500); /* Standby time. */


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

    //MPU
    gyro();
    eixoX -= eixoXCalibre;
    eixoY -= eixoYCalibre;
    eixoZ -= eixoZCalibre;
    //Armazena o valor dos sensores nas variaveis correspondentes
    acelX = Wire.read()<<8|Wire.read();  //0x3B (ACCEL_XOUT_H) & 0x3C (ACCEL_XOUT_L)  
    acelY = Wire.read()<<8|Wire.read();  //0x3D (ACCEL_YOUT_H) & 0x3E (ACCEL_YOUT_L)  
    acelZ = Wire.read()<<8|Wire.read();  //0x3F (ACCEL_ZOUT_H) & 0x40 (ACCEL_ZOUT_L)  
  
    temperatura = Wire.read()<<8|Wire.read();  //0x41 (TEMP_OUT_H) & 0x42 (TEMP_OUT_L)

    //Printa o valor X do acelerômetro na serial
    Serial.print("Acelerômetro X = "); 
    Serial.print(acelX);
    dataFrame.print(acelX); 
    dataFrame.print(" ");
    dataFrame.print("m/s^2");
    dataFrame.print("\t");
  
    //Printa o valor Y do acelerômetro na serial
    Serial.print(" \tY = "); 
    Serial.print(acelY);
    dataFrame.print(acelY); 
    dataFrame.print(" ");
    dataFrame.print("m/s^2");
    dataFrame.print("\t");    
    //Printa o valor Z do acelerômetro na serial
    Serial.print(" \tZ = "); 
    Serial.println(acelZ);
    dataFrame.print(acelZ); 
    dataFrame.print(" ");
    dataFrame.print("m/s^2");
    dataFrame.print("\t");
  
    //Printa o valor X do giroscópio na serial
    Serial.print("Giroscópio X = "); 
    Serial.print(eixoX);
    dataFrame.print(eixoX); 
    dataFrame.print(" ");
    dataFrame.print("rad/s");
    dataFrame.print("\t");
  

  //  //Printa o valor Y do giroscópio na serial
    Serial.print(" \tY = "); 
    Serial.print(eixoY);
    dataFrame.print(eixoY); 
    dataFrame.print(" ");
    dataFrame.print("rad/s");
    dataFrame.print("\t");
    
  //   
  //  //Printa o valor Z do giroscópio na serial
    Serial.print(" \tZ = "); 
    Serial.println(eixoZ);
    dataFrame.print(eixoY); 
    dataFrame.print(" ");
    dataFrame.print("rad/s");
    dataFrame.print("\t");
 
    
  //  //Printa o valor da temperatura na serial, calculando em graus celsius
  //  Serial.print("Temperatura = "); 
  //  Serial.println(temperatura / 340.00 + 36.53);
    delay(50);

    //BMP
    altitude = bmp.readAltitude(1013.25);

    Serial.print(F("Altitude aproximada = "));
    Serial.print(altitude); /* Adjusted to local forecast! */
    Serial.println(" m");
    dataFrame.print(altitude); 
    dataFrame.print(" ");
    dataFrame.print("m");
    dataFrame.print("\t");
    // Altura
    altura = altitude - calibraAltitude;
    dataFrame.print(altura); 
    dataFrame.print(" ");
    dataFrame.print("m");
    dataFrame.print("\t");
    // Pressao atmosferica
    pressao = bmp.readPressure();
    dataFrame.print(pressao); 
    dataFrame.print(" ");
    dataFrame.print("Pa");
    dataFrame.print("\t");    

    // Temperatura BMP
    tempBMP = bmp.readTemperature();
    dataFrame.print(tempBMP); 
    dataFrame.print(" ");
    dataFrame.print("oC");
    dataFrame.print("\n");

    delay(100);



    if (altitude < altitudeAnterior) {
      pinmode(3, HIGH);
      delay(100);
    }
    
    altitudeAnterior = altitude;

    Serial.println();
    delay(500);
}
