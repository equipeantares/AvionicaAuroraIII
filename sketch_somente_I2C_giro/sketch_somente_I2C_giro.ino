#include<Wire.h>

//Endereço em hexadecimal do sensor MPU 6050
const int ENDERECO_SENSOR=0x68;  

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
}

void loop()
{
    gyro();
    eixoX -= eixoXCalibre;
    eixoY -= eixoYCalibre;
    eixoZ -= eixoZCalibre;
    //Armazena o valor dos sensores nas variaveis correspondentes
    //acelX = Wire.read()<<8|Wire.read();  //0x3B (ACCEL_XOUT_H) & 0x3C (ACCEL_XOUT_L)  
    //acelY = Wire.read()<<8|Wire.read();  //0x3D (ACCEL_YOUT_H) & 0x3E (ACCEL_YOUT_L)  
    //acelZ = Wire.read()<<8|Wire.read();  //0x3F (ACCEL_ZOUT_H) & 0x40 (ACCEL_ZOUT_L)  
  
    //temperatura = Wire.read()<<8|Wire.read();  //0x41 (TEMP_OUT_H) & 0x42 (TEMP_OUT_L)

    //Printa o valor X do acelerômetro na serial
    //Serial.print("Acelerômetro X = "); 
    //Serial.print(acelX);
  
    //Printa o valor Y do acelerômetro na serial
    //Serial.print(" \tY = "); 
    //Serial.print(acelY);
    
    //Printa o valor Z do acelerômetro na serial
    //Serial.print(" \tZ = "); 
    //Serial.println(acelZ);

    //Printa o valor X do giroscópio na serial
    Serial.print("Giroscópio X = "); 
    Serial.print(eixoX);
    
  //  //Printa o valor Y do giroscópio na serial
    Serial.print(" \tY = "); 
    Serial.print(eixoY);
  //   
  //  //Printa o valor Z do giroscópio na serial
    Serial.print(" \tZ = "); 
    Serial.println(eixoZ); 
    
  //  //Printa o valor da temperatura na serial, calculando em graus celsius
  //  Serial.print("Temperatura = "); 
  //  Serial.println(temperatura / 340.00 + 36.53);
    delay(50);
}
