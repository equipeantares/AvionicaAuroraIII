#include <SoftwareSerial.h>                               //Biblioteca de comunicação do módulo
#include <EBYTE.h>                                        //Biblioteca do Kris
#include <Wire.h>
#include <LiquidCrystal_I2C.h>

int counter = 0;
LiquidCrystal_I2C lcd(0x27,  16, 2);
                     // Clear the LCD screen
#define M0_LoRa   11
#define M1_LoRa   12
#define RX_LoRa   3                                       // Vai no TXD do módulo
#define TX_LoRa   2                                       // Vai no RXD do módulo
#define AUX_LoRa  4
                                                  
SoftwareSerial lora(RX_LoRa, TX_LoRa);                   //Definição dos pinos para criar a comunicação serial
EBYTE LoRa(&lora, M0_LoRa, M1_LoRa, AUX_LoRa);           //Criar objeto de Transmissão, passando na comunicação serial e pinos

void setup() {
  lcd.init();                       // Initialize the LCD
  lcd.backlight();                  // Turn on the backlight
  lcd.clear(); 
  Serial.begin (9600);                                    //Definir taxa de trabalho em 9600
  lora.begin(9600);
  LoRa.init();                                          // Inicializa a comunicação e obtem todos os parâmetros do módulo

  //LoRa.Reset();                                       // Reseta parâmetros para os de fábrica
  LoRa.SetAirDataRate(ADR_1K);                          // Estabelece a taxa de dados de transmissão
  LoRa.SetAddress(1);                                   // Estabelece o endereço da rede
  LoRa.SetChannel(23);                                  // Estabelece canal como 23
  LoRa.SaveParameters(TEMPORARY);                       // Salva todas as definições de forma temporária

  LoRa.PrintParameters();                               // Imprime todos os parâmetros (configurações) obtidos do módulo 
  LoRa.SetMode(MODE_NORMAL);                            // Define modo de operação como normal
}
 


void loop() {
  lcd.setCursor(0, 0);               // Set the cursor to the first column and first row
  lcd.print("Mensagem Lora:");     // Print some text
  lcd.setCursor(0,1);

  delay(500);    
if (lora.available()>0){
  String input = lora.readString();
  Serial.print(" A Mensagem do Lora é: "); 
  lcd.print(input);
  Serial.println(input);
  
}
delay(1500);
}
