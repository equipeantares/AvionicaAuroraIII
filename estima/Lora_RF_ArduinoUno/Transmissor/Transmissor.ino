#include <SoftwareSerial.h>

//SoftwareSerial mySerial(22, 23); // RX (D22), TX (D23)
String mensagem = "teste"; // Altere para a string desejada

SoftwareSerial lora(22, 23);
#define LORA_FREQUENCY 900E6


void setup() {
  Serial.begin(9600);
  while (!Serial);

  Serial.println("Iniciando transmissor ESP32...");

  lora.begin(LORA_FREQUENCY);

  Serial.println("Transmissor pronto!");
}

void loop() {
  lora.println(mensagem);
  Serial.println("Mensagem enviada: " + mensagem);

  delay(5000);
}