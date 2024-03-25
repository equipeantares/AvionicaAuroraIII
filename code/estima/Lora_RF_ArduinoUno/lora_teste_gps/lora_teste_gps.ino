#include <SoftwareSerial.h>
#include <TinyGPS.h>
#include <EBYTE.h>

#define RX_LORA 16 // Pino RX do módulo LoRa
#define TX_LORA 17 // Pino TX do módulo LoRa

SoftwareSerial lora(RX_LORA, TX_LORA);

// Defina os pinos M0, M1 e AUX de acordo com a pinagem do ESP32
#define M0_LORA 5
#define M1_LORA 18
#define AUX_LORA 19

EBYTE LoRa(&lora, M0_LORA, M1_LORA, AUX_LORA);

SoftwareSerial serialGPS(6, 7);

TinyGPS gps;

String urlMapa;

void leGPS();

void setup() {

  Serial.begin(9600);
  serialGPS.begin(9600);

  Serial.println("Sketch Iniciado!");
  
  while (!Serial);

  lora.begin(9600);
  LoRa.init();

  // LoRa.Reset(); // Opcional: Reseta os parâmetros para os de fábrica

  // Defina a taxa de dados de acordo com suas necessidades
  LoRa.SetAirDataRate(ADR_1K);

  // Defina o endereço da rede
  LoRa.SetAddress(1);

  // Defina o canal de comunicação
  LoRa.SetChannel(23);

  // Salve as configurações
  LoRa.SaveParameters(TEMPORARY);

  // Imprima os parâmetros do módulo
  LoRa.PrintParameters();

  // Defina o modo de operação
  LoRa.SetMode(MODE_NORMAL);

  leGPS();
}

void loop() {
  static unsigned long delayLeGPS = millis();

  if ((millis() - delayLeGPS) > 10000) {
    leGPS();
    delayLeGPS = millis();
  }

  if (Serial.available()>0){
  String msg = urlMapa;
  lora.print(msg);
  Serial.println(msg);
  }
}

void leGPS() {
  unsigned long delayGPS = millis();

  serialGPS.listen();
  bool lido = false;
  while ((millis() - delayGPS) < 500) {
    while (serialGPS.available()) {
      char cIn = serialGPS.read();
      lido = gps.encode(cIn);
    }

    if (lido) {
      float flat, flon;
      unsigned long age;

      gps.f_get_position(&flat, &flon, &age);

      urlMapa = "Local Identificado: https://maps.google.com/maps/?&z=10&q=";
      urlMapa += String(flat, 6);
      urlMapa += ",";
      urlMapa += String(flon, 6);
      Serial.println(urlMapa);

      break;
    }
  }
}

