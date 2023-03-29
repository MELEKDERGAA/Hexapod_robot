#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>

RF24 radio(0, 0); // CE, CSN pins
const byte address[6] = "00001";

void setup() {
  radio.begin();
  radio.openWritingPipe(address);
  radio.setPALevel(RF24_PA_MIN);
  radio.stopListening();
}

void loop() {
  String message = "walk/forward";
  radio.write(&message, sizeof(message));
  delay(1000);
}
