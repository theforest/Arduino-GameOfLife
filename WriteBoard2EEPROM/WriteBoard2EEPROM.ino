#include <EEPROM.h>
#include <Kamots.h>

void setup() {
EEPROM.write(339, 0x69);
const uint8_t gameBoard[28][5] = {
  { B00000000, B00000000, B00000010, B10000000, B10000010 },
  { B00000000, B00000000, B00000100, B00000001, B01000101 },
  { B00111111, B01100000, B11001000, B01000001, B00101001 },
  { B00111111, B01100000, B11010010, B11000000, B01000100 },
  { B00000000, B01100000, B00001100, B00000000, B01101100 },
  { B00110000, B01100000, B00000000, B00000000, B00000000 },
  { B00110000, B01100000, B00000000, B00000000, B00000000 },
  { B00110000, B01100000, B10000000, B10000000, B00000000 },
  { B00110000, B00000001, B01000001, B01000000, B00000000 },
  { B00110111, B11100000, B10000000, B10000000, B00000100 },
  { B00110111, B11100000, B00000000, B00000000, B00011010 },
  { B00000000, B00000000, B00000000, B00000000, B01000010 },
  { B00000000, B00000000, B00000000, B00000000, B11100000 },
  { B00000000, B00000000, B11100011, B10000000, B00000000 },
  { B00000000, B00000000, B10100010, B10000000, B00111000 },
  { B00000000, B00000000, B11100011, B10000010, B00010000 },
  { B11101110, B00000000, B00000000, B00000010, B11000000 },
  { B01010100, B00000000, B00000000, B00000001, B00000000 },
  { B00000000, B01100000, B00000000, B00000000, B00000000 },
  { B01000100, B01100000, B00000000, B00000000, B00001100 },
  { B00000000, B00011000, B00000000, B00000000, B00001100 },
  { B10101010, B00011000, B00000000, B00000000, B00000000 },
  { B00101000, B00000000, B10000000, B10000000, B00000100 },
  { B00000000, B00000001, B01000001, B01000000, B00001010 },
  { B00000000, B00010000, B10000000, B10000001, B10010010 },
  { B00001100, B00001100, B00001000, B00000001, B10100000 },
  { B00010100, B00110000, B00001000, B00000000, B00011000 },
  { B00001000, B00001000, B00001000, B00000000, B00000000 },
};
  EEPROM_writeAnything(340, gameBoard);
  pinMode(13, OUTPUT);
  while (1) {
    digitalWrite(13, HIGH);
    delay(500);
    digitalWrite(13, LOW);
    delay(500);
  }
}

void loop() {
}