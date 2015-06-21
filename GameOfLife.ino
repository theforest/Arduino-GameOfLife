/* Conway's Game of Life for Arduino
 * Version: 4.8 (20131202) By: Kamots http://theforest.us/lab/
 * Copyright 2013 kamotswind
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 ***************************************************************************
 * Originally based on code from http://brownsofa.org/blog/archives/170 by "Ian"  December 30, 2010
 * Changes and additions by Kamots:
 * 1.0 Modified to work with Adafruit ST7735 TFT display and grid size increased from 8x8 to 28x28
 * 1.1 Cleaned up code and removed a lot of unnecessary lines
 * 1.2 Added display of critical platform data and current grid generation
 * 2.0 Modified grid storage to use bits instead of boolean; increased grid to 28x40
 * 2.1 Added on-screen menu with EEPROM save/restore options and color toggle
 * 2.2 Consolidated the new grid calculation function to improve speed
 * 2.3 Added live cell count and display. Grid will random generate if stale life
 * 2.4 Changed grid copies to using memcpy instead of for loop
 * 2.5 Increased grid size to 29x40 and added additional EEPROM save/restore options
 * 2.6 Added ability to store generation 1 to EEPROM by saving to seperate array when gen=1
 * 3.0 Added new patterns submenu to load predefined patterns (suggestions?)
 * 3.1 Moved EEPROM options to submenu
 * 4.0 Added save all EEPROM blocks to SD option; saves in Golly .rle format, one file per block
 * 4.1 Grid now resets back to generation 1 if life is stale for 100 sequential generations
 * 4.2 Built array for "changed cells"; now only sends screen updates for changed cells (nice speed increase)
 * 4.3 Further optimized new generation calculation code, cut time in half!
 * 4.4 Added menu option to toggle speed from full to ~2 generations per second (normal is about 4.5gen/sec!)
 * 4.5 Corrected compatibility issues with newer Adafruit_ST7735 library (Nov 08, 2013 commit)
 * 4.7 Started Due/DigiX compatibility work (compiles, but not working yet)
 * 4.8 Added menu option to toggle stats display (10+gen/sec when off!); removed millis from stats display
 */

#ifndef __AVR__
#include <Wire.h>
#include <Extensive_EEPROM.h>
#else
#include <EEPROM.h>
#endif
#include <SPI.h>
#include <Adafruit_ST7735.h>
#include <Adafruit_GFX.h>
#include <SdFat.h>
#include <Kamots.h>

#ifndef __AVR__
#define SPI_SCK 13
#define SPI_DI  12
#define SPI_DO  11
#define TFT_CS   9
#define TFT_DC   8
Adafruit_ST7735 tft = Adafruit_ST7735(TFT_CS, TFT_DC, SPI_DO, SPI_SCK, 0);
#else
Adafruit_ST7735 tft = Adafruit_ST7735(9, 8, 0);
#endif

const int NUMROWS = 29; // 29x40 is largest grid size recommended for Adafruit 1.8" TFT with status line
const int NUMCOLS = 40; // This number must always be evenly divisible by 8
const int ANALOG_PIN = 4; // for randomness when generating grids
unsigned int gen = 0, stalegens = 0; // current generation and stale generations
unsigned long lastliving = 0, lastdisp = 0; // living cells count
boolean tcolor = false, slow = false, enstats = true; // disable or enable: color, slow speed, stats
uint8_t newGameBoard[NUMROWS][NUMCOLS/8]; // only 145 bytes!
uint8_t gameBoard[NUMROWS][NUMCOLS/8];
uint8_t gameBoardChanges[NUMROWS][NUMCOLS/8]; // Tracks changed cells to reduce screen updates
uint8_t startGameBoard[NUMROWS][NUMCOLS/8]; // Caches the gen = 1 board for reviewing or EEPROM save later
const uint8_t SD_CS = 7; // Change to appropriate SD card CS pin
SdFat sd;
#define error(s) sd.errorHalt_P(PSTR(s))

// Below are many pre-defined patterns which are stored in program memory (flash)
// Each only takes up 145 bytes, so feel free to add more, just need to modify the menus

PROGMEM prog_uint8_t j[NUMROWS][NUMCOLS/8] = {
  { B00000000, B00000000, B00000000, B00000000, B00000000 },
  { B00000000, B00000000, B00000000, B00000000, B00000000 },
  { B00000000, B00000000, B00000000, B00000000, B00000000 },
  { B00000000, B00000000, B00000000, B00000000, B00000000 },
  { B00000000, B00000000, B00000000, B00000000, B00000000 },
  { B00000000, B00000000, B00000000, B00000000, B00000000 },
  { B00000000, B00000000, B00000000, B00000000, B00000000 },
  { B00000000, B00000000, B00000000, B00000000, B00000000 },
  { B00000000, B00000000, B00000000, B00000000, B00000000 },
  { B00000000, B00000000, B00000000, B00000000, B00000000 },
  { B00000000, B00000000, B00000000, B00000000, B00000000 },
  { B00000000, B00000000, B00000000, B00000000, B00000000 },
  { B00000000, B00000000, B00011000, B00000000, B00000000 },
  { B00000000, B00000000, B00011100, B00000000, B00000000 },
  { B00000000, B00000000, B01000100, B00000000, B00000000 },
  { B00000000, B00000000, B01100000, B00000000, B00000000 },
  { B00000000, B00000000, B00100000, B00000000, B00000000 },
  { B00000000, B00000000, B00000000, B00000000, B00000000 },
  { B00000000, B00000000, B00000000, B00000000, B00000000 },
  { B00000000, B00000000, B00000000, B00000000, B00000000 },
  { B00000000, B00000000, B00000000, B00000000, B00000000 },
  { B00000000, B00000000, B00000000, B00000000, B00000000 },
  { B00000000, B00000000, B00000000, B00000000, B00000000 },
  { B00000000, B00000000, B00000000, B00000000, B00000000 },
  { B00000000, B00000000, B00000000, B00000000, B00000000 },
  { B00000000, B00000000, B00000000, B00000000, B00000000 },
  { B00000000, B00000000, B00000000, B00000000, B00000000 },
  { B00000000, B00000000, B00000000, B00000000, B00000000 },
  { B00000000, B00000000, B00000000, B00000000, B00000000 },
};

PROGMEM prog_uint8_t gliderGun[NUMROWS][NUMCOLS/8] = {
  { B00000000, B00000000, B00000000, B00000000, B00000000 },
  { B00000000, B00000000, B00000000, B01000000, B00000000 },
  { B00000000, B00000000, B00000001, B01000000, B00000000 },
  { B00000000, B00000110, B00000110, B00000000, B00011000 },
  { B00000000, B00001000, B10000110, B00000000, B00011000 },
  { B01100000, B00010000, B01000110, B00000000, B00000000 },
  { B01100000, B00010001, B01100001, B01000000, B00000000 },
  { B00000000, B00010000, B01000000, B01000000, B00000000 },
  { B00000000, B00001000, B10000000, B00000000, B00000000 },
  { B00000000, B00000110, B00000000, B00000000, B00000000 },
  { B00000000, B00000000, B00000000, B00000000, B00000000 },
  { B00000000, B00000000, B00000000, B00000000, B00000000 },
  { B00000000, B00000000, B00000000, B00000000, B00000000 },
  { B00000000, B00000000, B00000000, B00000000, B00000000 },
  { B00000000, B00000000, B00000000, B00000000, B00000000 },
  { B00000000, B00000000, B00000000, B00000000, B00000000 },
  { B00000000, B00000000, B00000000, B00000000, B00000000 },
  { B00000000, B00000000, B00000000, B00000000, B00000000 },
  { B00000000, B00000000, B00000000, B00000000, B00000000 },
  { B00000000, B00000000, B00000000, B00000000, B00000000 },
  { B00000000, B00000000, B00000000, B00000000, B00000000 },
  { B00000000, B00000000, B00000000, B00000000, B00000000 },
  { B00000000, B00000000, B00000000, B00000000, B00001100 },
  { B00000000, B00000000, B00000000, B00000000, B00001010 },
  { B00000000, B00000000, B00000000, B00000000, B00000010 },
  { B00000000, B00000000, B00000000, B00000000, B00000011 },
  { B00000000, B00000000, B00000000, B00000000, B00000000 },
  { B00000000, B00000000, B00000000, B00000000, B00000000 },
  { B00000000, B00000000, B00000000, B00000000, B00000000 },
};

PROGMEM prog_uint8_t achimsP144[NUMROWS][NUMCOLS/8] = {
  { B00000000, B00000000, B00000000, B00000000, B00000000 },
  { B00000000, B00000000, B00000000, B00000000, B00000000 },
  { B00110000, B00000000, B00000000, B00001100, B00000000 },
  { B00110000, B00000000, B00000000, B00001100, B00000000 },
  { B00000000, B00000000, B00001100, B00000000, B00000000 },
  { B00000000, B00000000, B00010010, B00000000, B00000000 },
  { B00000000, B00000000, B00001100, B00000000, B00000000 },
  { B00000000, B00000000, B10000000, B00000000, B00000000 },
  { B00000000, B00000001, B01000000, B00000000, B00000000 },
  { B00000000, B00000010, B00100000, B00000000, B00000000 },
  { B00000000, B00000010, B01000000, B00000000, B00000000 },
  { B00000000, B00000000, B00000000, B00000000, B00000000 },
  { B00000000, B00000010, B01000000, B00000000, B00000000 },
  { B00000000, B00000100, B01000000, B00000000, B00000000 },
  { B00000000, B00000010, B10000000, B00000000, B00000000 },
  { B00000000, B00000001, B00000000, B00000000, B00000000 },
  { B00000000, B00110000, B00000000, B00000000, B00000000 },
  { B00000000, B01001000, B00000000, B00000000, B00000000 },
  { B00000000, B00110000, B00000000, B00000000, B00000000 },
  { B00110000, B00000000, B00000000, B00001100, B00000000 },
  { B00110000, B00000000, B00000000, B00001100, B00000000 },
  { B00000000, B00000000, B00000000, B00000000, B00000000 },
  { B00000000, B00000000, B00000000, B00000000, B00000000 },
  { B00000000, B00000000, B00000000, B00000000, B00000000 },
  { B00000000, B00000000, B00000000, B00000000, B00000000 },
  { B00000000, B00000000, B00000000, B00000000, B00000000 },
  { B00000000, B00000000, B00000000, B00000000, B00000000 },
  { B00000000, B00000000, B00000000, B00000000, B00000000 },
  { B00000000, B00000000, B00000000, B00000000, B00000000 },
};

PROGMEM prog_uint8_t GbtD[NUMROWS][NUMCOLS/8] = {
  { B00000000, B00000000, B00000000, B00000000, B00000000 },
  { B00000000, B00000000, B00000000, B00000000, B00000000 },
  { B00000000, B00000000, B00000000, B00000000, B00000000 },
  { B00000000, B00000000, B00000000, B00000000, B00000000 },
  { B00000000, B00000000, B00000000, B00000000, B00000000 },
  { B00000000, B00000000, B00000000, B00000000, B00000000 },
  { B00000000, B00000000, B00000000, B00000000, B00000000 },
  { B00000000, B00000000, B00000000, B00000000, B00000000 },
  { B00000000, B00000000, B00000000, B00000000, B00000000 },
  { B00000000, B00000000, B00000000, B00000000, B00000000 },
  { B00000000, B00000000, B00000000, B00000000, B00000000 },
  { B00000000, B00000000, B00000000, B00000000, B00000000 },
  { B00000000, B00000000, B00000000, B00000000, B00000000 },
  { B00000000, B00000000, B00110010, B00000000, B00000000 },
  { B00000000, B00000000, B00100010, B00000000, B00000000 },
  { B00000000, B00000000, B00100110, B00000000, B00000000 },
  { B00000000, B00000000, B00000000, B00000000, B00000000 },
  { B00000000, B00000000, B00000000, B00000000, B00000000 },
  { B00000000, B00000000, B00000000, B00000000, B00000000 },
  { B00000000, B00000000, B00000000, B00000000, B00000000 },
  { B00000000, B00000000, B00000000, B00000000, B00000000 },
  { B00000000, B00000000, B00000000, B00000000, B00000000 },
  { B00000000, B00000000, B00000000, B00000000, B00000000 },
  { B00000000, B00000000, B00000000, B00000000, B00000000 },
  { B00000000, B00000000, B00000000, B00000000, B00000000 },
  { B00000000, B00000000, B00000000, B00000000, B00000000 },
  { B00000000, B00000000, B00000000, B00000000, B00000000 },
  { B00000000, B00000000, B00000000, B00000000, B00000000 },
  { B00000000, B00000000, B00000000, B00000000, B00000000 },
};

PROGMEM prog_uint8_t heart[NUMROWS][NUMCOLS/8] = {
  { B00000000, B00000000, B00000000, B00000000, B00000000 },
  { B00000000, B00000000, B00000000, B00000000, B00000000 },
  { B00000000, B00000000, B00000000, B00000000, B00000000 },
  { B00000000, B00000000, B00000000, B00000000, B00000000 },
  { B00000000, B00000000, B00000000, B00000000, B00000000 },
  { B00000000, B00000000, B00000000, B00000000, B00000000 },
  { B00000000, B00000000, B00000000, B00000000, B00000000 },
  { B00000000, B00000000, B00000000, B00000000, B00000000 },
  { B00000000, B00000000, B01101100, B00000000, B00000000 },
  { B00000000, B00000000, B01000100, B00000000, B00000000 },
  { B00000000, B00000000, B00111000, B00000000, B00000000 },
  { B00000000, B00000000, B00000000, B00000000, B00000000 },
  { B00000000, B00000000, B00000000, B00000000, B00000000 },
  { B00000000, B00000000, B00000000, B00000000, B00000000 },
  { B00000000, B00000000, B00000000, B00000000, B00000000 },
  { B00000000, B00000000, B00000000, B00000000, B00000000 },
  { B00000000, B00000000, B00000000, B00000000, B00000000 },
  { B00000000, B00000000, B00000000, B00000000, B00000000 },
  { B00000000, B00000000, B00000000, B00000000, B00000000 },
  { B00000000, B00000000, B00000000, B00000000, B00000000 },
  { B00000000, B00000000, B00000000, B00000000, B00000000 },
  { B00000000, B00000000, B00000000, B00000000, B00000000 },
  { B00000000, B00000000, B00000000, B00000000, B00000000 },
  { B00000000, B00000000, B00000000, B00000000, B00000000 },
  { B00000000, B00000000, B00000000, B00000000, B00000000 },
  { B00000000, B00000000, B00000000, B00000000, B00000000 },
  { B00000000, B00000000, B00000000, B00000000, B00000000 },
  { B00000000, B00000000, B00000000, B00000000, B00000000 },
  { B00000000, B00000000, B00000000, B00000000, B00000000 },
};

PROGMEM prog_uint8_t blockAndGlider[NUMROWS][NUMCOLS/8] = {
  { B00000000, B00000000, B00000000, B00000000, B00000000 },
  { B00000000, B00000000, B00000000, B00000000, B00000000 },
  { B00000000, B00000000, B00000000, B00000000, B00000000 },
  { B00000000, B00000000, B00000000, B00000000, B00000000 },
  { B00000000, B00000000, B00000000, B00000000, B00000000 },
  { B00000000, B00000000, B00000000, B00000000, B00000000 },
  { B00000000, B00000000, B00000000, B00000000, B00000000 },
  { B00000000, B00000000, B00000000, B00000000, B00000000 },
  { B00000000, B00000000, B00000000, B00000000, B00000000 },
  { B00000000, B00000000, B00000000, B00000000, B00000000 },
  { B00000000, B00000000, B00000000, B00000000, B00000000 },
  { B00000000, B00000000, B00000000, B00000000, B00000000 },
  { B00000000, B00000000, B00000000, B00000000, B00000000 },
  { B00000000, B00000000, B00000000, B00000000, B00000000 },
  { B00000000, B00000000, B00110000, B00000000, B00000000 },
  { B00000000, B00000000, B00101000, B00000000, B00000000 },
  { B00000000, B00000000, B00001100, B00000000, B00000000 },
  { B00000000, B00000000, B00000000, B00000000, B00000000 },
  { B00000000, B00000000, B00000000, B00000000, B00000000 },
  { B00000000, B00000000, B00000000, B00000000, B00000000 },
  { B00000000, B00000000, B00000000, B00000000, B00000000 },
  { B00000000, B00000000, B00000000, B00000000, B00000000 },
  { B00000000, B00000000, B00000000, B00000000, B00000000 },
  { B00000000, B00000000, B00000000, B00000000, B00000000 },
  { B00000000, B00000000, B00000000, B00000000, B00000000 },
  { B00000000, B00000000, B00000000, B00000000, B00000000 },
  { B00000000, B00000000, B00000000, B00000000, B00000000 },
  { B00000000, B00000000, B00000000, B00000000, B00000000 },
  { B00000000, B00000000, B00000000, B00000000, B00000000 },
};

PROGMEM prog_uint8_t gens987[NUMROWS][NUMCOLS/8] = {
  { B00000001, B01111010, B00000000, B00000000, B00000000 },
  { B00000001, B01100100, B11111101, B11000100, B00000000 },
  { B01000111, B00001110, B00010001, B10110001, B11000000 },
  { B01001111, B00000110, B00011110, B00010001, B00001000 },
  { B01000000, B10010110, B00011000, B01100000, B01000000 },
  { B10010100, B01010010, B00010001, B01100111, B01011000 },
  { B01010100, B00110100, B00001110, B00011110, B00000100 },
  { B01100001, B00000000, B10010010, B00000100, B10110011 },
  { B00110101, B10100100, B11110000, B00110110, B01001101 },
  { B00001110, B00011101, B00101000, B00000110, B00001011 },
  { B00000000, B10000001, B10000000, B10001000, B00001110 },
  { B00000000, B00000001, B01100000, B01111000, B00110010 },
  { B00011001, B10011001, B01100000, B00011100, B01010000 },
  { B00010010, B11111001, B01100001, B00000000, B00010010 },
  { B00110000, B00000001, B01000101, B00001111, B00100000 },
  { B01001010, B11100000, B00000000, B00000001, B00100000 },
  { B00010110, B10010111, B10000000, B00000100, B00000000 },
  { B10001001, B11001011, B00000100, B01010001, B11010100 },
  { B11000000, B11110000, B01000100, B00100110, B11011000 },
  { B00000000, B00001001, B11000110, B00011110, B11000100 },
  { B00000000, B01110001, B01000110, B11001100, B00000000 },
  { B00000111, B00000000, B00000100, B10110011, B00110000 },
  { B00000001, B00011100, B00000100, B01000000, B00010000 },
  { B01000011, B00011000, B00000000, B01110110, B10000000 },
  { B01110111, B00110010, B00010100, B11110100, B00000000 },
  { B00101010, B01100110, B00001010, B11101110, B00000000 },
  { B00011100, B11100111, B01001000, B01101100, B00000011 },
  { B00000010, B00000001, B01011110, B11000110, B00000110 },
  { B00000000, B00000000, B00000000, B10000000, B00000110 },
};

PROGMEM prog_uint8_t initialGameBoard[NUMROWS][NUMCOLS/8] = {
  { B00000000, B00000000, B00000010, B10000000, B10000010 },
  { B00000000, B00000000, B00000100, B00000001, B01000101 },
  { B00111111, B01100000, B11001000, B01000001, B00101001 },
  { B00111111, B01100000, B11010010, B11000000, B01000100 },
  { B00000000, B01100000, B00001100, B00000000, B01101100 },
  { B00110000, B01100000, B00000000, B00000000, B00000000 },
  { B00110000, B01100000, B00000000, B00000000, B00000000 },
  { B00110000, B01100000, B10000000, B10000100, B00000000 },
  { B00110000, B00000001, B01000001, B01000100, B00000000 },
  { B00110111, B11100000, B10000000, B10000100, B00000100 },
  { B00110111, B11100000, B00000000, B00000000, B00011010 },
  { B00000000, B00000000, B00000000, B00000000, B01000010 },
  { B00000000, B00000000, B00000000, B00000000, B11100000 },
  { B00000000, B00000000, B11100011, B10000000, B00000000 },
  { B11000000, B00000000, B10100010, B10000000, B00111000 },
  { B10100000, B01000000, B11100011, B10000010, B00010000 },
  { B00000000, B10100000, B00000000, B00000010, B11000000 },
  { B00101000, B10100000, B00000000, B00000001, B00000000 },
  { B00011000, B01000000, B00000000, B00000000, B00000000 },
  { B00000000, B00000000, B00000000, B00000000, B00000000 },
  { B00000000, B00000000, B00000000, B00001110, B00001100 },
  { B00000000, B00110000, B00000000, B00000111, B00001100 },
  { B11101110, B01010000, B10000000, B10000000, B00000000 },
  { B01010100, B00100001, B01000001, B01000000, B00000100 },
  { B00000000, B00000000, B10000000, B10000000, B00001010 },
  { B01000100, B00001000, B00001000, B00011001, B10010010 },
  { B00000000, B00001010, B00000110, B00011001, B10100000 },
  { B10101010, B00100001, B00011000, B01100000, B00011000 },
  { B00101000, B00111111, B00000100, B01100000, B00000000 },
};

// Simple function to collect system/game resource data and display on screen
void getresources() {
  //int fr = freeRam();
  unsigned long vcc = readVcc();
  char res[28];
  const char* fmt = "%ig %luca %ludV    ";
  sprintf(res, fmt, gen, lastliving, vcc);
  //Serial.println(res);
  tft.setCursor(0,119);
  tft.print(res);
}

// Reads a cell's status from the current grid
boolean getGridItem(byte row, byte col) {
  uint8_t bitcol = 0;
  while(col >= 8) {
    col = col - 8;
    bitcol++;
  }
  col = 7 - col;
  return ((1<<col) & gameBoard[row][bitcol]);
}

// Reads a cell's changed status
boolean getGridChanged(byte row, byte col) {
  uint8_t bitcol = 0;
  while(col >= 8) {
    col = col - 8;
    bitcol++;
  }
  col = 7 - col;
  return ((1<<col) & gameBoardChanges[row][bitcol]);
}

// Sets a cell's status on the current grid
void setGridItem(byte row, byte col, boolean val) {
  uint8_t bitcol = 0;
  while(col >= 8) {
    col = col - 8;
    bitcol++;
  }
  col = 7 - col;
  if (val) gameBoard[row][bitcol] |= (1 << col);
  else gameBoard[row][bitcol] &= ~(1 << col);
}

// Sets a cell's status on the next generation grid
void setNewGridItem(byte row, byte col, boolean val) {
  uint8_t bitcol = 0;
  while(col >= 8) {
    col = col - 8;
    bitcol++;
  }
  col = 7 - col;
  if (val) newGameBoard[row][bitcol] |= (1 << col);
  else newGameBoard[row][bitcol] &= ~(1 << col);
}

// Sets a cell's changed status (for screen updates)
void setChangedItem(byte row, byte col) {
  uint8_t bitcol = 0;
  while(col >= 8) {
    col = col - 8;
    bitcol++;
  }
  col = 7 - col;
  gameBoardChanges[row][bitcol] |= (1 << col);
}

// Toggles a cell's status on the current grid
void toggleGridItem(byte row, byte col) {
  byte bitcol = 0;
  while(col >= 8) {
    col = col - 8;
    bitcol++;
  }
  col = 7 - col;
  gameBoard[row][bitcol] ^= (1 << col);
}

// Copies the initial game board from program memory to current grid
// Also sets generation counter to 1. This is run on startup/reset.
void setupInitialBoard() {
  memcpy_P(gameBoard, &initialGameBoard, sizeof(gameBoard));
  gen = 1;
}

// Copies current grid to "start grid" so it can be recalled or saved later
void copyBoardToStart() {
  memcpy(startGameBoard, gameBoard, sizeof(gameBoard));
}

// Generates a new random grid and resets generation counter to 0
// This is done so one evolution runs before it is saved
// May display a little strange for the first calcuation
void setupNewBoard() {
  memset(gameBoard, 0, sizeof(gameBoard));
  randomSeed(analogRead(ANALOG_PIN));
  unsigned long numChanges = random(50,1000);
  for (unsigned long i=0; i<numChanges; i++) {
    byte row = random(0, NUMROWS);
    byte col = random(0, NUMCOLS);
    toggleGridItem(row, col);
  }
  gen = 0;
}

// Updates a specific cell's status on the display, can modify this for other displays
void setCell(byte row, byte col, boolean state) {
  if(state) {
    if(tcolor) {
      switch (random(0,3)) {
        case 1:
          tft.fillRect((int16_t)col*4, (int16_t)row*4, 4, 4, ST7735_BLUE);
          break;
        case 2:
          tft.fillRect((int16_t)col*4, (int16_t)row*4, 4, 4, ST7735_GREEN);
          break;
        default:
          tft.fillRect((int16_t)col*4, (int16_t)row*4, 4, 4, ST7735_RED);
      }
    } else tft.fillRect((int16_t)col*4, (int16_t)row*4, 4, 4, ST7735_WHITE);
  } else tft.fillRect((int16_t)col*4, (int16_t)row*4, 4, 4, ST7735_BLACK);
}

// Loops through board data and calls functions to display/update each cell
void displayGameBoard(void) {
  for (byte row=0; row<NUMROWS; row++) {
    for (byte col=0; col<NUMCOLS; col++) {
      if(getGridChanged(row, col)) setCell(row, col, getGridItem(row, col));
    }
  }
}

// Checks if a cell is alive. Always returns false for locations outside the grid
boolean isCellAlive(int row, int col) {
  if (row < 0 || col < 0 || row >= NUMROWS || col >= NUMCOLS) {
    return false;
  }
  return getGridItem(row, col);
}

// Counts the live neighbors around a cell
byte countNeighbors(byte row, byte col) {
  byte count = 0;
  for (int rowDelta=-1; rowDelta<=1; rowDelta++) {
    for (int colDelta=-1; colDelta<=1; colDelta++) {
      // skip the center cell
      if (!(colDelta == 0 && rowDelta == 0)) {
        if (isCellAlive(rowDelta+row, colDelta+col)) {
          count++;
        }
      }
    }
  }
  return count;
}

// Runs life B3/S23 calcuations against current grid array and saves results to new grid array
// Also increases generation count and copies new grid array to current grid array once calcuations are completed
void calculateNewGameBoard() {
  memset(gameBoardChanges, 0x0, sizeof(gameBoardChanges));
  memcpy(newGameBoard, gameBoard, sizeof(newGameBoard));
  boolean cell, changed;
  unsigned long living = 0;
  for (byte row=0; row<NUMROWS; row++) {
    for (byte col=0; col<NUMCOLS; col++) {
      byte numNeighbors = countNeighbors(row, col);
      cell = getGridItem(row, col);
      changed = false;
      if (cell && (numNeighbors == 2 || numNeighbors == 3)) {
        // Any live cell with two or three live neighbours lives on to the next generation.
        living++;
      } else if (!cell && numNeighbors == 3) {
        // Any dead cell with exactly three live neighbours becomes a live cell, as if by reproduction.
        setNewGridItem(row, col, true);
        living++;
        changed=true;
      } else if(cell) {
        // Any live cell with fewer than two live neighbours dies, as if caused by under-population.
        // Any live cell with more than three live neighbours dies, as if by overcrowding.
        setNewGridItem(row, col, false);
        changed=true;
      }
      // All other cells will remain off
      if(changed) setChangedItem(row, col); // Flags cell for display update if changed
    }
  }
  gen++; // increases generation count
  if(living == lastliving) stalegens++;
  else {
    lastliving = living;
    stalegens=0;
  }
  memcpy(gameBoard, newGameBoard, sizeof(gameBoard)); // Copies newly calcuated grid to current grid array
}

// Sets up Adafruit TFT shield for use
void tftsetup() {
  tft.initR(INITR_REDTAB);
  tft.fillScreen(ST7735_BLACK);
  tft.setRotation(3);
  tft.setCursor(0,0);
  tft.setTextColor(ST7735_WHITE, ST7735_BLACK);
  tft.setTextWrap(false);
}

// Saves all 3 EEPROM blocks to 3 seperate files on an SD card
boolean sdsave() {
  if (!sd.begin(SD_CS, SPI_HALF_SPEED)) {
    sd.initErrorPrint();
    return false;
  }

  const long eeprom_locs[] = {192, 340, 540};
  const char* filenames[] = {"EEPROM1.RLE", "EEPROM2.RLE", "EEPROM3.RLE"};

  for (byte i = 0; i < 3; i++) {
    ofstream sdout(filenames[i], ios::out | ios::app);
    if (!sdout) {
      error("open failed");
      return false;
    }
    sdout << endl;
    sdout << F("x = 40, y = 29, rule = B3/S23:P40,29") << endl;
    EEPROM_readAnything(eeprom_locs[i], gameBoard);
    for (byte row=0; row<NUMROWS; row++) {
      for (byte col=0; col<NUMCOLS; col++) {
        if(getGridItem(row, col)) sdout << "o";
        else sdout << "b";
      }
      sdout << "$" << endl;
    }
    sdout << "!" << endl;
    sdout.close();
    if (!sdout) {
      error("append data failed");
      return false;
    }
  }
  return true;
}

// Current grid on-screen array editor, uses shield's joystick, press in to change a cell's status
// If changes are made, resets generation count to 1 so grid will be saved to start grid array
void gridEdit() {
  long row = 0, col = 0, prevrow = 0, prevcol = 0;
  boolean oldtcolor = tcolor, changed = false;
  tcolor = false;
  tft.fillScreen(ST7735_BLACK);
  memset(gameBoardChanges, 0xFF, sizeof(gameBoardChanges));
  displayGameBoard();
  tft.fillRect((int16_t)0, (int16_t)116, (int16_t)160, (int16_t)12, ST7735_YELLOW);
  tft.setTextColor(ST7735_BLUE, ST7735_YELLOW);
  tft.setCursor(0,119);
  tft.println(F(" Move outside grid to exit"));
  tft.drawRect((int16_t)col*4, (int16_t)row*4, 4, 4, ST7735_RED);
  delay(300);
  while ((row >= 0) && (row < NUMROWS) && (col >= 0) && (col < NUMCOLS)) {
    if((row != prevrow) || (col != prevcol)) {
      setCell(prevrow, prevcol, getGridItem(prevrow, prevcol));
      tft.drawRect((int16_t)col*4, (int16_t)row*4, 4, 4, ST7735_RED);
      prevrow = row;
      prevcol = col;
      delay(200);
    }
    switch (getButton()) {
      case bselect:
        changed = true;
        toggleGridItem(row, col);
        displayGameBoard();
        tft.drawRect((int16_t)col*4, (int16_t)row*4, 4, 4, ST7735_RED);
        delay(250);
        break;
      case bup:
        row--;
        break;
      case bdown:
        row++;
        break;
      case bleft:
        col--;
        break;
      case bright:
        col++;
        break;
      default:
        break;
    }
  }
  tcolor = oldtcolor;
  tft.setTextColor(ST7735_WHITE, ST7735_BLACK);
  tft.fillScreen(ST7735_BLACK);
  if(changed) gen = 1;
}

boolean doMenu(byte m); // Dirty fix for Arduino 1.5 - TODO: make header file

// EEPROM menu (duh?) - this is a sub-menu to the main menu
void menuEEPROM() {
  tft.fillScreen(ST7735_BLACK);
  tft.setCursor(0,0);
  byte selection = 1;
  button b;
  tft.println(F(" ------ EEPROM Menu ------ "));
  tft.println(F("  Load grid from block 1"));
  tft.println(F("  Load grid from block 2"));
  tft.println(F("  Load grid from block 3"));
  tft.println(F("  Save grid to block 1"));
  tft.println(F("  Save grid to block 2"));
  tft.println(F("  Save grid to block 3"));
  tft.println(F("  Save all blocks to SD"));
  tft.println(F(" ------------------------- "));
  tft.println(F("  left=exit, right=select"));
  tft.setCursor(0,8*selection);
  tft.write(0x1A);
  while(getButton() != bleft) {
    b = getButton();
    if (b == bright) doMenu(selection+30);
    if (b == bdown) if(selection < 7) {
      tft.setCursor(0,8*selection);
      tft.print(F("  "));
      selection++;
      tft.setCursor(0,8*selection);
      tft.write(0x1A);
      delay(250);
    }
    if (b == bup) if(selection > 1) {
      tft.setCursor(0,8*selection);
      tft.print(F("  "));
      selection--;
      tft.setCursor(0,8*selection);
      tft.write(0x1A);
      delay(250);
    }
  }
  tft.fillScreen(ST7735_BLACK);
}

// Patterns menu (duh?) - this is a sub-menu to the main menu
void menuPatterns() {
  tft.fillScreen(ST7735_BLACK);
  tft.setCursor(0,0);
  byte selection = 1;
  button b;
  tft.println(F(" ----- Patterns Menu ----- "));
  tft.println(F("  Initial Grid"));
  tft.println(F("  Gosper Glider Gun"));
  tft.println(F("  Achims P144"));
  tft.println(F("  Gliders by the dozen"));
  tft.println(F("  Heart"));
  tft.println(F("  Block and Glider"));
  tft.println(F("  The Letter 'J'"));
  tft.println(F("  987 Evolving Generations"));
  tft.println(F(" ------------------------- "));
  tft.println(F("  left=exit, right=select"));
  tft.setCursor(0,8*selection);
  tft.write(0x1A);
  while(getButton() != bleft) {
    b = getButton();
    if (b == bright) doMenu(selection+20);
    if (b == bdown) if(selection < 8) {
      tft.setCursor(0,8*selection);
      tft.print(F("  "));
      selection++;
      tft.setCursor(0,8*selection);
      tft.write(0x1A);
      delay(250);
    }
    if (b == bup) if(selection > 1) {
      tft.setCursor(0,8*selection);
      tft.print(F("  "));
      selection--;
      tft.setCursor(0,8*selection);
      tft.write(0x1A);
      delay(250);
    }
  }
  tft.fillScreen(ST7735_BLACK);
}

// Processes a menu selection
// TODO: Do this better?
boolean doMenu(byte m) {
  boolean success = false;
  unsigned int bytes;
  byte selection = m;
  while(m > 10) m = m - 10;
  switch(selection) {
    case 1:
      setupNewBoard();
      success = true;
      break;
    case 2:
      slow = !slow;
      success = true;
      break;
    case 3:
      enstats = !enstats;
      success = true;
      break;
    case 4:
      tcolor = !tcolor;
      success = true;
      break;
    case 5:
      memcpy(gameBoard, startGameBoard, sizeof(gameBoard));
      success = true;
      gen = 1;
      break;
    case 6:
      gridEdit();
      return true;
      break;
    case 7:
      menuPatterns();
      return true;
      break;
    case 8:
      menuEEPROM();
      return true;
      break;
    case 21:
      memcpy_P(gameBoard, &initialGameBoard, sizeof(gameBoard));
      success = true;
      gen = 1;
      break;
    case 22:
      memcpy_P(gameBoard, &gliderGun, sizeof(gameBoard));
      success = true;
      gen = 1;
      break;
    case 23:
      memcpy_P(gameBoard, &achimsP144, sizeof(gameBoard));
      success = true;
      gen = 1;
      break;
    case 24:
      memcpy_P(gameBoard, &GbtD, sizeof(gameBoard));
      success = true;
      gen = 1;
      break;
    case 25:
      memcpy_P(gameBoard, &heart, sizeof(gameBoard));
      success = true;
      gen = 1;
      break;
    case 26:
      memcpy_P(gameBoard, &blockAndGlider, sizeof(gameBoard));
      success = true;
      gen = 1;
      break;
    case 27:
      memcpy_P(gameBoard, &j, sizeof(gameBoard));
      success = true;
      gen = 1;
      break;
    case 28:
      memcpy_P(gameBoard, &gens987, sizeof(gameBoard));
      success = true;
      gen = 1;
      break;
    case 31:
      bytes = EEPROM_readAnything(192, gameBoard);
      if (bytes > 0) {
        success = true;
        gen = 1;
      }
      break;
    case 32:
      bytes = EEPROM_readAnything(340, gameBoard);
      if (bytes > 0) {
        success = true;
        gen = 1;
      }
      break;
    case 33:
      bytes = EEPROM_readAnything(540, gameBoard);
      if (bytes > 0) {
        success = true;
        gen = 1;
      }
      break;
    case 34:
      tft.setCursor(5,8*m);
      tft.write(0xE7);
      bytes = EEPROM_writeAnything(192, gameBoard);
      if (bytes > 0) {
        success = true;
      }
      break;
    case 35:
      tft.setCursor(5,8*m);
      tft.write(0xE7);
      bytes = EEPROM_writeAnything(340, gameBoard);
      if (bytes > 0) {
        success = true;
      }
      break;
    case 36:
      tft.setCursor(5,8*m);
      tft.write(0xE7);
      bytes = EEPROM_writeAnything(540, gameBoard);
      if (bytes > 0) {
        success = true;
      }
      break;
    case 37:
      tft.setCursor(5,8*m);
      tft.write(0xE7);
      success = sdsave();
      memcpy(gameBoard, startGameBoard, sizeof(gameBoard));
      gen = 1;
      break;
    default:
      break;
  }
  tft.setCursor(5,8*m);
  if(success) {
    tft.setTextColor(ST7735_GREEN, ST7735_BLACK);
    tft.write(0x2);
  } else {
    tft.setTextColor(ST7735_RED, ST7735_BLACK);
    tft.write(0x1);
  }
  tft.setTextColor(ST7735_WHITE, ST7735_BLACK);
  delay(250);
  return false;
}

// Does this really need comments? ^-^
void menuMain() {
  byte selection = 1;
  redomenu:
  boolean exit = false;
  tft.fillScreen(ST7735_BLACK);
  tft.setCursor(0,0);
  button b;
  tft.println(F(" ------- Main Menu ------- "));
  tft.println(F("  Generate random grid"));
  tft.println(F("  Toggle speed"));
  tft.println(F("  Toggle stats display"));
  tft.println(F("  Toggle color or B&W"));
  tft.println(F("  Reset to gen=1 grid"));
  tft.println(F("  Grid Edit Mode"));
  tft.println(F("  Patterns Menu ->"));
  tft.println(F("  EEPROM Menu ->"));
  tft.println(F(" ------------------------- "));
  tft.println(F("  left=exit, right=select"));
  tft.setCursor(0,8*selection);
  tft.write(0x1A);
  while((getButton() != bleft)) {
    b = getButton();
    if (b == bright) exit = doMenu(selection);
    if (exit) {
      if(selection == 4) return;
      else goto redomenu;
    }
    if (b == bdown) if(selection < 8) {
      tft.setCursor(0,8*selection);
      tft.print(F("  "));
      selection++;
      tft.setCursor(0,8*selection);
      tft.write(0x1A);
      delay(250);
    }
    if (b == bup) if(selection > 1) {
      tft.setCursor(0,8*selection);
      tft.print(F("  "));
      selection--;
      tft.setCursor(0,8*selection);
      tft.write(0x1A);
      delay(250);
    }
  }
  tft.fillScreen(ST7735_BLACK);
}

// Arduino setup loop, calls other setup functions
void setup() {
  Serial.begin(115200); // Startup hardware serial for any errors/debugging, doesn't slow anything down
  tftsetup();
  getresources();
  setupInitialBoard();
}

// Arduino main loop
void loop() {
  //Serial.print("loop");
  //Serial.println(millis(), DEC);
  switch (getButton()) {
    case bselect:
      menuMain(); // Pushing in on the shield's joystick loads the main menu, response time may be slow
      memset(gameBoardChanges, 0xFF, sizeof(gameBoardChanges)); // Set all cells to changed so screen displays correctly
    default:
      break;
  }
  if(stalegens > 100) {
    stalegens = 0;
    memcpy(gameBoard, startGameBoard, sizeof(gameBoard)); // Reset to first generation grid that was saved below
    gen = 1;
  }
  if(gen==1) {
    copyBoardToStart(); // Saves first generation of grid so it can be recalled/stored later
    memset(gameBoardChanges, 0xFF, sizeof(gameBoardChanges)); // Set all cells to changed so screen displays correctly
  }
  if(millis() >= lastdisp) {
    if(enstats) getresources(); // ~131ms! TODO: make faster
    //Serial.print("disp+");
    //Serial.println(millis(), DEC);
    displayGameBoard(); // There is always something to display in the grid array (~50ms)
    //Serial.print("disp-");
    //Serial.println(millis(), DEC);
    if(slow) lastdisp = millis() + 250; // Will need to adjust this if code or CPU clock changes
    else lastdisp = millis();
    //Serial.print("gen+");
    //Serial.println(millis(), DEC);
    calculateNewGameBoard(); // ~63ms
    //Serial.print("gen-");
    //Serial.println(millis(), DEC);
  }
}

