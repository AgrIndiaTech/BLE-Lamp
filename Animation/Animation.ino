#include <string.h>
#include <Arduino.h>
#include <SPI.h>
#if not defined (_VARIANT_ARDUINO_DUE_X_) && not defined (_VARIANT_ARDUINO_ZERO_)
  #include <SoftwareSerial.h>
#endif

#include "Adafruit_BLE.h"
#include "Adafruit_BluefruitLE_SPI.h"
#include "Adafruit_BluefruitLE_UART.h"

#include "BluefruitConfig.h"

#include <Adafruit_NeoPixel.h>

/*=========================================================================

    FACTORYRESET_ENABLE       Perform a factory reset when running this sketch
   
                              Enabling this will put your Bluefruit LE module
                              in a 'known good' state and clear any config
                              data set in previous sketches or projects, so
                              running this at least once is a good idea.
   
                              When deploying your project, however, you will
                              want to disable factory reset by setting this
                              value to 0.  If you are making changes to your
                              Bluefruit LE device via AT commands, and those
                              changes aren't persisting across resets, this
                              is the reason why.  Factory reset will erase
                              the non-volatile memory where config data is
                              stored, setting it back to factory default
                              values.
    -----------------------------------------------------------------------*/
    #define FACTORYRESET_ENABLE     1

    #define PIN                     6
    #define NUMPIXELS               24
    #define BRIGHTNESS              100
/*=========================================================================*/

Adafruit_NeoPixel pixel = Adafruit_NeoPixel(NUMPIXELS, 6); // NeoPixel Object

// Create the bluefruit object, either software serial
//Hardware SPI, using SCK/MOSI/MISO hardware SPI pins and then user selected CS/IRQ/RST
Adafruit_BluefruitLE_SPI ble(BLUEFRUIT_SPI_CS, BLUEFRUIT_SPI_IRQ, BLUEFRUIT_SPI_RST);

// A small helper
void error(const __FlashStringHelper*err) {
  Serial.println(err);
  while (1);
}

// function prototypes over in packetParser.cpp
uint8_t readPacket(Adafruit_BLE *ble, uint16_t timeout);
float parsefloat(uint8_t *buffer);
void printHex(const uint8_t * data, const uint32_t numBytes);

// the packet buffer
extern uint8_t packetbuffer[];

//additional variables

//Color
    uint8_t red = 255;
    uint8_t green = 255;
    uint8_t blue = 255;
    //uint8_t animationState = 1;
    uint8_t animationState = 0;

void setup(void)
{
  //while (!Serial);  // required for Flora & Micro
  delay(500);

  // turn off neopixel
  pixel.setBrightness(BRIGHTNESS);
  pixel.begin(); // This initializes the NeoPixel library.

  for(uint8_t i=0; i<NUMPIXELS; i++) {
    pixel.setPixelColor(i, pixel.Color(0,0,0)); // off
  }
  colorWipe(pixel.Color(255, 255, 255), 15); 
  colorWipe(pixel.Color(0, 0, 0), 15); 
  pixel.show();

  Serial.begin(115200);
  Serial.println(F("Adafruit Bluefruit Neopixel Color Picker Example"));
  Serial.println(F("------------------------------------------------"));

  /* Initialise the module */
  Serial.print(F("Initialising the Bluefruit LE module: "));

  if ( !ble.begin(VERBOSE_MODE) )
  {
    error(F("Couldn't find Bluefruit, make sure it's in CoMmanD mode & check wiring?"));
  }
  Serial.println( F("OK!") );

  if ( FACTORYRESET_ENABLE )
  {
    /* Perform a factory reset to make sure everything is in a known state */
    Serial.println(F("Performing a factory reset: "));
    if ( ! ble.factoryReset() ){
      error(F("Couldn't factory reset"));
    }
  }

  /* Disable command echo from Bluefruit */
  ble.echo(false);

  Serial.println("Requesting Bluefruit info:");
  /* Print Bluefruit information */
  ble.info();

  Serial.println(F("Please use Adafruit Bluefruit LE app to connect in Controller mode"));
  Serial.println(F("Then activate/use the sensors, color picker, game controller, etc!"));
  Serial.println();

  ble.verbose(false);  // debug info is a little annoying after this point!

  /* Wait for connection */
  while (! ble.isConnected()) {
      delay(500);
  }

  Serial.println(F("***********************"));

  // Set Bluefruit to DATA mode
  Serial.println( F("Switching to DATA mode!") );
  ble.setMode(BLUEFRUIT_MODE_DATA);

  Serial.println(F("***********************"));
}

/**************************************************************************/
/*!
    @brief  Constantly poll for new command or response data
*/
/**************************************************************************/
void loop(void) {
  /* Wait for new data to arrive */
  uint8_t len = readPacket(&ble, BLE_READPACKET_TIMEOUT);
  if (len == 0) return;

  /* Got a packet! */
  // printHex(packetbuffer, len);

  // Color
  if (packetbuffer[1] == 'C') {
    uint8_t red = packetbuffer[2];
    uint8_t green = packetbuffer[3];
    uint8_t blue = packetbuffer[4];
    Serial.print ("RGB #");
    if (red < 0x10) Serial.print("0");
    Serial.print(red, HEX);
    if (green < 0x10) Serial.print("0");
    Serial.print(green, HEX);
    if (blue < 0x10) Serial.print("0");
    Serial.println(blue, HEX);

    for(uint8_t i=0; i<NUMPIXELS; i++) {
      pixel.setPixelColor(i, pixel.Color(red,green,blue));
    }
    pixel.show(); // This sends the updated pixel color to the hardware.
  }

  // Buttons
  if (packetbuffer[1] == 'B') {
 
    uint8_t buttnum = packetbuffer[2] - '0';
    boolean pressed = packetbuffer[3] - '0';
    Serial.print ("Button "); Serial.print(buttnum);
    //animationState = buttnum;
    if (pressed) {
      Serial.println(" pressed");
      animationState = buttnum;
    } else {
      Serial.println(" released");
    }

  switch (animationState) {
            case 1:
                authenticateTouchPoint();
                allColorsWipe(1);
                break;
            case 2:
                allColorsPulse(1, 125);            
                break;
            case 3:
                rainbow(20);                              
                break;
            case 4:
                rainbowCycle(10);
                break;
            case 5:
                rainbowRing();                
                break;
            case 6:
                middleFill(pixel.Color(0, 255, 0), 100);
                sideFill(pixel.Color(255, 0, 0), 100);
                break;
            case 7:
                randomColorFill(20);
                break;
            case 8:            
                RGBLoop();
                break;
  }
  clearpixel();
  animationState = 0;
 }
}

// Fill the dots one after the other with a color
void colorWipe(uint32_t c, uint8_t wait) {
  for(uint16_t i=0; i<pixel.numPixels(); i++) {
      pixel.setPixelColor(i, c);
      pixel.show();
      delay(wait);
  }
}

void rainbow(uint8_t wait) {
  uint16_t i, j;

  for(j=0; j<256; j++) {
    for(i=0; i<pixel.numPixels(); i++) {
      pixel.setPixelColor(i, Wheel((i+j) & 255));
    }
    pixel.show();
    delay(wait);
  }
}

// Slightly different, this makes the rainbow equally distributed throughout
void rainbowCycle(uint8_t wait) {
  uint16_t i, j;

  for(j=0; j<256*5; j++) { // 5 cycles of all colors on wheel
    for(i=0; i< pixel.numPixels(); i++) {
      pixel.setPixelColor(i, Wheel(((i * 256 / pixel.numPixels()) + j) & 255));
    }
    pixel.show();
    delay(wait);
  }
}

// Input a value 0 to 255 to get a color value.
// The colours are a transition r - g - b - back to r.
uint32_t Wheel(byte WheelPos) {
  WheelPos = 255 - WheelPos;
  if(WheelPos < 85) {
    return pixel.Color(255 - WheelPos * 3, 0, WheelPos * 3);
  }
  if(WheelPos < 170) {
    WheelPos -= 85;
    return pixel.Color(0, WheelPos * 3, 255 - WheelPos * 3);
  }
  WheelPos -= 170;
  return pixel.Color(WheelPos * 3, 255 - WheelPos * 3, 0);
}

void whiteCycle() {
  for(int i=NUMPIXELS-1;i>=0;i--){
      pixel.setPixelColor((i+8)%NUMPIXELS, pixel.Color(63, 63, 63)); //change RGB color value here
      pixel.setPixelColor((i+7)%NUMPIXELS, pixel.Color(63, 63, 63)); //change RGB color value here
      pixel.setPixelColor((i+6)%NUMPIXELS, pixel.Color(95, 95, 95)); //change RGB color value here
      pixel.setPixelColor((i+5)%NUMPIXELS, pixel.Color(255, 255, 255)); //change RGB color value here
      pixel.setPixelColor((i+4)%NUMPIXELS, pixel.Color(255, 255, 255)); //change RGB color value here
      pixel.setPixelColor((i+3)%NUMPIXELS, pixel.Color(255, 255, 255)); //change RGB color value here
      pixel.setPixelColor((i+2)%NUMPIXELS, pixel.Color(95, 95, 95)); //change RGB color value here
      pixel.setPixelColor((i+1)%NUMPIXELS, pixel.Color(63, 63, 63)); //change RGB color value here
      pixel.setPixelColor(i%NUMPIXELS, pixel.Color(63, 63, 63)); //change RGB color value here
      pixel.show();
      if(NUMPIXELS == 16) { delay(185); } //correct timing but speed makes it look spotty instead of a smooth moving ring
      if(NUMPIXELS  == 24) { delay(115); } //delay(85); } //maybe try for 24 leds. 115 is exact timing, 85 looks nicer.
      clearRing();
   }
}

void clearRing(){
  for(int i=0;i<NUMPIXELS;i++){
      pixel.setPixelColor(i, pixel.Color(0, 0, 0)); //change RGB color value here
   }
}

void clearpixel(){
  for(int i=0;i<NUMPIXELS;i++){
      pixel.setPixelColor(i, pixel.Color(0, 0, 0)); //change RGB color value here
   }
   pixel.show();
}


void clearColor(int r, int g, int b, uint8_t wait) {
  for(int i=NUMPIXELS-1;i>=0;i--){
      pixel.setPixelColor(i, pixel.Color(r, g, b)); //change RGB color value here
      pixel.show();
      delay(wait);
   }
}

void authenticateTouchPoint() {
  whiteSpeed(15, 4);
  whiteSpeed(10, 4);
  colorWipe(pixel.Color(127, 127, 127), 0);
  colorWipe(pixel.Color(255, 255, 255), 0);
  delay(1000);
  clearColor(0,0,0,35);
  delay(250);
  pixel.show();
}

void authenticateFastPass() {
  whiteSpeed(15, 1);
}

void whiteSpeed(int wait, int cycles){
  for(int i=(NUMPIXELS*cycles)-1;i>=0;i--){
      pixel.setPixelColor((i+8)%NUMPIXELS, pixel.Color(63, 63, 63)); //change RGB color value here
      pixel.setPixelColor((i+7)%NUMPIXELS, pixel.Color(63, 63, 63)); //change RGB color value here
      pixel.setPixelColor((i+6)%NUMPIXELS, pixel.Color(95, 95, 95)); //change RGB color value here
      pixel.setPixelColor((i+5)%NUMPIXELS, pixel.Color(255, 255, 255)); //change RGB color value here
      pixel.setPixelColor((i+4)%NUMPIXELS, pixel.Color(255, 255, 255)); //change RGB color value here
      pixel.setPixelColor((i+3)%NUMPIXELS, pixel.Color(255, 255, 255)); //change RGB color value here
      pixel.setPixelColor((i+2)%NUMPIXELS, pixel.Color(95, 95, 95)); //change RGB color value here
      pixel.setPixelColor((i+1)%NUMPIXELS, pixel.Color(63, 63, 63)); //change RGB color value here
      pixel.setPixelColor(i%NUMPIXELS, pixel.Color(63, 63, 63)); //change RGB color value here
      pixel.show();
      delay(wait);
      clearRing();
   }
}

void transitionToWhiteCycle(int r, int g, int b, uint8_t wait) {
  for(int i=NUMPIXELS-1;i>8;i--){
      pixel.setPixelColor(i, pixel.Color(r, g, b)); //change RGB color value here
      pixel.show();
      delay(wait);
   }
}

void colorPulse(int r, int g, int b, uint8_t wait) {
  clearRing();
  pixel.show();
  delay(wait);
  for(int i=NUMPIXELS-1;i>=0;i--) {
    pixel.setPixelColor(i, pixel.Color(r/4, g/4, b/4));
    pixel.show();
  }
  delay(wait);
  for(int i=NUMPIXELS-1;i>=0;i--) {
    pixel.setPixelColor(i, pixel.Color(r/3, g/3, b/3));
    pixel.show();
  }
  delay(wait);
  for(int i=NUMPIXELS-1;i>=0;i--) {
    pixel.setPixelColor(i, pixel.Color(r/2, g/2, b/2));
    pixel.show();
  }
  delay(wait);
  for(int i=NUMPIXELS-1;i>=0;i--) {
    pixel.setPixelColor(i, pixel.Color(r, g, b));
    pixel.show();
  }
  delay(10*wait);
  for(int i=NUMPIXELS-1;i>=0;i--) {
    pixel.setPixelColor(i, pixel.Color(r/2, g/2, b/2));
    pixel.show();
  }
  delay(wait);
  for(int i=NUMPIXELS-1;i>=0;i--) {
    pixel.setPixelColor(i, pixel.Color(r/3, g/3, b/3));
    pixel.show();
  }
  delay(wait);
  for(int i=NUMPIXELS-1;i>=0;i--) {
    pixel.setPixelColor(i, pixel.Color(r/4, g/4, b/4));
    pixel.show();
  }
  delay(wait);
}

void rainbowRing() {
   int i = 0;
   while(i<NUMPIXELS*4) {
      pixel.setPixelColor(i%NUMPIXELS, pixel.Color(255, 255, 255)); //change RGB color value here
      pixel.setPixelColor((i+1)%NUMPIXELS, pixel.Color(255, 5, 180)); //change RGB color value here
      pixel.setPixelColor((i+2)%NUMPIXELS, pixel.Color(255, 0, 0)); //change RGB color value here
      pixel.setPixelColor((i+3)%NUMPIXELS, pixel.Color(255, 150, 0)); //change RGB color value here
      pixel.setPixelColor((i+4)%NUMPIXELS, pixel.Color(255, 255, 5)); //change RGB color value here
      pixel.setPixelColor((i+5)%NUMPIXELS, pixel.Color(0, 255, 0)); //change RGB color value here
      pixel.setPixelColor((i+6)%NUMPIXELS, pixel.Color(0, 0, 255)); //change RGB color value here
      pixel.setPixelColor((i+7)%NUMPIXELS, pixel.Color(135, 10, 215)); //change RGB color value here
      pixel.setPixelColor((i+8)%NUMPIXELS, pixel.Color(255, 255, 255)); //change RGB color value here
      pixel.setPixelColor((i+9)%NUMPIXELS, pixel.Color(255, 5, 180)); //change RGB color value here
      pixel.setPixelColor((i+10)%NUMPIXELS, pixel.Color(255, 0, 0)); //change RGB color value here
      pixel.setPixelColor((i+11)%NUMPIXELS, pixel.Color(255, 150, 0)); //change RGB color value here
      pixel.setPixelColor((i+12)%NUMPIXELS, pixel.Color(255, 255, 5)); //change RGB color value here
      pixel.setPixelColor((i+13)%NUMPIXELS, pixel.Color(0, 255, 0)); //change RGB color value here
      pixel.setPixelColor((i+14)%NUMPIXELS, pixel.Color(0, 0, 255)); //change RGB color value here
      pixel.setPixelColor((i+15)%NUMPIXELS, pixel.Color(135, 10, 215)); //change RGB color value here
      pixel.setPixelColor((i+16)%NUMPIXELS, pixel.Color(255, 255, 255)); //change RGB color value here
      pixel.setPixelColor((i+17)%NUMPIXELS, pixel.Color(255, 5, 180)); //change RGB color value here
      pixel.setPixelColor((i+18)%NUMPIXELS, pixel.Color(255, 0, 0)); //change RGB color value here
      pixel.setPixelColor((i+19)%NUMPIXELS, pixel.Color(255, 150, 0)); //change RGB color value here
      pixel.setPixelColor((i+20)%NUMPIXELS, pixel.Color(255, 255, 5)); //change RGB color value here
      pixel.setPixelColor((i+21)%NUMPIXELS, pixel.Color(0, 255, 0)); //change RGB color value here
      pixel.setPixelColor((i+22)%NUMPIXELS, pixel.Color(0, 0, 255)); //change RGB color value here
      pixel.setPixelColor((i+23)%NUMPIXELS, pixel.Color(135, 10, 215)); //change RGB color value here
      i++;
      pixel.show();
      delay(75);
   }
}

void allColorsWipe(int i){   
    //if(i<2){
      colorWipe(pixel.Color(255, 5, 180), 35);  //pink
      colorWipe(pixel.Color(255, 0, 0), 35);    //red
      colorWipe(pixel.Color(255, 150, 0), 35);  //orange
      colorWipe(pixel.Color(255, 255, 5), 35);  //yellow
      colorWipe(pixel.Color(0, 255, 0), 35);    //green
      colorWipe(pixel.Color(0, 0, 255), 35);    //blue
      colorWipe(pixel.Color(135, 10, 215), 35); //purple
      //i++;
      //allColorsWipe(i);
    //}
}

void allColorsPulse(int i, int wait){   
    //if(i<3){
      colorPulse(255, 255, 255, wait);
      colorPulse(255, 5, 180, wait);  //pink
      colorPulse(255, 0, 0, wait);    //red
      colorPulse(255, 150, 0, wait);  //orange
      colorPulse(255, 255, 5, wait);  //yellow
      colorPulse(0, 255, 0, wait);    //green
      colorPulse(0, 0, 255, wait);    //blue
      colorPulse(135, 10, 215, wait); //purple
      //i++;
      //allColorsPulse(i, wait);
    //}
}

// gradually fill up the pixel with random colors
void randomColorFill(uint8_t wait) {
  clearpixel();

  for(uint16_t i=0; i<pixel.numPixels(); i++) { // iterate over every LED of the pixel
    int r = random(0,255); // generate a random color
    int g = random(0,255);
    int b = random(0,255);

    for(uint16_t j=0; j<pixel.numPixels()-i; j++) { // iterate over every LED of the pixel, that hasn't lit up yet
      pixel.setPixelColor(j-1, pixel.Color(0, 0, 0)); // turn previous LED off
      pixel.setPixelColor(j, pixel.Color(r, g, b)); // turn current LED on
      pixel.show(); // apply the colors
      delay(wait);
    }
  }
}

// pick a random LED to light up until entire pixel is lit
void randomPositionFill(uint32_t c, uint8_t wait) {
  clearpixel();

  int used[pixel.numPixels()]; // array to keep track of lit LEDs
  int lights = 0; // counter

  for(int i = 0; i<pixel.numPixels(); i++){ // fill array with 0
    used[i] = 0;
  }

  while(lights<pixel.numPixels()-1) {
    int j = random(0,pixel.numPixels()-1); // pick a random LED
    if(used[j] != 1){ // if LED not already lit, proceed
      pixel.setPixelColor(j, c);
      used[j] = 1; // update array to remember it is lit
      lights++;
      pixel.show(); // display
      delay(wait);
    }
  }
}

// Light up the pixel starting from the middle
void middleFill(uint32_t c, uint8_t wait) {
  clearpixel();
  delay(wait);

  for(uint16_t i=0; i<(pixel.numPixels()/2); i++) { // start from the middle, lighting an LED on each side
    pixel.setPixelColor(pixel.numPixels()/2 + i, c);
    pixel.setPixelColor(pixel.numPixels()/2 - i, c);
    pixel.show();
    delay(wait);
  }
  pixel.setPixelColor(0, c);
  pixel.show();
  delay(wait);

  for(uint16_t i=0; i<(pixel.numPixels()/2); i++) { // reverse
    pixel.setPixelColor(i, pixel.Color(0, 0, 0));
    pixel.setPixelColor(pixel.numPixels() - i, pixel.Color(0, 0, 0));
    pixel.show();
    delay(wait);
  }
  clearpixel();
}

// Light up the pixel starting from the sides
void sideFill(uint32_t c, uint8_t wait) {
  clearpixel();
  delay(wait);

  for(uint16_t i=0; i<(pixel.numPixels()/2); i++) { // fill pixel from sides to middle
    pixel.setPixelColor(i, c);
    pixel.setPixelColor(pixel.numPixels() - i, c);
    pixel.show();
    delay(wait);
  }

  pixel.setPixelColor(pixel.numPixels()/2, c);
  pixel.show();
  delay(wait);

  for(uint16_t i=0; i<(pixel.numPixels()/2); i++) { // reverse
    pixel.setPixelColor(pixel.numPixels()/2 + i, pixel.Color(0, 0, 0));
    pixel.setPixelColor(pixel.numPixels()/2 - i, pixel.Color(0, 0, 0));
    pixel.show();
    delay(wait);
  }
  clearpixel();
}

/*Extra stuff*/
void showStrip() {
 #ifdef ADAFRUIT_NEOPIXEL_H 
   // NeoPixel
   pixel.show();
 #endif
 #ifndef ADAFRUIT_NEOPIXEL_H
   // FastLED
   FastLED.show();
 #endif
}

void setPixel(int Pixel, byte red, byte green, byte blue) {
 #ifdef ADAFRUIT_NEOPIXEL_H 
   // NeoPixel
   pixel.setPixelColor(Pixel, pixel.Color(red, green, blue));
 #endif
 #ifndef ADAFRUIT_NEOPIXEL_H 
   // FastLED
   leds[Pixel].r = red;
   leds[Pixel].g = green;
   leds[Pixel].b = blue;
 #endif
}

void setAll(byte red, byte green, byte blue) {
  for(int i = 0; i < NUMPIXELS; i++ ) {
    setPixel(i, red, green, blue); 
  }
  showStrip();
}

void RGBLoop(){
  for(int j = 0; j < 3; j++ ) { 
    // Fade IN
    for(int k = 0; k < 256; k++) { 
      switch(j) { 
        case 0: setAll(k,0,0); break;
        case 1: setAll(0,k,0); break;
        case 2: setAll(0,0,k); break;
      }
      showStrip();
      delay(3);
    }
    // Fade OUT
    for(int k = 255; k >= 0; k--) { 
      switch(j) { 
        case 0: setAll(k,0,0); break;
        case 1: setAll(0,k,0); break;
        case 2: setAll(0,0,k); break;
      }
      showStrip();
      delay(3);
    }
  }
}
