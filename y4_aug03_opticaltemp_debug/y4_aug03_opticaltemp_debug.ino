#include <Wire.h>
#include "Adafruit_TCS34725.h"
#include <Adafruit_PCT2075.h>

#include <SPI.h>
//#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SH110X.h>

Adafruit_SH1107 display = Adafruit_SH1107(64, 128, &Wire);

// OLED FeatherWing buttons map to different pins depending on board:
#if defined(ESP8266)
#define BUTTON_A  0
#define BUTTON_B 16
#define BUTTON_C  2
#elif defined(ESP32)
#define BUTTON_A 15
#define BUTTON_B 32
#define BUTTON_C 14
#elif defined(ARDUINO_STM32_FEATHER)
#define BUTTON_A PA15
#define BUTTON_B PC7
#define BUTTON_C PC5
#elif defined(TEENSYDUINO)
#define BUTTON_A  4
#define BUTTON_B  3
#define BUTTON_C  8
#elif defined(ARDUINO_NRF52832_FEATHER)
#define BUTTON_A 31
#define BUTTON_B 30
#define BUTTON_C 27
#else // 32u4, M0, M4, nrf52840 and 328p
#define BUTTON_A  9
#define BUTTON_B  6
#define BUTTON_C  5
#endif

// Color definitions
#define BLACK    0x0000
#define BLUE     0x001F
#define RED      0xF800
#define GREEN    0x07E0
#define CYAN     0x07FF
#define MAGENTA  0xF81F
#define YELLOW   0xFFE0
#define WHITE    0xFFFF

/* Connect SCL    to analog 5
   Connect SDA    to analog 4
   Connect VDD    to 3.3V DC
   Connect GROUND to common ground */

// Initialize ams light sensor
/* Initialise with default values (int time = 2.4ms, gain = 1x) */
// Adafruit_TCS34725 tcs = Adafruit_TCS34725();
/* Initialise with specific int time and gain values */
Adafruit_TCS34725 tcs = Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_2_4MS, TCS34725_GAIN_60X);
// set to 2.4ms integration time
// NOTE: Adafruit_PCT2075.cpp implements a 3ms delay, not 2.4ms

//Initialize LM75 temperature sensor
Adafruit_PCT2075 PCT2075;

uint16_t r, g, b, c, readingsum;
unsigned long starttime, lux, tempraw, tempread;
float temp, tempconvert;
int state, LEDpin, threshold, readthreshold;
bool lighted, primary;

void setup(void) {

  LEDpin = A0; // LED pin on the Feather, change this
  threshold = 200; // for a single reading (initial)
  readthreshold = 4000; // for a 5x reading (comms)
  lighted = false;
  primary = true; // select if this fiber is primary or secondary

  pinMode(LEDpin, OUTPUT);
  digitalWrite(LEDpin, LOW);

  Serial.begin(115200); // USB for debug
//  while (!Serial) {delay(1);}  // wait until debug opened to continue

  PCT2075 = Adafruit_PCT2075(); // initialize temp
  delay(500);
  if (!PCT2075.begin(72)) {
    Serial.println("Couldn't find PCT2075 chip");  // kickout if no temp sensor at address 1001000b
    while (1);
  }
  if (tcs.begin()) {
    Serial.println("Found light sensor"); // kickout if no light sensor
  } else {
    Serial.println("No TCS34725 found");
    while (1);
  }

  // preliminary temperature reading:
  temp = PCT2075.getTemperature();
  Serial.print("Starting temperature is: "); Serial.print(temp); Serial.println("??C");

  display.begin(0x3C, true); // OLED address 0x3C default
  Serial.println("OLED started");
  // Show image buffer on the display hardware.
  // Since the buffer is intialized with an Adafruit splashscreen
  // internally, this will display the splashscreen.
  display.display();
  delay(1000);
  // Clear the buffer.
  display.clearDisplay();
  display.display();
  display.setRotation(1);
  display.setTextSize(1);
  display.setTextColor(SH110X_WHITE);
  display.setCursor(0, 0);
  Serial.println("entering state 0");
  display.print("OFF");
  display.display();
  display.setCursor(0, 0);

}

void loop(void) {
  // state 0 = waiting for start signal
  if (state == 0) {
    // continuously monitor the light sensor

    // if was dark, check to see if lighted now
    if (lighted == false) {
      tcs.getRawData(&r, &g, &b, &c);
      if (c > threshold) {
        starttime = millis();
        lighted = true;
        // if primary, turn on LED to signal to secondary
        if (primary) {
          digitalWrite(LEDpin, HIGH);
        }

        display.clearDisplay();
        display.setCursor(0,0);
        display.setTextSize(1);
        display.print("Light Detected");
        display.display();
      }
    }
    // if was lighted, check to see if still lighted
    else {
      tcs.getRawData(&r, &g, &b, &c);
      // if still lighted
      if (c > threshold) {

        // if primary, turn on LED to signal to secondary
        if (primary) {
          digitalWrite(LEDpin, HIGH);
        }

        // if there has been constant light for >3 seconds, kick on
        if (millis() - starttime > 3000) {
          state = 1;
          Serial.println("state set to 1");
          lighted = false;
          // LED off happens after this, to ensure detection by secondary
        }
      } else {

        // if primary, turn off LED to signal to secondary
        if (primary) {
          digitalWrite(LEDpin, LOW);
        }
        lighted = false;

        display.clearDisplay();
        display.setCursor(0,0);
        display.print("Light no longer detected");
        display.display();
      }
    }
  }

  // state 1 = collecting temperature
  else if (state == 1) {
    // if primary, stop signaling to secondary
    if (primary) {
      delay(5);
      digitalWrite(LEDpin, LOW);
    }

    display.setCursor(0,0);
    display.clearDisplay();
    display.print("Collecting temperature in 3...");
    display.display();
    delay(1000);
    display.print("2...");
    display.display();
    delay(1000);
    display.print("1...");
    display.display();
    delay(1000);

    // take temperature reading
    temp = PCT2075.getTemperature();
    Serial.print("Temperature detected: "); Serial.print(temp); Serial.println("??C");
    tempconvert = temp / 0.125;
    tempraw = (long)tempconvert;
    Serial.print("binary conversion: "); Serial.println(tempraw, BIN);
    state = 2;

    display.clearDisplay();
    display.setCursor(0, 0);
    display.setTextSize(2);
    display.print(String(temp, 1));
    display.drawCircle(56,3,2,SH110X_WHITE);  //  display.print((char)247); 
    display.println(" C");
    display.display();

    display.setTextSize(1);
    display.print("Transmit in 3..");
    display.display();
    delay(1000);
    display.print("2..");
    display.display();
    delay(1000);
    display.println("1..");
    display.display();
    delay(1000);
  }

  // state 2 = communicating data (simultaneous read/write)
  else if (state == 2) {
    // set LED state, then poll the sensor 5x
    for (int i = 0; i <= 15; i++) {
      // either turn the LED on or off
      digitalWrite(LEDpin, bitRead(tempraw, i));
      // flush the light sensor reading
      readingsum = 0;
      tcs.getRawData(&r, &g, &b, &c); readingsum += c;
      tcs.getRawData(&r, &g, &b, &c); readingsum += c;
      tcs.getRawData(&r, &g, &b, &c); readingsum += c;
      tcs.getRawData(&r, &g, &b, &c); readingsum += c;
      tcs.getRawData(&r, &g, &b, &c); readingsum += c;
      tcs.getRawData(&r, &g, &b, &c); readingsum += c;
      tcs.getRawData(&r, &g, &b, &c); readingsum += c;
      tcs.getRawData(&r, &g, &b, &c); readingsum += c;
      tcs.getRawData(&r, &g, &b, &c); readingsum += c;
      tcs.getRawData(&r, &g, &b, &c); readingsum += c;
      Serial.print("bit "); Serial.print(i); Serial.print(" = raw "); Serial.print(readingsum, DEC); Serial.print(" = ");
      // if sufficient light input, put a one at that index
      if (readingsum > readthreshold) {
        Serial.println("1b");
        bitWrite(tempread, i, 1);
      }
      // otherwise, zero at that index
      else {
        Serial.println("0b");
        bitWrite(tempread, i, 0);
      }
    }
    state = 3;
    digitalWrite(LEDpin, LOW);

    display.setTextSize(2);
    display.print(String(tempread * 0.125, 1));
    display.drawCircle(56,27,2,SH110X_WHITE); 
    display.print(" C");
    display.display();
    
    Serial.println("Transmitted, waiting before displaying");
    delay(3000);
  }

  // state 3 = displaying higher
  else if (state == 3) {
    display.setCursor(80,40);
    
    if (tempraw < tempread) {
      display.print("LO");
      display.display();    
      delay(3000);
    } else {
      if (tempraw > tempread) {
        display.print("HI");
        display.display();
      }
      else { // equal reads
        display.print("SAME");
        display.display();
      }
      
      digitalWrite(LEDpin, HIGH); delay(500); digitalWrite(LEDpin, LOW); delay(500);
      digitalWrite(LEDpin, HIGH); delay(500); digitalWrite(LEDpin, LOW); delay(500);
      digitalWrite(LEDpin, HIGH); delay(500); digitalWrite(LEDpin, LOW); delay(500);
    }
    
    state = 4;
    Serial.println("Entering state 4");
  }
  // state 4 = off
  else if (state == 4) {

    delay(5000);
    display.clearDisplay();
    display.display();

    state = 0;
    Serial.println("Back to state 0");
  }

}
