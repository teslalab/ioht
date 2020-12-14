/***************************************************************************
  This is a library for the BME680 gas, humidity, temperature & pressure sensor

  Designed specifically to work with the Adafruit BME680 Breakout
  ----> http://www.adafruit.com/products/3660

  These sensors use I2C or SPI to communicate, 2 or 4 pins are required
  to interface.

  Adafruit invests time and resources providing this open source code,
  please support Adafruit and open-source hardware by purchasing products
  from Adafruit!

  Written by Limor Fried & Kevin Townsend for Adafruit Industries.
  BSD license, all text above must be included in any redistribution
 ***************************************************************************/

#include <EasyBuzzer.h>
#include <Wire.h>
#include <SPI.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <Adafruit_FeatherOLED.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_NeoPixel.h>
#include "Adafruit_BME680.h"

#include "bsec.h" // Libreria BSEC de Bosch para BME680

#define BME_SCK 13
#define BME_MISO 12
#define BME_MOSI 11
#define BME_CS 10

#define SEALEVELPRESSURE_HPA (1013.25)

#define buzzer 27

#define pinRelay 35

Adafruit_FeatherOLED oled = Adafruit_FeatherOLED();

Adafruit_BME680 bme; // I2C
//Adafruit_BME680 bme(BME_CS); // hardware SPI
//Adafruit_BME680 bme(BME_CS, BME_MOSI, BME_MISO,  BME_SCK);

#define PIXEL_PIN   14  // Digital IO pin connected to the NeoPixels.
#define PIXEL_COUNT 6  // Number of NeoPixels

//values for alarms
#define TEMP_MAX -999
#define TEMP_MIN -999

#define HUME_MAX -999
#define HUME_MIN -999

#define SAQI_MAX -999
#define SAQI_MIN -999

#define WIFI_WARNING -999


int temp = 0;
int hume = 0;
int pres = 0;
int gas = 0;


// Declare our NeoPixel strip object:
Adafruit_NeoPixel neopixelLEDs(PIXEL_COUNT, PIXEL_PIN, NEO_GRB + NEO_KHZ800);

void setup() {


  Serial.begin(9600);         //Initialize serial port

  oled.init();                //Initialize OLED Display
  //oled.setBatteryVisible(true);

  oled.println("Starting ....");
  oled.display();

  EasyBuzzer.setPin(buzzer);  //Initialize buzzer
  testingBuzzer();

  testingRelay();

  while (!Serial);
  Serial.println(F("BME680 test"));

  if (!bme.begin()) {
    Serial.println("Could not find a valid BME680 sensor, check wiring!");
    oled.println("No BME680");
    oled.display();
    while (1);
  }

  // Set up oversampling and filter initialization
  bme.setTemperatureOversampling(BME680_OS_8X);
  bme.setHumidityOversampling(BME680_OS_2X);
  bme.setPressureOversampling(BME680_OS_4X);
  bme.setIIRFilterSize(BME680_FILTER_SIZE_3);
  bme.setGasHeater(320, 150); // 320*C for 150 ms


  // Testing buzzer
  pinMode(buzzer, OUTPUT);

  //Initialize neopixels
  neopixelLEDs.begin(); // Initialize NeoPixel strip object (REQUIRED)
  neopixelLEDs.setBrightness(128);
  neopixelLEDs.show();  // Initialize all pixels to 'off'

  //Testing LEDs
  testLEDs();
  oled.setCursor(0, 0);
  oled.println("Tesla Lab Data");
  oled.display();

}

void loop() {

  oled.clearDisplay();
  oled.setCursor(0, 0);
  oled.println("Tesla Lab Data");
  oled.display();

  // update the battery icon
  //oled.setBattery(battery);
  //oled.renderBattery();

  if (! bme.performReading()) {
    Serial.println("Failed to perform reading :(");
    oled.println("Problemas con lecturas de : (");
    return;
  }

  int temp = bme.temperature;
  int hume = bme.humidity;
  int pres = bme.pressure;

  Serial.print("Temperature = ");
  Serial.print(bme.temperature);
  Serial.println(" *C");
  oled.print("T ");
  oled.print(bme.temperature);
  oled.println(" *C");

  Serial.print("Pressure = ");
  Serial.print(bme.pressure / 100.0);
  Serial.println(" hPa");

  Serial.print("Humidity = ");
  Serial.print(bme.humidity);
  Serial.println(" %");
  oled.print("H ");
  oled.print(bme.humidity);
  oled.println(" %");

  Serial.print("Gas = ");
  Serial.print(bme.gas_resistance / 1000.0);
  Serial.println(" KOhms");
  oled.print("G ");
  oled.print(bme.gas_resistance / 1000.0);
  oled.println(" KOhms");

  Serial.print("Approx. Altitude = ");
  Serial.print(bme.readAltitude(SEALEVELPRESSURE_HPA));
  Serial.println(" m");

  // update the display with the new count
  oled.display();

  // checking values for alarms
  temp = bme.temperature;
  hume = bme.humidity;
  pres = bme.pressure;
  gas = bme.gas_resistance;


  int rojoRGB[3] = {255, 0, 0};
  int celesteRGB[3] = {0, 181, 255};
  
  checkAlarm(temp, TEMP_MAX, TEMP_MIN, "Temperatura", rojoRGB, celesteRGB);
  checkAlarm(hume, HUME_MAX, HUME_MIN, "Humedad", rojoRGB, celesteRGB);

  if (bme.temperature > 35.0) {
    beepBuzzer();
    flashLED(0);
  }

  //Probando Touch Button ARRIBA
  int lecturaBotonArriba = touchRead(4);
  Serial.print("Arriba? ");
  Serial.println(lecturaBotonArriba);
  testTouchButton(lecturaBotonArriba, 0);
  delay(50);

  //Probando Touch Button DERECHA
  int lecturaBotonDerecha = touchRead(15);
  Serial.print("Derecha? ");
  Serial.println(lecturaBotonDerecha);
  testTouchButton(lecturaBotonDerecha, 1);
  delay(50);

  //Probando Touch Button ABAJO
  int lecturaBotonAbajo = touchRead(32);
  Serial.print("Abajo? ");
  Serial.println(lecturaBotonAbajo);
  testTouchButton(lecturaBotonAbajo, 2);
  delay(50);

  //Probando Touch Button IZQUIERDA
  int lecturaBotonIzquierda = touchRead(2);
  Serial.print("Izquierda? ");
  Serial.println(lecturaBotonIzquierda);
  testTouchButton(lecturaBotonIzquierda, 3);
  delay(50);

  //Probando Touch Button CANCEL
  int lecturaBotonCancel = touchRead(12);
  Serial.print("Cancel? ");
  Serial.println(lecturaBotonCancel);
  testTouchButton(lecturaBotonCancel, 4);
  delay(50);

  //Probando Touch Button ENTER
  int lecturaBotonEnter = touchRead(13);
  Serial.print("Enter? ");
  Serial.println(lecturaBotonEnter);
  testTouchButton(lecturaBotonEnter, 5);
  delay(50);

  Serial.println();

  delay(10000);
}

void testingBuzzer() {
  EasyBuzzer.beep(840);    // Frequency in hertz(HZ).
  delay(500);
  EasyBuzzer.stopBeep();
  delay(250);
  EasyBuzzer.beep(840);    // Frequency in hertz(HZ).
  delay(500);
  EasyBuzzer.stopBeep();
  delay(250);
}

void beepBuzzer() {
  EasyBuzzer.beep(1000);    // Frequency in hertz(HZ).
  delay(500);
  EasyBuzzer.stopBeep();
  delay(250);
}

void flashLED (int led) {
  uint32_t color = neopixelLEDs.Color(255, 0, 0);   //Set color red
  for (int i = 0; i < 3; i++) { // For each pixel in strip...
    neopixelLEDs.setPixelColor(led, color); //  Set pixel's color (in RAM)
    delay(50);
    neopixelLEDs.show();                          //  Update strip to match
    delay(500);
    //Turning LEDs off
    neopixelLEDs.clear();
    neopixelLEDs.show();
    delay(250);
  }

}

void testingRelay() {
  digitalWrite(pinRelay, HIGH);
  delay(1000);
  digitalWrite(pinRelay, LOW);
  delay(1000);
}

void testLEDs() {
  for (int i = 0; i < neopixelLEDs.numPixels(); i++) { // For each pixel in strip...
    neopixelLEDs.setPixelColor(i, neopixelLEDs.Color(255, 0, 0)); //  Set pixel's color (in RAM)
    neopixelLEDs.show();                          //  Update strip to match
    delay(250);
    neopixelLEDs.setPixelColor(i, neopixelLEDs.Color(0, 255, 0)); //  Set pixel's color (in RAM)
    neopixelLEDs.show();                          //  Update strip to match
    delay(250);
    neopixelLEDs.setPixelColor(i, neopixelLEDs.Color(0, 0, 255)); //  Set pixel's color (in RAM)
    neopixelLEDs.show();                          //  Update strip to match
    delay(250);
  }
  //Turning LEDs off
  neopixelLEDs.clear();
  neopixelLEDs.show();
  delay(250);
}

void testTouchButton (int dataTouchButton, int LED) {
  if (dataTouchButton < 12) {
    neopixelLEDs.setPixelColor(LED, neopixelLEDs.Color(0, 0, 255)); //  Set pixel's color (in RAM)
    neopixelLEDs.show();                          //  Update strip to match
    delay(1000);
    //Turning LEDs off
    neopixelLEDs.clear();
    neopixelLEDs.show();
    delay(250);
  }
}

void setLedsRGB(int R, int G, int B, int last_led, int delay1) {

  for (int i = 0; i < last_led; i++) {
    neopixelLEDs.setPixelColor(i, neopixelLEDs.Color(255, 0, 0));
  }

  neopixelLEDs.show();
  delay(delay1);
  neopixelLEDs.clear();
  neopixelLEDs.show();
}

void setBuzzer(int frequency, int count, int delay1) {
  for (int i = 0; i < count; i++) {
    EasyBuzzer.beep(frequency);
    delay(delay1);
    EasyBuzzer.stopBeep();
    delay(delay1);
  }
}

void setLedsAndBuzzer(int colors[3], int last_led, int frequency, int count, int delay1) {

  for (int i = 0; i < count; i++) {
    for (int i = 0; i < last_led; i++) {
      neopixelLEDs.setPixelColor(i, neopixelLEDs.Color(colors[0], colors[1], colors[2]));
    }
    neopixelLEDs.show();
    delay(delay1);
    EasyBuzzer.beep(frequency);
    delay(delay1);
    neopixelLEDs.clear();
    neopixelLEDs.show();
    EasyBuzzer.stopBeep();
    delay(delay1);
  }
}

void checkAlarm(int value, int max, int min, char nombre[], int colorsHigh[3], int colorsLow[3]) {
  if (max != -999) {
    if (value > max) {

      oled.clearDisplay();
      oled.setCursor(0, 0);
      oled.print("Niveles de ");
      oled.print(nombre);
      oled.println(" altos");

      oled.print("Valor: ");
      oled.println(value);

      oled.display();

      setLedsAndBuzzer(colorsHigh, PIXEL_COUNT, 1000, 6, 100);
    }
  }
  if (min != -999) {
    if (value < min) {
      oled.clearDisplay();
      oled.setCursor(0, 0);
      oled.print("Niveles de ");
      oled.print(nombre);
      oled.println(" bajos");

      oled.print("Valor: ");
      oled.println(value);

      oled.display();

      setLedsAndBuzzer(colorsLow, PIXEL_COUNT, 500, 3, 200);
    }
  }
}
