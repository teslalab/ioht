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
const int TEMP_MAX = -999;
const int TEMP_MIN = -999;

const int HUME_MAX = -999;
const int HUME_MIN = -999;

const int SAQI_MAX = -999;
const int SAQI_MIN = -999;

const int WIFI_WARNING = -999;


double temp, hume, pres, sAQI = 0;

#include "bsec.h" // Libreria BSEC de Bosch para BME680

//************** Funciones para el Sensor BME680 **************
//Declaración de funciones de ayuda
void checkIaqSensorStatus(void);
// Creamos un objeto con clase BSEC Software
Bsec iaqSensor;
String output, output2;


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

  output2 = "";
  if (iaqSensor.run()) { // If new data is available
    output2 += ", " + String(iaqSensor.rawTemperature);
    output2 += ", " + String(iaqSensor.pressure);
    output2 += ", " + String(iaqSensor.rawHumidity);
    output2 += ", " + String(iaqSensor.gasResistance);
    output2 += ", " + String(iaqSensor.iaq);
    output2 += ", " + String(iaqSensor.iaqAccuracy);
    output2 += ", " + String(iaqSensor.temperature);
    output2 += ", " + String(iaqSensor.humidity);
    output2 += ", " + String(iaqSensor.staticIaq);
    output2 += ", " + String(iaqSensor.co2Equivalent);
    output2 += ", " + String(iaqSensor.breathVocEquivalent);
    Serial.println(output2);     
  } else {
      checkIaqSensorStatus();
  }

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

  temp = bme.temperature;
  hume = bme.humidity;
  pres = bme.pressure;
  sAQI = iaqSensor.staticIaq;

  Serial.print("Temperature = ");
  Serial.print(temp);
  Serial.println(" *C");
  oled.print("T ");
  oled.print(temp);
  oled.println(" *C");

  Serial.print("Pressure = ");
  Serial.print(bme.pressure / 100.0);
  Serial.println(" hPa");

  Serial.print("Humidity = ");
  Serial.print(hume);
  Serial.println(" %");
  oled.print("H ");
  oled.print(hume);
  oled.println(" %");

  Serial.print("sAQI = ");
  Serial.println(sAQI);
  oled.print("sAQI ");
  oled.println(sAQI);
  /*
  Serial.print("Gas = ");
  Serial.print(bme.gas_resistance / 1000.0);
  Serial.println(" KOhms");
  oled.print("G ");
  oled.print(bme.gas_resistance / 1000.0);
  oled.println(" KOhms");

  Serial.print("Approx. Altitude = ");
  Serial.print(bme.readAltitude(SEALEVELPRESSURE_HPA));
  Serial.println(" m");
  */

  // update the display with the new count
  oled.display();

  // checking values for alarms

  int rojoRGB[3] = {255, 0, 0};
  int celesteRGB[3] = {0, 181, 255};
  
  checksAlarm(temp, TEMP_MAX, TEMP_MIN, "Temperatura", rojoRGB, celesteRGB);
  checksAlarm(hume, HUME_MAX, HUME_MIN, "Humedad", rojoRGB, celesteRGB);
  checksAQI(sAQI, SAQI_MAX, SAQI_MIN);

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

void setLedsRGB(int colors[3], int last_led, int delay1) {

  for (int i = 0; i < last_led; i++) {
    neopixelLEDs.setPixelColor(i, neopixelLEDs.Color(colors[0], colors[1], colors[2]));
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

void checksAQI(int value, int max, int min) {
  bool check = false;
  
  int rojoRGB[3] = {255, 0, 0};
  int celesteRGB[3] = {0, 181, 255};
  
  if (max != -999) {
    if (value > max) {

      check = true;
      oled.clearDisplay();
      oled.setCursor(0, 0);
      oled.println("Niveles de sAQI altos");
      oled.print("Valor: ");
      oled.println(value);

      oled.display();

      setLedsAndBuzzer(rojoRGB, PIXEL_COUNT, 1000, 6, 100);
    }
  }
  if (min != -999) {

      check = true;
    if (value < min) {
      oled.clearDisplay();
      oled.setCursor(0, 0);
      oled.println("Niveles de sAQI bajos");
      oled.print("Valor: ");
      oled.println(value);

      oled.display();

      setLedsAndBuzzer(celesteRGB, PIXEL_COUNT, 500, 3, 200);
    }
  }
  if (!check){
    oled.clearDisplay();
    oled.setCursor(0, 0);
    oled.println("Niveles de sAQI:");
    oled.println(value);
    if (value > 300){
      int colors [3] = {126,0,35};
      setLedsRGB(colors, PIXEL_COUNT, 1000);
    }else if (value > 200){
      int colors [3] = {143,63,151};
      setLedsRGB(colors, PIXEL_COUNT, 1000);
    }else if (value > 150){
      int colors [3] = {255,0,0};
      setLedsRGB(colors, PIXEL_COUNT, 1000);
    }else if (value > 100){
      int colors [3] = {255,126,0};
      setLedsRGB(colors, PIXEL_COUNT, 1000);
    }else if (value > 50){
      int colors [3] = {255,255,0};
      setLedsRGB(colors, PIXEL_COUNT, 1000);
    }else if (value > 0){
      int colors [3] = {0,228,0};
      setLedsRGB(colors, PIXEL_COUNT, 1000);
    }
  }
}

void checksAlarm(int value, int max, int min, String nombre, int colorsHigh [3], int colorsLow [3]) {
  bool check = false;
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


// Funciones de ayuda en el sensor BME680
void checkIaqSensorStatus(void){
  if (iaqSensor.status != BSEC_OK) {
    if (iaqSensor.status < BSEC_OK) {
      output = "BSEC error code : " + String(iaqSensor.status);
      Serial.println(output);
    } else {
      output = "BSEC warning code : " + String(iaqSensor.status);
      Serial.println(output);
    }
  }

  if (iaqSensor.bme680Status != BME680_OK) {
    if (iaqSensor.bme680Status < BME680_OK) {
      output = "BME680 error code : " + String(iaqSensor.bme680Status);
      Serial.println(output);
    } else {
      output = "BME680 warning code : " + String(iaqSensor.bme680Status);
      Serial.println(output);
    }
  }
}
