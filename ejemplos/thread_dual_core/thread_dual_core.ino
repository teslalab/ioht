/*********
  Rui Santos
  Complete project details at https://randomnerdtutorials.com  
*********/

#include <Wire.h>
#include <Adafruit_NeoPixel.h>
#include <Adafruit_FeatherOLED.h>

TaskHandle_t Task1;

Adafruit_FeatherOLED oled = Adafruit_FeatherOLED();

#define PIXEL_PIN 14  // Digital IO pin connected to the NeoPixels.
#define PIXEL_COUNT 6 // Number of NeoPixels

Adafruit_NeoPixel neopixelLEDs(PIXEL_COUNT, PIXEL_PIN, NEO_GRB + NEO_KHZ800);

void setup()
{

    Serial.begin(9600); //Initialize serial port

    oled.init();

    //create a task that will be executed in the Task1code() function, with priority 1 and executed on core 0
    xTaskCreatePinnedToCore(
        Task1code, /* Task function. */
        "Task1",   /* name of task. */
        10000,     /* Stack size of task */
        NULL,      /* parameter of the task */
        1,         /* priority of the task */
        &Task1,    /* Task handle to keep track of created task */
        1);        /* pin task to core 0 */
    delay(500);
}

void Task1code( void * pvParameters ){
    while (true)
    {
        for (int i = 0; i < PIXEL_COUNT; i++)
        {
            neopixelLEDs.setPixelColor(i, neopixelLEDs.Color(0, 0, 50));
        }
        neopixelLEDs.show();
        delay(500);
        neopixelLEDs.clear();
        neopixelLEDs.show();
        delay(500);
    }
}

void loop()
{
    int tiempo = 0;
    while (true)
    {
        
        oled.clearDisplay();
        oled.setCursor(0, 0);
        oled.println("usando core: ");
        oled.println(xPortGetCoreID());
        oled.println(tiempo);
        oled.display();
        delay(1000);
        tiempo += 1;
    }
    
}