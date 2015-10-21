#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_ADXL345_U.h>
#include <SPI.h>
//#include <ADXL345.h>
#include <Adafruit_NeoPixel.h>

uint8_t spi_clock = 15; // aka SCL
uint8_t spi_miso = 14; // aka SDO
uint8_t spi_mosi = 16; // aka SDA
uint8_t spi_cs = 10;
uint8_t button_PIN = 7;

uint8_t neopixel_PIN = 9;
uint8_t NUMPIXELS = 1;

Adafruit_NeoPixel pixels = Adafruit_NeoPixel(NUMPIXELS, neopixel_PIN, NEO_GRB + NEO_KHZ800);
Adafruit_ADXL345_Unified accel = Adafruit_ADXL345_Unified(spi_clock, spi_miso, spi_mosi, spi_cs, 12345);
sensors_event_t event;

uint8_t interrupt1 = 4; //pin 7 is interrupt 4, pin 2 is interrupt 1, pin 0 is interrupt 2, and pin 1 is interrupt 3
uint8_t interrupt2 = 1; //pin 3 maps to interrupt 0

void setup()
{
//  pixels.begin();
//
//  pixels.setPixelColor(0, pixels.Color(150,150,0)); // Moderately bright green color.
//  pixels.show();

  delay(1000);

  Serial.begin(9600);
  while (!Serial);

  if(!accel.begin())
  {
    /* There was a problem detecting the ADXL345 ... check your connections */
    Serial.println("Ooops, no ADXL345 detected ... Check your wiring!");
    //while(1);
  }

  accel.setRange(ADXL345_RANGE_4_G);
}

void loop()
{  
  /* Get a new sensor event */ 
  sensors_event_t event; 
  accel.getEvent(&event);
 
  /* Display the results (acceleration is measured in m/s^2) */
  Serial.print("X: "); Serial.print(event.acceleration.x); Serial.print("  ");
  Serial.print("Y: "); Serial.print(event.acceleration.y); Serial.print("  ");
  Serial.print("Z: "); Serial.print(event.acceleration.z); Serial.print("  ");Serial.println("m/s^2 ");
  delay(500);
}

