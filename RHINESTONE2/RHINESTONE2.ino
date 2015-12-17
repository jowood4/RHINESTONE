#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_ADXL345_U.h>
#include <SPI.h>
//#include <ADXL345.h>
#include <Adafruit_NeoPixel.h>

uint8_t neopixel_PIN = 9;
uint8_t NUMPIXELS = 1;
uint8_t spi_clock = 15; // aka SCL
uint8_t spi_miso = 14; // aka SDO
uint8_t spi_mosi = 16; // aka SDA
uint8_t spi_cs = 10;
uint8_t button_PIN = 7;

uint8_t interrupt1 = 4; //pin 7 is interrupt 4, pin 2 is interrupt 1, pin 0 is interrupt 2, and pin 1 is interrupt 3
uint8_t interrupt2 = 1; //pin 3 maps to interrupt 0

float CONSTANT_UPWARD_DT = 0.65;
float CONSTANT_DOWNWARD_DT = 0.65; //0.3
long CONSTANT_DEBOUNCETIME = 3000; // in milliseconds
float CONSTANT_STRAIGHTANDLEVELRANGE = 0.2; //was 0.05
float CONSTANT_GRAVITY = 9.81;  //flat, not moving = 1.59, straight down = 1.29
//float CONSTANT_ZERO_READING = 1.59;
const float pi = 3.14159;
const uint8_t VARIABLE_QUEUE_DECELERATION_SIZE = 32; //must be less than 32 to use FIFO

int16_t temp_FIFO_int[VARIABLE_QUEUE_DECELERATION_SIZE];
uint8_t FIFO_STATUS;
uint8_t VARIABLE_UPDOWN;
float VARIABLE_COMPENSATED_DECELERATION;
String VARIABLE_CURRENT_MODE;
float VARIABLE_QUEUE_DECELERATION[VARIABLE_QUEUE_DECELERATION_SIZE];
float VARIABLE_QUEUE_RANGE;
float VARIABLE_COMPUTED_PITCHANGLE; // in radians.  Arduino trig functions default to radians.
//float VARIABLE_SAMPLE_RATE; // in Hz
float VARIABLE_STRAIGHTANDLEVEL_DECISION_RATE; // in Hz

Adafruit_NeoPixel pixels = Adafruit_NeoPixel(NUMPIXELS, neopixel_PIN, NEO_GRB + NEO_KHZ800);
/* Assign a unique ID to this sensor at the same time */
Adafruit_ADXL345_Unified accel = Adafruit_ADXL345_Unified(spi_clock, spi_miso, spi_mosi, spi_cs, 12345);
sensors_event_t event;
volatile uint8_t state = 0;
unsigned long sample_time; //initial sample time
unsigned long decision_time; //initial decision time //does this need to be started now?  What about sample time?
float min_val = 0; //used to calculate data range
float max_val = 0; //used to calculate data range
float mean = 0;
float range;

uint8_t openlog_connected;
uint8_t print_serial = 1;

void setup()
{

  if(print_serial)
  {
    Serial.begin(9600);
    while (!Serial);
  }

  //Start datalogger
  Serial1.begin(9600);
  if(gotoCommandMode())
  {
    openlog_connected = 1;
    Serial1.write("append newfile.csv");
    Serial1.write(13);
    Serial.println("OpenLog online");
    //Serial.write(13);

    //Serial1.print("Test write stuff");
    //Serial1.write(13);
  }
  else
  {
    Serial.print("OpenLog offline");
    Serial.write(13);
  }

  //Start accelerometer
  sensor_t sensor;
  accel.getSensor(&sensor);
  delay(1000);
  
  if(!accel.begin())
  {
    if(print_serial)
    {
      Serial.println("Problem detecting the ADXL345 - check connections");
    }
  }
  
  accel.setRange(ADXL345_RANGE_4_G);
  accel.setDataRate(ADXL345_DATARATE_25_HZ);
  accel.setFIFOMode(ADXL345_FIFO_MODE_FIFO);
  accel.setFIFOSamples(VARIABLE_QUEUE_DECELERATION_SIZE-1);
  accel.writeRegister(ADXL345_REG_INT_ENABLE, 0);
  accel.writeRegister(ADXL345_REG_INT_MAP, 0);
  accel.writeBits(ADXL345_REG_INT_ENABLE, 1, ADXL345_INT_WATERMARK_BIT, 1);

  //Start neopixel
  pixels.begin();
  pixels.setPixelColor(0, pixels.Color(150,150,0)); // Moderately bright green color.
  pixels.show();

  pinMode(button_PIN, INPUT);
  attachInterrupt(interrupt1, pushbutton_ISR, FALLING); // Interrupt on Pin 7

  BOOT();
  TRANSITION_TO_SLEEP();
}

void loop()
{  
}

void pushbutton_ISR()
{
  detachInterrupt(interrupt1);
  delay(CONSTANT_DEBOUNCETIME);
  if(digitalRead(button_PIN) == 0) //(SENSOR_UCS) == 0)
  {
      if(print_serial)
      {
        Serial.println("UCS has been pressed for the debounce period");
      }
      if(VARIABLE_CURRENT_MODE == "mode_normal")
      {
        TRANSITION_TO_SLEEP();
      }
      else if(VARIABLE_CURRENT_MODE == "mode_sleep")
      {
        TRANSITION_TO_NORMAL();
      }
   }
   attachInterrupt(interrupt1, pushbutton_ISR, FALLING); // Interrupt on Pin 7
}

void BOOT()
{
  if(print_serial)
  {
    Serial.println("BOOT ROUTINE");
  }
  VARIABLE_UPDOWN = 0;
  VARIABLE_COMPENSATED_DECELERATION = 0;
  VARIABLE_CURRENT_MODE = "boot";
  VARIABLE_QUEUE_RANGE = 0;
  VARIABLE_COMPUTED_PITCHANGLE = 0;
  //VARIABLE_SAMPLE_RATE = 90; //was 60
  VARIABLE_STRAIGHTANDLEVEL_DECISION_RATE = 30; //was 1
  FIFO_STATUS = 0;
}

void TRANSITION_TO_SLEEP()
{
  if(print_serial)
  {
    Serial.println("TRANSITION_TO_SLEEP ROUTINE");
  }
  detachInterrupt(interrupt2);
  accel.writeBits(ADXL345_REG_POWER_CTL, 0, ADXL345_PCTL_MEASURE_BIT, 1); 
  VARIABLE_CURRENT_MODE = "transition_to_sleep";  
  SLEEP_SIGNAL();
  MODE_SLEEP();
}

void TRANSITION_TO_NORMAL()
{
  if(print_serial)
  {
    Serial.println("TRANSITION_TO_NORMAL ROUTINE");
  }
  attachInterrupt(interrupt2, Read_FIFO_ISR, RISING); // Interrupt on Pin 7
  accel.writeBits(ADXL345_REG_POWER_CTL, 1, ADXL345_PCTL_MEASURE_BIT, 1);
  VARIABLE_CURRENT_MODE = "transition_to_normal"; 
  NORMAL_SIGNAL();
  MODE_NORMAL();
}

void MODE_NORMAL()
{
  if(print_serial)
  {
    Serial.println("MODE_NORMAL ROUTINE");
  }
  VARIABLE_CURRENT_MODE = "mode_normal";  
}

void Read_FIFO_ISR()
{
  detachInterrupt(interrupt2);

  if(print_serial)
  {
    Serial.println("FIFO filled");
  }
  
  for(uint8_t i = 0; i<32; i++)
  {
    temp_FIFO_int[i] = accel.getX();
  }

  CALCULATE_QUEUE_STATISTICS();
  EVALUATE_DECELERATION();
  attachInterrupt(interrupt2, Read_FIFO_ISR, RISING); // Interrupt on Pin 7
}

void MODE_SLEEP()
{
  if(print_serial)
  {
    Serial.println("MODE_SLEEP ROUTINE");
  }
  VARIABLE_CURRENT_MODE = "mode_sleep";
}

void SLEEP_SIGNAL()
{
  if(print_serial)
  {
    Serial.println("SLEEP_SIGNAL ROUTINE");
  }
  unsigned long time = millis();

  //digitalWrite(ACTUATOR_SL,LOW);
  pixels.setPixelColor(0, pixels.Color(200,0,0)); // Moderately bright green color.
  pixels.show();
  while(millis() - time < 200);
  //digitalWrite(ACTUATOR_SL,HIGH);
  pixels.setPixelColor(0, pixels.Color(0,0,0)); // Moderately bright green color.
  pixels.show();
  delay(50);
  
  //digitalWrite(ACTUATOR_SL,LOW);
  pixels.setPixelColor(0, pixels.Color(200,0,0)); // Moderately bright green color.
  pixels.show();
  while(millis() - time < 100);
  //digitalWrite(ACTUATOR_SL,HIGH);
  pixels.setPixelColor(0, pixels.Color(0,0,0)); // Moderately bright green color.
  pixels.show();
  delay(50);
  
  //digitalWrite(ACTUATOR_SL,LOW);
  pixels.setPixelColor(0, pixels.Color(200,0,0)); // Moderately bright green color.
  pixels.show();
  while(millis() - time < 50);
  //digitalWrite(ACTUATOR_SL,HIGH);
  pixels.setPixelColor(0, pixels.Color(0,0,0)); // Moderately bright green color.
  pixels.show();
  while(millis() - time < 1000);

}

void NORMAL_SIGNAL()
{
  if(print_serial)
  {
    Serial.println("NORMAL_SIGNAL ROUTINE");
  }
  unsigned long time = millis();

  //digitalWrite(ACTUATOR_SL,LOW);
  pixels.setPixelColor(0, pixels.Color(0,0,0)); // Moderately bright green color.
  pixels.show();
  while(millis() - time < 200);
  //digitalWrite(ACTUATOR_SL,HIGH);
  pixels.setPixelColor(0, pixels.Color(200,0,0)); // Moderately bright green color.
  pixels.show();
  delay(50);
  
  //digitalWrite(ACTUATOR_SL,LOW);
  pixels.setPixelColor(0, pixels.Color(0,0,0)); // Moderately bright green color.
  pixels.show();
  while(millis() - time < 100);
  //digitalWrite(ACTUATOR_SL,HIGH);
  pixels.setPixelColor(0, pixels.Color(200,0,0)); // Moderately bright green color.
  pixels.show();
  delay(50);
  
  //digitalWrite(ACTUATOR_SL,LOW);
  pixels.setPixelColor(0, pixels.Color(0,0,0)); // Moderately bright green color.
  pixels.show();
  while(millis() - time < 50);
  //digitalWrite(ACTUATOR_SL,HIGH);
  pixels.setPixelColor(0, pixels.Color(200,0,0)); // Moderately bright green color.
  pixels.show();
  while(millis() - time < 1000);
  //digitalWrite(ACTUATOR_SL,LOW);
  pixels.setPixelColor(0, pixels.Color(0,0,0)); // Moderately bright green color.
  pixels.show();
}

void BRAKING_SIGNAL_ON()
{
  if(print_serial)
  {
    Serial.println("BRAKING_SIGNAL_ON ROUTINE");
  }
  //digitalWrite(ACTUATOR_SL,HIGH);
  pixels.setPixelColor(0, pixels.Color(200,0,0)); // Moderately bright green color.
  pixels.show();
}

void BRAKING_SIGNAL_OFF()
{
  if(print_serial)
  {
    Serial.println("BRAKING_SIGNAL_OFF ROUTINE");
  }
  //digitalWrite(ACTUATOR_SL,LOW);
  pixels.setPixelColor(0, pixels.Color(0,0,0)); // Moderately bright green color.
  pixels.show();
}

void TRANSITION_ACTIVATE_ACTUATOR_SL()
{
  if(print_serial)
  {
    Serial.println("TRANSITION_ACTIVATE_ACTUATOR_SL ROUTINE");
  }
  //VARIABLE_CURRENT_MODE = "transition_activate_actuator_sl";
  BRAKING_SIGNAL_ON();
  VARIABLE_UPDOWN = 1;
}

void TRANSITION_DEACTIVATE_ACTUATOR_SL()
{
  if(print_serial)
  {
    Serial.println("TRANSITION_DEACTIVATE_ACTUATOR_SL ROUTINE");
  }
  //VARIABLE_CURRENT_MODE = "transition_deactivate_actuator_sl";
  BRAKING_SIGNAL_OFF();
  VARIABLE_UPDOWN = 0;
}

void CALCULATE_QUEUE_STATISTICS()
{
  if(print_serial)
  {
    Serial.println("CALCULATE_QUEUE_STATISTICS ROUTINE");
  }
  float temp_val = temp_FIFO_int[0] * ADXL345_MG2G_MULTIPLIER * SENSORS_GRAVITY_STANDARD;

  //Calculate range and mean of sampled data
  min_val = temp_val;
  max_val = temp_val;
  mean = temp_val;

  if(print_serial)
  {
    Serial.print("Q- "); //Q- i1: x.xx | i2: x.xy | i..n: x.xz
  }
  if(openlog_connected)
  {
    Serial1.print("Q- ");
  }
    
  for(uint8_t i=1; i<VARIABLE_QUEUE_DECELERATION_SIZE; i++) //was: for(uint8_t i=1; i<FIFO_STATUS; i++)
  {
    temp_val = temp_FIFO_int[i] * ADXL345_MG2G_MULTIPLIER * SENSORS_GRAVITY_STANDARD;

    if(print_serial)
    {
      Serial.print("i");
      Serial.print(i);
      Serial.print(": ");
      Serial.print(temp_val);
      Serial.print(" | ");
    }

    if(openlog_connected)
    {
      Serial1.print("i");
      Serial1.print(i);
      Serial1.print(": ");
      Serial1.print(temp_val);
      Serial1.print(" | ");
    }
    
    if(temp_val < min_val)
    {
      min_val = temp_val;
    }
    if(temp_val > max_val)
    {
      max_val = temp_val;
    }
    mean = mean + temp_val;
  }

  mean = mean/VARIABLE_QUEUE_DECELERATION_SIZE; //was: mean = mean/FIFO_STATUS;
  range = max_val - min_val;

  if(print_serial)
  {
    Serial.println(" ");
    Serial.print("Queue mean: ");
    Serial.println(mean);
    Serial.print("Queue range: ");
    Serial.println(range);
  }
  if(openlog_connected)
  {
    Serial1.println(" ");
    Serial1.print("Queue mean: ");
    Serial1.println(mean);
    Serial1.print("Queue range: ");
    Serial1.println(range);
  }
}

void EVALUATE_DECELERATION()
{
  if(print_serial)
  {
    Serial.println("EVALUATE_DECELERATION ROUTINE");
    Serial.println(VARIABLE_UPDOWN);
  }
  
  if(VARIABLE_UPDOWN == 1) //Light is ON
  {
    VARIABLE_COMPENSATED_DECELERATION = abs(mean - (CONSTANT_GRAVITY*sin(VARIABLE_COMPUTED_PITCHANGLE)))*cos(VARIABLE_COMPUTED_PITCHANGLE);
    if(print_serial)
    {
      Serial.print("Compensated deceleration: ");
      Serial.println(VARIABLE_COMPENSATED_DECELERATION);
    }
    
    if(VARIABLE_COMPENSATED_DECELERATION < CONSTANT_DOWNWARD_DT)
    {
      if(print_serial)
      {
        Serial.println("Downward");
      }
      TRANSITION_DEACTIVATE_ACTUATOR_SL();
    }
  }
  else if(VARIABLE_UPDOWN == 0) //Light is OFF
  {
    //Determine if event occurred based on mean and range
    if(range <= CONSTANT_STRAIGHTANDLEVELRANGE) //20150715 This if could largely be superseded by an if in an earlier routine, maybe mode normal or calculate statistics, and only the relevant pitch angle/calculate decel code called.
    {
      VARIABLE_COMPUTED_PITCHANGLE = asin(mean/CONSTANT_GRAVITY);//*(180/pi);
      if(print_serial)
      {
        Serial.println("Queue range indicates insignificant deceleration, updating pitch angle");
        Serial.print("Pitch Angle: ");
        Serial.println(VARIABLE_COMPUTED_PITCHANGLE);
      //20150714 IS THERE ANY REASON THIS ALWAYS SEEMS TO EVALUATE TO 0.03 ON LAST TEST?
      }
    }
    else
    {
      //20150714 ONLY IF THE DECELERATION IS "SIGNIFICANT" (i.e. range is wide enough) ARE THE DECISIONS MADE TO TURN LIGHT ON OR OFF.  DOESN'T ACCOUNT FOR THE LIKELY POSSIBILITY THAT LIGHT GETS TURNED ON THEN THE RANGE DROPS AND IT NEVER GETS TURNED OFF...
      VARIABLE_COMPENSATED_DECELERATION = abs(mean - (CONSTANT_GRAVITY*sin(VARIABLE_COMPUTED_PITCHANGLE)))*cos(VARIABLE_COMPUTED_PITCHANGLE);
      if(print_serial)
      {
        Serial.println("Queue range indicates significant deceleration, calculating compensated deceleration");
        Serial.print("Compensated deceleration: ");
        Serial.println(VARIABLE_COMPENSATED_DECELERATION);
      }
  
      if(VARIABLE_COMPENSATED_DECELERATION >= CONSTANT_UPWARD_DT)
      {
        if(print_serial)
        {
          Serial.println("Upward");
        }
        TRANSITION_ACTIVATE_ACTUATOR_SL();
      }
    }
  }

//  //Determine if event occurred based on mean and range
//  if(range <= CONSTANT_STRAIGHTANDLEVELRANGE) //20150715 This if could largely be superseded by an if in an earlier routine, maybe mode normal or calculate statistics, and only the relevant pitch angle/calculate decel code called.
//  {
//    Serial.println("Queue range indicates insignificant deceleration, updating pitch angle");
//    VARIABLE_COMPUTED_PITCHANGLE = asin(mean/CONSTANT_GRAVITY);//*(180/pi);
//    Serial.print("Pitch Angle: ");
//    Serial.println(VARIABLE_COMPUTED_PITCHANGLE);
//    //20150714 IS THERE ANY REASON THIS ALWAYS SEEMS TO EVALUATE TO 0.03 ON LAST TEST?
//  }
//  else
//  {
//    //20150714 ONLY IF THE DECELERATION IS "SIGNIFICANT" (i.e. range is wide enough) ARE THE DECISIONS MADE TO TURN LIGHT ON OR OFF.  DOESN'T ACCOUNT FOR THE LIKELY POSSIBILITY THAT LIGHT GETS TURNED ON THEN THE RANGE DROPS AND IT NEVER GETS TURNED OFF...
//    Serial.println("Queue range indicates significant deceleration, calculating compensated deceleration");
//    VARIABLE_COMPENSATED_DECELERATION = abs(mean - (CONSTANT_GRAVITY*sin(VARIABLE_COMPUTED_PITCHANGLE)))*cos(VARIABLE_COMPUTED_PITCHANGLE);
//    Serial.print("Compensated deceleration: ");
//    Serial.println(VARIABLE_COMPENSATED_DECELERATION);
//
//    if((VARIABLE_UPDOWN == 0)&&(VARIABLE_COMPENSATED_DECELERATION >= CONSTANT_UPWARD_DT))
//    {
//      Serial.println("Upward");
//      TRANSITION_ACTIVATE_ACTUATOR_SL();
//    }
//    else if((VARIABLE_UPDOWN == 1)&&(VARIABLE_COMPENSATED_DECELERATION < CONSTANT_DOWNWARD_DT)) //20150714 Any reason this isn't encapsulated in curly braces in the same way as the previous else? 
//    {
//      Serial.println("Downward");
//      TRANSITION_DEACTIVATE_ACTUATOR_SL();
//    }
//  }

  while((millis() - decision_time) < (1000/VARIABLE_STRAIGHTANDLEVEL_DECISION_RATE));
}


//This function pushes OpenLog into command mode
bool gotoCommandMode(void) {

  bool result = false;
  int reading;

  if(print_serial)
  {
    Serial.println("Going into command mode");
    //Serial.write(13);
  }
  
  //Send three control z to enter OpenLog command mode
  //Works with Arduino v1.0
  Serial1.write(26);
  Serial1.write(26);
  Serial1.write(26);

  for(int timeOut = 0 ; timeOut < 10000 ; timeOut++)
    {
      while(Serial1.available())
      {
        reading = Serial1.read();
        if(reading == '>')
        {
          result = true;
          break;
        }
      }
    }

    return result;
}

