#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_ADXL345_U.h>
#include <SPI.h>
//#include <ADXL345.h>

uint8_t spi_clock = 15; // aka SCL
uint8_t spi_miso = 14; // aka SDO
uint8_t spi_mosi = 16; // aka SDA
uint8_t spi_cs = 10;
uint8_t button_PIN = 7;

uint8_t interrupt1 = 4; //pin 7 is interrupt 4, pin 2 is interrupt 1, pin 0 is interrupt 2, and pin 1 is interrupt 3
uint8_t interrupt2 = 1; //pin 3 maps to interrupt 0

const uint8_t VARIABLE_QUEUE_DECELERATION_SIZE = 32; //must be less than 32 to use FIFO
int16_t temp_FIFO_int[VARIABLE_QUEUE_DECELERATION_SIZE];

//Adafruit_ADXL345_Unified accel = Adafruit_ADXL345_Unified(spi_cs, 12345);
Adafruit_ADXL345_Unified accel = Adafruit_ADXL345_Unified(spi_clock, spi_miso, spi_mosi, spi_cs, 12345);
sensors_event_t event;
volatile uint8_t state = 0;
unsigned long sample_time; //initial sample time
unsigned long decision_time; //initial decision time //does this need to be started now?  What about sample time?
float min_val = 0; //used to calculate data range
float max_val = 0; //used to calculate data range
float mean = 0;
float range;

void setup()
{
  Serial.begin(9600);
  //while (!Serial);

  sensor_t sensor;
  accel.getSensor(&sensor);
  delay(1000);
  
  if(!accel.begin())
  {
    Serial.println("Problem detecting the ADXL345 - check connections");
    while(1); //All this does is fails forever, maybe get it to retry?
  }

  accel.setRange(ADXL345_RANGE_4_G);
  accel.setDataRate(ADXL345_DATARATE_25_HZ);
  accel.setFIFOMode(ADXL345_FIFO_MODE_FIFO);
  accel.setFIFOSamples(VARIABLE_QUEUE_DECELERATION_SIZE-1);
  accel.writeRegister(ADXL345_REG_INT_ENABLE, 0);
  accel.writeRegister(ADXL345_REG_INT_MAP, 0);
  accel.writeBits(ADXL345_REG_INT_MAP, 1, ADXL345_INT_WATERMARK_BIT, 1);

  Serial.println("Begin");
  Serial.print("FIFO samples: ");
  Serial.println(accel.readRegister(ADXL345_REG_INT_MAP));

  pinMode(2, INPUT);
  attachInterrupt(interrupt2, Read_FIFO, RISING); // Interrupt on Pin 7
}

void loop()
{  
  Serial.print("FIFO samples: ");
  Serial.println(accel.readRegister(ADXL345_REG_FIFO_STATUS));
  delay(1000);
}

void Read_FIFO()
{
  detachInterrupt(interrupt2);

  Serial.println("FIFO filled");

  for(uint8_t i = 0; i<32; i++)
  {
    temp_FIFO_int[i] = accel.getX();
  }
  delay(1000);

  //Serial.print("FIFO samples: ");
  //Serial.println(accel.readRegister(ADXL345_REG_FIFO_STATUS));

  CALCULATE_QUEUE_STATISTICS();
  attachInterrupt(interrupt2, Read_FIFO, RISING); // Interrupt on Pin 7
}

void CALCULATE_QUEUE_STATISTICS()
{
  Serial.println("CALCULATE_QUEUE_STATISTICS ROUTINE");
  float temp_val = temp_FIFO_int[0] * ADXL345_MG2G_MULTIPLIER * SENSORS_GRAVITY_STANDARD;

  //Calculate range and mean of sampled data
  min_val = temp_val;
  max_val = temp_val;
  mean = temp_val;

  for(uint8_t i=1; i<VARIABLE_QUEUE_DECELERATION_SIZE; i++) //was: for(uint8_t i=1; i<FIFO_STATUS; i++)
  {
    temp_val = temp_FIFO_int[i] * ADXL345_MG2G_MULTIPLIER * SENSORS_GRAVITY_STANDARD;
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
  Serial.print("Queue mean: ");
  Serial.println(mean);
  range = max_val - min_val;
  Serial.print("Queue range: ");
  Serial.println(range);
}

