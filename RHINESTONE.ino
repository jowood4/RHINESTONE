#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_ADXL345_U.h>

uint8_t clock = 11;
uint8_t miso = 12;
uint8_t mosi = 13;
uint8_t cs = 10;

/* Assign a unique ID to this sensor at the same time */
Adafruit_ADXL345_Unified accel = Adafruit_ADXL345_Unified(clock, miso, mosi, cs, 12345);

uint8_t SENSOR_UCS = 4; //Digital Pin for Pushbutton Switch
//uint8_t SENSOR_DS = 2; //Analog Pin for Reading Accelerometer
uint8_t ACTUATOR_SL = 3; //Digital Pin for LED

float CONSTANT_UPWARD_DT = 0.05;
float CONSTANT_DOWNWARD_DT = 0.05;
long CONSTANT_DEBOUNCETIME = 3000; // in milliseconds
float CONSTANT_STRAIGHTANDLEVELRANGE = 0.05;
float CONSTANT_GRAVITY = 0.3;  //flat, not moving = 1.59, straight down = 1.29
float CONSTANT_ZERO_READING = 1.59;
const float pi = 3.14159;
const uint8_t VARIABLE_QUEUE_DECELERATION_SIZE = 5;

uint8_t VARIABLE_UPDOWN;
float VARIABLE_COMPENSATED_DECELERATION;
String VARIABLE_CURRENT_MODE;
float VARIABLE_QUEUE_DECELERATION[VARIABLE_QUEUE_DECELERATION_SIZE];
float VARIABLE_QUEUE_RANGE;
float VARIABLE_COMPUTED_PITCHANGLE; // in radians.  Arduino trig functions default to radians.
float VARIABLE_SAMPLE_RATE; // in Hz
float VARIABLE_STRAIGHTANDLEVEL_DECISION_RATE; // in Hz


void setup()
{
	if(!accel.begin())
	{
	/* There was a problem detecting the ADXL345 ... check your connections */
	Serial.println("Ooops, no ADXL345 detected ... Check your wiring!");
	while(1);
	}

	/* Set the range to whatever is appropriate for your project */
	accel.setRange(ADXL345_RANGE_16_G);
	// displaySetRange(ADXL345_RANGE_8_G);
	// displaySetRange(ADXL345_RANGE_4_G);
	// displaySetRange(ADXL345_RANGE_2_G);

	pinMode(ACTUATOR_SL, OUTPUT);
	pinMode(SENSOR_UCS, INPUT);
        //Serial.begin(9600);
	BOOT();
	TRANSITION_TO_SLEEP();
}

void loop()
{
	//Debouncing done in this function
	if(digitalRead(SENSOR_UCS) == 0)
	{
		delay(CONSTANT_DEBOUNCETIME);
		if(digitalRead(SENSOR_UCS) == 0)
		{
			if(VARIABLE_CURRENT_MODE == "mode_normal")
			{
				TRANSITION_TO_SLEEP();
			}
			else if(VARIABLE_CURRENT_MODE == "mode_sleep")
			{
				TRANSITION_TO_NORMAL();
			}
		}
	}
	else if(VARIABLE_CURRENT_MODE == "mode_normal") //Go back to mode_normal if button not pressed
	{
		MODE_NORMAL();
	}
}

void BOOT()
{
	VARIABLE_UPDOWN = 0;
	VARIABLE_COMPENSATED_DECELERATION = 0;
	VARIABLE_CURRENT_MODE = "boot";
	VARIABLE_QUEUE_RANGE = 0;
	VARIABLE_COMPUTED_PITCHANGLE = 0;
	VARIABLE_SAMPLE_RATE = 50;
	VARIABLE_STRAIGHTANDLEVEL_DECISION_RATE = 0.5;

	for(uint8_t i = 0; i<VARIABLE_QUEUE_DECELERATION_SIZE; i++)
	{
		VARIABLE_QUEUE_DECELERATION[i] = 0;
	}
}

void TRANSITION_TO_SLEEP()
{
	VARIABLE_CURRENT_MODE = "transition_to_sleep";
	SLEEP_SIGNAL();
	MODE_SLEEP();
}

void MODE_SLEEP()
{
	VARIABLE_CURRENT_MODE = "mode_sleep";
}

void TRANSITION_TO_NORMAL()
{
	VARIABLE_CURRENT_MODE = "transition_to_normal";
	NORMAL_SIGNAL();
	MODE_NORMAL();
}

void MODE_NORMAL()
{
        //float sample;
        unsigned long sample_time; //initial sample time
        unsigned long decision_time = millis(); //initial decision time
        float min_val = 0; //used to calculate data range
        float max_val = 0; //used to calculate data range
        float mean = 0;
        float range;
        VARIABLE_CURRENT_MODE = "mode_normal";
        //Serial.println("Entered Normal Mode");
        sensors_event_t event;
        accel.getEvent(&event);

        //Sampling loop
        for(uint8_t i=0; i<VARIABLE_QUEUE_DECELERATION_SIZE; i++)
        {
            sample_time  = millis();
            //sample = analogRead(SENSOR_DS); //this takes roughly 100us
            VARIABLE_QUEUE_DECELERATION[i] = event.acceleration.y;
            //VARIABLE_QUEUE_DECELERATION[i] = ((float(sample)/float(1024))*5) - CONSTANT_ZERO_READING;
            while((millis() - sample_time) < (1000/VARIABLE_SAMPLE_RATE));
        }
        
        //Serial.println("Done Sampling");
        
        //Calculate range and mean of sampled data
        min_val = VARIABLE_QUEUE_DECELERATION[0];
        max_val = VARIABLE_QUEUE_DECELERATION[0];
        mean = VARIABLE_QUEUE_DECELERATION[0];
        for(uint8_t i=1; i<VARIABLE_QUEUE_DECELERATION_SIZE; i++)
        {
            if(VARIABLE_QUEUE_DECELERATION[i] < min_val)
            {
                min_val = VARIABLE_QUEUE_DECELERATION[i];
            }
            if(VARIABLE_QUEUE_DECELERATION[i] > max_val)
            {
                max_val = VARIABLE_QUEUE_DECELERATION[i];
            }
            mean = mean + VARIABLE_QUEUE_DECELERATION[i];
        }
        mean = mean/VARIABLE_QUEUE_DECELERATION_SIZE;
        range = max_val - min_val;
        
        //Determine if event occured based on mean and range
        if(range <= CONSTANT_STRAIGHTANDLEVELRANGE)
        {
            VARIABLE_COMPUTED_PITCHANGLE = asin(mean/CONSTANT_GRAVITY);//*(180/pi);
            Serial.println(VARIABLE_COMPUTED_PITCHANGLE);
        }
        else
        {
            VARIABLE_COMPENSATED_DECELERATION = abs(mean - (CONSTANT_GRAVITY*sin(VARIABLE_COMPUTED_PITCHANGLE)))*cos(VARIABLE_COMPUTED_PITCHANGLE);
            //Serial.println(VARIABLE_COMPENSATED_DECELERATION);
            //Serial.println(VARIABLE_COMPUTED_PITCHANGLE);
            //Serial.println(mean);
            if((VARIABLE_UPDOWN == 0)&&(VARIABLE_COMPENSATED_DECELERATION >= CONSTANT_UPWARD_DT))
            {
                Serial.println("Upward");
                TRANSITION_ACTIVATE_ACTUATOR_SL();
            }
            else if((VARIABLE_UPDOWN == 1)&&(VARIABLE_COMPENSATED_DECELERATION >= CONSTANT_DOWNWARD_DT))
            {
                Serial.println("Downward");
                TRANSITION_DEACTIVATE_ACTUATOR_SL();
            }    
        }

        while((millis() - decision_time) < (1000/VARIABLE_STRAIGHTANDLEVEL_DECISION_RATE));
}

void TRANSITION_ACTIVATE_ACTUATOR_SL()
{
	//VARIABLE_CURRENT_MODE = "transition_activate_actuator_sl";
	BRAKING_SIGNAL_ON();
	VARIABLE_UPDOWN = 1;
}

void TRANSITION_DEACTIVATE_ACTUATOR_SL()
{
	//VARIABLE_CURRENT_MODE = "transition_deactivate_actuator_sl";
	BRAKING_SIGNAL_OFF();
	VARIABLE_UPDOWN = 0;
}

void SLEEP_SIGNAL()
{
    unsigned long time = millis();
    
    digitalWrite(ACTUATOR_SL,HIGH);
    while(millis() - time < 200);
    digitalWrite(ACTUATOR_SL,LOW);
    delay(50);
    digitalWrite(ACTUATOR_SL,HIGH);
    while(millis() - time < 100);
    digitalWrite(ACTUATOR_SL,LOW);
    delay(50);
    digitalWrite(ACTUATOR_SL,HIGH);
    while(millis() - time < 50);
    digitalWrite(ACTUATOR_SL,LOW);
    while(millis() - time < 1000);
}

void NORMAL_SIGNAL()
{
    unsigned long time = millis();
    
    digitalWrite(ACTUATOR_SL,LOW);
    while(millis() - time < 200);
    digitalWrite(ACTUATOR_SL,HIGH);
    delay(50);
    digitalWrite(ACTUATOR_SL,LOW);
    while(millis() - time < 100);
    digitalWrite(ACTUATOR_SL,HIGH);
    delay(50);
    digitalWrite(ACTUATOR_SL,LOW);
    while(millis() - time < 50);
    digitalWrite(ACTUATOR_SL,HIGH);
    while(millis() - time < 1000);
    digitalWrite(ACTUATOR_SL,LOW);
}

void BRAKING_SIGNAL_ON()
{
    digitalWrite(ACTUATOR_SL,HIGH);
}

void BRAKING_SIGNAL_OFF()
{
    digitalWrite(ACTUATOR_SL,LOW);
}
