#include <Wire.h>
#include <Adafruit_MotorShield.h>
#include "utility/Adafruit_MS_PWMServoDriver.h"

Adafruit_MotorShield AFMS = Adafruit_MotorShield(); 
Adafruit_DCMotor *m1 = AFMS.getMotor(1);
Adafruit_DCMotor *m2 = AFMS.getMotor(2);
Adafruit_DCMotor *m_grab = AFMS.getMotor(3);
Adafruit_DCMotor *m_lift = AFMS.getMotor(4);

#define echoPin 3 // attach pin D2 Arduino to pin Echo of HC-SR04
#define trigPin 2 //attach pin D3 Arduino to pin Trig of HC-SR04
#define redLed 11
#define blueLed 8
#define yellowLed 6 
#define Button 13

//colour sensing variables
int reading_red, reading_blue;
bool grabbed=false;
short int colour = 0; // red = 1, blue = 2
short int nred = 0;
short int nblue = 0;

bool button_pressed = false;

//physical parameters
int duration;
float theta_target; //TRY REMOVING THIS - SHOULDN'T CAUSE PROBLEMS
float wheel_R = 0.052;
float half_width = 0.1215;
float dwlf = 0.18; //distance between line followers and wheels
bool sense;
bool searching = 0;

//line following parameters
int analogPin_L = A2;
int analogPin_C = A1;
int analogPin_R = A0;

int reading_L=0;
int reading_C=0;
int reading_R=0;  // variables to store the values read

bool left, centre, right;
int limit_low = 50;
int limit_high = 60;

// motor speeds
int ns = 180;   
int m1s = ns;
int m2s = ns;

void follow_line(void);

bool line_sensor(int value, bool current);

void initialise_motors();

short int stat=2;
short int prevstat;

short int crosses = 0;
unsigned long blink_start;
bool yellow_led_status = LOW;

unsigned long tlost = 0; //time at which line lost

int update_status(bool left, bool centre, bool right);

void lfsteer(short int x);

void lost(void);

float get_distance(void);

void twist_to_target(float theta_target);

float omega(int m_speed);

void check_colour();

void twist_to_line(bool sense);

void forward(float d, int m_speed, bool searching=1);

void grab();

void release();

void reset_grabber(int t)
{
    m_grab->run(BACKWARD);
    m_grab->setSpeed(255);
    delay(t);
    m_grab->run(RELEASE);
}

void setup()
{
    pinMode(trigPin, OUTPUT); // Sets the trigPin as an OUTPUT
    pinMode(echoPin, INPUT); // Sets the echoPin as an INPUT

    pinMode(redLed, OUTPUT);
    pinMode(blueLed, OUTPUT);
    pinMode(yellowLed, OUTPUT);
    pinMode(Button, INPUT);
    digitalWrite(redLed, LOW);
    digitalWrite(blueLed, LOW);
    digitalWrite(yellowLed, LOW);
        
    Serial.begin(9600);


    bool Button_state = 0;
    /*while(!button_pressed)
    {
        Button_state = digitalRead(Button);
        if (Button_state == HIGH)
            button_pressed = true;
    }*/

    initialise_motors();

    //grab();
    //reset_grabber(3000);
    //delay(600);
    //release();
    //forward(5, 120);

}

void loop() {
  while(Serial.available() > 0)
  {
    char comm = Serial.read();
    if (comm == 'q')
    {
      Serial.println("Grab!");
      grab(); 
    }
    else if (comm == 'r')
    {
      Serial.println("Release!");
      release();
    }
    else if (comm == ' ')
    {
      reset_grabber(1000);
    }
    comm = 'x'; 
    
}
}

void grab()
{
    unsigned long start = millis();
    m_lift->setSpeed(255);
    m_lift->run(FORWARD);
    while (millis()-start < 3000)
    {

    }
    m_lift->setSpeed(150);
    //delay(500);
    m_grab->setSpeed(255);
    m_grab->run(FORWARD);

    start = millis();
    while (millis()-start < 6000)
    {

    }
    //m_grab->setSpeed(200);
    //m_grab->run(RELEASE);
    
    grabbed = true;

    start = millis();
    m_lift->setSpeed(80);
    m_lift->run(BACKWARD);
    while (millis()-start < 3000)
    {

    }
    m_lift->run(RELEASE);
    while (millis()-start < 4000)
    {

    }    
    //m_grab->run(RELEASE);
}

void release()
{
    unsigned long start = millis();
    m_lift->setSpeed(255);
    m_lift->run(FORWARD);
    while (millis()-start < 3000)
    {

    }
    m_lift->setSpeed(150);
    //delay(500);
    m_grab->setSpeed(255);
    m_grab->run(BACKWARD);

    start = millis();
    while (millis()-start < 4000)
    {

    }
    m_grab->run(RELEASE);

    grabbed = false;

    start = millis();
    m_lift->setSpeed(80);
    m_lift->run(BACKWARD);
    while (millis()-start < 3000)
    {

    }
    m_lift->run(RELEASE);

}

void initialise_motors()
{
  AFMS.begin();
  m1->setSpeed(m1s);
  m2->setSpeed(m2s);
  m_grab->setSpeed(0);
  m_lift->setSpeed(0);
  //m1->run(FORWARD);
  //m2->run(FORWARD);
  Serial.println("hi");
  //START FLASHING AMBER LIGHT
}
