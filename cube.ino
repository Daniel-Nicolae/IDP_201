#define echoPin 2 // attach pin D2 Arduino to pin Echo of HC-SR04
#define trigPin 3 //attach pin D3 Arduino to pin Trig of HC-SR04

#include <Wire.h> 
#include <Adafruit_MotorShield.h>
#include "utility/Adafruit_MS_PWMServoDriver.h"

Adafruit_MotorShield AFMS = Adafruit_MotorShield(); 
Adafruit_DCMotor *m1 = AFMS.getMotor(1);
Adafruit_DCMotor *m2 = AFMS.getMotor(2);


// Stuff to initialize
int m1s = 75;
int m2s = 75;
void initialize_motors();
void initialize_sensors();


// Physical parameters of Herbie, used for kinematics calculations
float wheel_R = 0.052;
float half_width = 0.1215;



// Parameters of the current position in the sweep
bool rotation_direction = true;
float theta = 0;
short int cube_found=0;
// 0 - initial sweep
// 1 - found
// 2 - lost on the way
// 3 - picked up


// Things dealing with keeping track of the angle
float last_reset;           //time in seconds from the start to the last time sweeping changed direction
float theta_last_reset;     //angle at which Herbie changed spinning direction. Theoretically it's only +range or -range 
                            //but could deviate slightly so it's safer to keep track of it
float update_theta(float timer, int motor_speed);



// Functions that make Herbie move
void sweep(int range, int motor_speed);
float omega(int m_speed);

void twist_to_target(float theta_target);
void forward(float d, int m_speed, bool searching=1);
void go_to_cube(float d, float theta);

float theta_cube;
float d_cube;



// Keeping track of sweep data
const int sweep_len=10;
float sweep_data[sweep_len][2];

void initialize_sweep_data();
short int sample_period = 200; // ms period for sweep data
void update_sweep_data(int index);

float check_found();
void check_not_lost();

// Stuff for the ultrasonic sensor
long duration;
int distance;
float get_distance();





void setup() 
{
    initialize_sensors();
    initialize_motors();
    //twist_to_target(90);
    //forward(20, 200);

}






void loop() 
{
 
    if (cube_found == 0) // not found yet
    {
          
          sweep(17, 80);
    }
        

    else if (cube_found == 2) // found then lost
    {

        sweep(20  ,80);
    }

    else if (cube_found == 1) // found 
    {
        
        go_to_cube(d_cube, sweep_data[sweep_len-1][1],  theta_cube);

    }
    else if (cube_found == 3) // picked up and back on line
    {
      
    }
   

}


///////////////////////////////////

        //  Functions

///////////////////////////////////


void update_theta(float motor_speed)
{
    float delta_t = millis()/1000.0 - last_reset;
    theta = theta_last_reset+ omega(motor_speed) * delta_t;

}

float omega(int m_speed)
{
    return 18* m_speed/255.0 * 6 *wheel_R/half_width;
}



void sweep(int range, int motor_speed)
{  
    initialize_sweep_data();
    float sweep_start = millis();

    last_reset = millis()/1000.0;
    theta_last_reset = 0;

    short int index=-1;
    short int last_index = -1;

    
    while (cube_found != 1)
    {
        if (rotation_direction)
        {
            m1->run(FORWARD);
            m2->run(BACKWARD);
            update_theta(motor_speed);
    
            if (theta >= range)
            { 
                theta_last_reset = theta;
                rotation_direction = false;
                last_reset = millis()/1000.0;
            }
        }
    
        else
        {
            m1->run(BACKWARD);
            m2->run(FORWARD);
            update_theta(-motor_speed);

            if (theta <= (-1)*range)
            {
                theta_last_reset = theta;
                rotation_direction = true;
                last_reset = millis()/1000.0;
            }
        }


        last_index = index;
        index = (millis()-sweep_start)/sample_period; // integer number of sample periods since the start
        
        if (index != last_index)     // if one more sample period has passed, record new data
            update_sweep_data(index);
        
        
        m1->setSpeed(motor_speed);
        m2->setSpeed(motor_speed);

        check_found(); // This will change the cube_found to 1, therefore exiting the while
        Serial.println(sweep_data[sweep_len-1][1]);
    }
    last_reset = millis()/1000.0;

    
}

void twist_to_target(float theta_target)
{
    m1s=100;
    m2s=100;
    m1->setSpeed(m1s);
    m2->setSpeed(m2s);

    //theta_target*=1.2;

    if (theta_target >= 0)
    {
        m1->run(FORWARD);
        m2->run(BACKWARD);
        
    }
    else
    {
        theta_target*=-1;
        m1->run(BACKWARD);
        m2->run(FORWARD); 
        
    }

    delay (theta_target*1000/omega(m1s));

    m1->setSpeed(0);
    m2->setSpeed(0); 
       
}





void go_to_cube(float d_init, float d_to_go,  float theta)
{
    float d2 = sqrt(d_init*d_init + 30*30 - 2*30*d_init*cos(theta*3.14/180.0));
    float theta2 = asin(30/d2*sin(theta*3.14/180.0))*180/3.14;

    forward(d_to_go-4, 100, 1);
    if (cube_found == 2)
        return;
    
    delay(300);
    twist_to_target(180.0+theta2);
    delay(100);
    forward(d2, 100, 0);

    cube_found = 3;

}



void forward(float d, int m_speed, bool searching=1)
{ 
    short int index=-1;
    short int last_index = -1;

    int start = millis();
    m1->run(FORWARD);
    m2->run(FORWARD);
    m1->setSpeed(m_speed);
    m2->setSpeed(m_speed);

    float deltat = d/100/(18* m_speed/255.0 *3.14/30.0 * wheel_R)*1000;
    Serial.println(deltat);

    while (millis()-start < deltat);// and cube_found == 1)
    {
        /*Serial.println(millis()-start);
        last_index = index;
        index = ((int)(millis()-start*1000))/sample_period;
        if (last_index != index)
            update_sweep_data(2*sweep_len); //for debug purposes, will disappear later */

        if (!searching) check_not_lost(); 
    }
    if (cube_found == 2)
        return;
    m1->setSpeed(0);
    m2->setSpeed(0);

}









float get_distance() //return the read distance in centimeters 
{
    digitalWrite(trigPin, LOW);
    delayMicroseconds(2);
    // Sets the trigPin HIGH (ACTIVE) for 10 microseconds
    digitalWrite(trigPin, HIGH);
    delayMicroseconds(10);
    digitalWrite(trigPin, LOW);
    // Reads the echoPin, returns the sound wave travel time in microseconds
    duration = pulseIn(echoPin, HIGH);
    // Calculating the distance
    return duration * 0.034 / 2;
}

float check_found()
{
    short int threshold = 10;

    if (sweep_data[sweep_len-1][1] == -100)// or sweep_data[sweep_len-1][1] > cutoff)
        return 0;


    if (sweep_data[sweep_len-5][1] - sweep_data[sweep_len-1][1] > threshold &&
        sweep_data[sweep_len-6][1] - sweep_data[sweep_len-2][1] > threshold &&
        sweep_data[sweep_len-7][1] - sweep_data[sweep_len-3][1] > threshold &&
        sweep_data[sweep_len-8][1] - sweep_data[sweep_len-4][1] > threshold)
    {
        if (cube_found == 0)
        {
            theta_cube = sweep_data[sweep_len-1][0];
            d_cube = sweep_data[sweep_len-1][1];
        }
        cube_found = 1;
        return sweep_data[sweep_len-2][0];
    }
}

void check_not_lost()
{
    float threshold = 10;
    if (sweep_data[sweep_len-1][1] - sweep_data[sweep_len-5][1] > threshold &&
        sweep_data[sweep_len-2][1] - sweep_data[sweep_len-6][1] > threshold &&
        sweep_data[sweep_len-3][1] - sweep_data[sweep_len-7][1] > threshold &&
        sweep_data[sweep_len-4][1] - sweep_data[sweep_len-8][1] > threshold)
        cube_found = 2;
}


void update_sweep_data(int index)
{
    if (index < 0) return;
    
    if (index >= sweep_len)  //full
    {
        for (int i=0; i<sweep_len-1; i++)
        {
            sweep_data[i][0]=sweep_data[i+1][0];
            sweep_data[i][1]=sweep_data[i+1][1];
        }
        sweep_data[sweep_len-1][0] = theta;
        sweep_data[sweep_len-1][1] = get_distance();
    }
    else
    {
        sweep_data[index][0] = theta;
        sweep_data[index][1] = get_distance();
    }

    // Printing the data for debug
    
    
}





void initialize_sweep_data()
{
    for (int i=0; i<sweep_len; i++)
    {
        sweep_data[i][0]=-100;  //angle
        sweep_data[i][1]=-100;  //distance
    }
}

void initialize_sensors()
{
    pinMode(trigPin, OUTPUT); // Sets the trigPin as an OUTPUT
    pinMode(echoPin, INPUT); // Sets the echoPin as an INPUT
    
    Serial.begin(9600);
    Serial.println("hi");
}

void initialize_motors()
{
    AFMS.begin();
    m1->setSpeed(m1s);
    m2->setSpeed(m2s);
    m1->run(FORWARD);
    m2->run(FORWARD);

}
