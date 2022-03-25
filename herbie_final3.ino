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
#define yellowLed 7 
#define Button 13

//colour sensing variables
int reading_red, reading_blue;
bool grabbed=false;
short int colour; // red = true, blue = false
short int nred = 0;
short int nblue = 0;

bool button_pressed = false;

// For the distance sensor
long duration;

//physical parameters
float wheel_R = 0.052;
float half_width = 0.1215;
float dwlf = 0.18; //distance between line followers and wheels
bool sense;
bool searching = 0;

//line following parameters
int analogPin_L = A0;
int analogPin_C = A1;
int analogPin_R = A2;

int reading_L=0;
int reading_C=0;
int reading_R=0;  // variables to store the values read

bool left, centre, right;
int limit_low = 60;
int limit_high = 70;

// motor speeds
int ns = 180;   
int m1s = ns;
int m2s = ns;

void follow_line(void);

bool line_sensor(int value, bool current);

void initialise_motors();

short int stat=2;
short int prevstat;
short int loststat;

short int crosses = 0;
unsigned long blink_start;
bool yellow_led_status = LOW;

// Gathering sweep data
const int sweep_len=10;
float distance_data[sweep_len];
short int sample_period = 30; // ms period for sweep data

// Parameters of the current position in the sweep
short int cube_found=0;
// 0 - initial sweep
// 1 - found
// 2 - lost on the way
// 3 - picked up

unsigned long tlost = 0; //time at which line lost
bool cube_right_side = true;

int update_status(bool left, bool centre, bool right);

void lfsteer(short int x);

void lost(void);

float get_distance(void);

void twist_to_target(float theta_target);

float omega(int m_speed);

void check_colour();

void twist_to_line(bool sense, bool reason = 0);

void forward(float d, int m_speed, bool searching=0);

void grab();

void release();

void go_to_delivery();

void back_to_line();

// Sweeping and detecting functions

bool sweep(int range, int motor_speed);

void initialize_distance_data();

void update_distance_data(int index);

void go_to_cube(float d_to_go);

void check_found();

void check_not_lost();

void find_cube(); //the function that puts together all the sweeping

void back_to_delivery();

void correct_grab();


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
    Serial.println("hola");



    bool Button_state = 0;
    while(!button_pressed)
    {
        Button_state = digitalRead(Button);
        if (Button_state == HIGH)
            button_pressed = true;
    }
    Serial.println("go");
    initialize_distance_data();
    initialise_motors();

    twist_to_target(-20);



}
void loop() 
{/*
    m1->run(FORWARD);
    m2->run(FORWARD);
    blink_start = millis();
    while (crosses < 3)
    {
        follow_line();
    }
    yellow_led_status = LOW;  
    digitalWrite(yellowLed, LOW);

    crosses = 0;
    Serial.println(get_distance());
    float distance = get_distance();

    while (distance > 7.0 || distance == 0.0)
    {
        follow_line();

        distance = get_distance();
        Serial.println(distance);
    }
    Serial.println("aaaaa");
    Serial.println(distance);
    forward(distance-0.7, 120); //*************
    m1->run(RELEASE);
    m2->run(RELEASE);

*/

    //sweeping
    find_cube();


    check_colour();

    grab();

    //delay(1000);

    release();

    delay(60000);

    //back_to_delivery();

    //m1->run(RELEASE);
    //m2->run(RELEASE);
    
    //release();

    /*
    twist_to_line(1);  //******************* /wheel speeds
    crosses = 1; //this might need to change depending on how the 180 turn works out
    m1->run(FORWARD);
    m2->run(FORWARD);

    
    while(crosses < 3)
    {
        follow_line();
    }
    */


/*
    forward(36, 180);
    twist_to_line(0);
    while (crosses < 4)
    {
        follow_line();
    }

    // Here Herbie moves a bit backwards to place the block in the correct place
    m1->run(BACKWARD);
    m2->run(BACKWARD);
    start = millis();
    while (millis()-start < 650) {}

    m1->run(RELEASE);
    m2->run(RELEASE);*/
    
    //go_to_delivery();

    //release();

/*
    twist_to_target(180);

    m1->setSpeed(ns);
    m2->setSpeed(ns);
    m1->run(FORWARD);
    m2->run(FORWARD);

    // Herbie returns to the main line
    forward(11, 180);
    twist_to_line(1);


    crosses = 1;*/
    //back_to_line();

    //TURN 90 DEPENDING ON COLOUR
    //BOX FOLLOWING!!!
    //CUBE DELIVERY


}

void find_cube()
{
    cube_found == 0;
    while (cube_found != 3)
    {
        if (cube_found == 0) // not found yet
        {
            sweep(700, 120);
        }
        

        else if (cube_found == 2) // found then lost
        {
            sweep(700, 120);
        }

        else if (cube_found == 1) // found 
        {
            go_to_cube(distance_data[sweep_len-1]);
        }

    }
    correct_grab();    
}

void go_to_delivery()
{
    if ((colour && nred == 1) || (!colour && nblue == 1))
        forward(30, 180);

    twist_to_line(!colour);
    while (crosses < 4)
    {
      follow_line();
    }

    // Here Herbie moves a bit backwards to place the block in the correct place
    m1->run(BACKWARD);
    m2->run(BACKWARD);
    m1->setSpeed(120);
    m2->setSpeed(120);
    unsigned long back_start = millis();
    while (millis()-back_start < 700) {}

    m1->run(RELEASE);
    m2->run(RELEASE);
}

void back_to_line()
{
    if (colour) twist_to_target(-190); //anticl if it was red, to not hit the ramp after the second block
    else twist_to_target(190);  //same reason for clockwise at blue


    // Herbie returns to the main line
    forward(13, 180);
    twist_to_line(colour);

    if ((colour && nred == 1) || (!colour && nblue == 1))
      crosses = 1;
    else crosses = 2;
}


void grab()
{
    unsigned long start = millis();
    m_lift->setSpeed(255);
    m_lift->run(FORWARD);
    while (millis()-start < 4500)
    {

    }
    m_lift->setSpeed(150);
    //delay(500);
    m_grab->setSpeed(255);
    m_grab->run(FORWARD);

    start = millis();
    while (millis()-start < 7500)
    {

    }
    //m_grab->setSpeed(200);
    //m_grab->run(RELEASE);
    
    grabbed = true;
    m_grab->setSpeed(100);
    start = millis();
    m_lift->setSpeed(80);
    m_lift->run(BACKWARD);
    while (millis()-start < 1600)
    {

    }
    m_lift->run(RELEASE);
    /*while (millis()-start < 4000)
    {

    }    */

    if (colour) nred++;
    else nblue++;
}

void release()
{
    unsigned long start = millis();
    m_lift->setSpeed(255);
    m_lift->run(FORWARD);
    while (millis()-start < 3500)
    {
    
    }
    m_lift->setSpeed(150);
    //delay(500);
    m_grab->setSpeed(255);
    m_grab->run(BACKWARD);

    start = millis();
    while (millis()-start < 8000)
    {

    }
    m_grab->run(RELEASE);

    grabbed = false;

    start = millis();
    m_lift->setSpeed(80);
    m_lift->run(BACKWARD);
    while (millis()-start < 1600)
    {

    }
    m_lift->run(RELEASE);

}


void follow_line()
{
    //Serial.println(get_distance());

    if (millis()-blink_start > 250)
    {
        blink_start = millis();
        if (yellow_led_status == LOW)
        {
            yellow_led_status = HIGH;
            digitalWrite(yellowLed, HIGH);
        }
        else
        {
            yellow_led_status = LOW;
            digitalWrite(yellowLed, LOW);
        }
    }


    update_status();
    if (stat != prevstat)
    {
        lfsteer(stat);
    }
    else if (stat == 0 && millis() - tlost > 3000)
    {
        Serial.println("Running lost");
        Serial.println(tlost);
        lost();
    }
}

void initialise_motors()
{
    AFMS.begin();
    m1->setSpeed(m1s);
    m2->setSpeed(m2s);
    m_grab->setSpeed(0);
    m_lift->setSpeed(0);
    m1->run(FORWARD);
    m2->run(FORWARD);
    Serial.println("hi");
}

int update_status(void)
/* 
 *  0 0 0 - lost maybe              (returns 0) (run lost algr after 001 or 100)
 *  0 0 1 - drifted to left a lot   (returns 1) (steer right)
 *  0 1 0 - good                    (returns 2) 
 *  0 1 1 - drifted left a bit      (returns 3) (steer right just a bit otherwise it's chaotic)
 *  1 0 0 - drifted right a lot     (4)         (steer left)
 *  1 0 1 - keep going              (5)
 *  1 1 0 - drifted right a bit     (6)         (steer left just a bit)
 *  1 1 1 - cross or gone sideways  (7)         (stop and twist)
 */
{
    reading_L = analogRead(analogPin_L);
    reading_C = analogRead(analogPin_C);
    reading_R = analogRead(analogPin_R);// read the input pins
    
    left = line_sensor(reading_L, left);
    centre = line_sensor(reading_C, centre);
    right = line_sensor(reading_R, right);
    
    short int value;
    prevstat = stat; 
    value = left*4+centre*2+right;
    if (value!=5) stat = value;
    //if (value == 0 and (stat == 4 or stat == 1)) stat = 0;
    return value;
}

bool line_sensor(int value, bool current)
{
    if (current and value < limit_low) return false;

    if (!current and value > limit_high) return true;

    return current;  
}

void lfsteer(short int x)
{
    Serial.println(stat);
    //small right
    if (x == 3)
    {
        m1s = (int) ns*1.15;
        m2s = (int) ns*0.85;
        Serial.println(m2s);
    }
    //big right
    else if (x == 1)
    {
        m1s = (int) ns*1.2;
        m2s = (int) ns*0.8;
    }
    //small left
    else if (x == 6)
    {
        m1s = (int) ns*0.85;
        m2s = (int) ns*1.15;
    }
    //big left
    else if (x == 4)
    {
        m1s = (int) ns*0.8;
        m2s = (int) ns*1.2;
    }
    //forward
    else if (x == 2)
    {
        m1s = ns;
        m2s = ns;
    }
    //line cross
    else if (x == 7 && prevstat != 0)
    {
        crosses = crosses + 1;
        Serial.print("Crosses: "); Serial.println(crosses);
        if (crosses == 2)
        {
            ns = 210;
        }
        if (crosses == 3)
        {
            ns = 120;
        }
        m1s = ns;
        m2s = ns;
    }
    //lost
    else if (x == 0)
    {
        m1s = ns;
        m2s = ns;
        tlost = millis(); 
        loststat = prevstat;
    }
    m1->setSpeed(m1s);
    m2->setSpeed(m2s);
    
    if (stat == 0) Serial.println("maybe lost");
    /*if (stat == 1) Serial.println("STEER RIGHT!!!");
    if (stat == 2) Serial.println("Keep going!! :)");
    if (stat == 3) Serial.println("steer right");
    if (stat == 4) Serial.println("STEER LEFT!!!");
    if (stat == 5) Serial.println("nothing");
    if (stat == 6) Serial.println("steer left!");
    if (stat == 7) Serial.println("twist!");*/
    Serial.print("m1s: "); Serial.println(m1s);
    Serial.print("m2s: "); Serial.println(m2s);
    Serial.println(" ");
}
void lost(void)
{
    unsigned long talert = millis();
    Serial.print("Lost at "); Serial.println(talert);
    Serial.println(loststat);
    if (loststat == 1 or loststat == 3)
    {
        twist_to_line(0, 1);
    }
    else if (loststat == 4 or loststat == 6)
    {
        twist_to_line(1, 1);
    }
    else
    {
        m1->run(BACKWARD);
        m2->run(BACKWARD);
        while(millis() - talert < 5000)
        {
            update_status();
            if (stat != 0)
            {
                break;
            }
        }
    }
    m1->run(FORWARD);
    m2->run(FORWARD); 
    if (stat == 0)
    {
      Serial.println("Very lost");
      unsigned long tvlost = millis();
      m1->run(RELEASE);
      m2->run(RELEASE);
      //MAYBE SOMETHING NEEDED TO HANDLE THIS
    }
}


float get_distance() //return the read distance in meters 
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

void twist_to_target(float theta_target) //positive angle means clockwise
{
    m1s=130;
    m2s=130;
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

    unsigned long start = millis();
    while (millis() < theta_target*1000/omega(m1s)) 
    {}

    m1->run(RELEASE);
    m2->run(RELEASE);
}

float omega(int m_speed)
{
    return 18* m_speed/255.0 * 6 *wheel_R/half_width;
}

void check_colour() // red - true, blue - false
{
    unsigned long start;
    int threshold = 650;

    reading_red=analogRead(3);
    
    colour = 0;
    start = millis();
    while (millis()-start < 3000 and colour == 0)
    {
        if (reading_red > threshold)
        {
            colour = 1;
            digitalWrite(redLed, HIGH);
            Serial.println("red");
        }
    }
    if (!colour)
    {
        digitalWrite(blueLed, HIGH);
        Serial.println("blue");
    }
    start = millis();
    while (millis()-start < 5000)
    {}
    digitalWrite(redLed, LOW);
    digitalWrite(blueLed, LOW);    
}

void twist_to_line(bool sense, bool reason=0) // 0 clockwise, 1 anticlockwise; 0 navigational, 1 lost
{
    Serial.println("beginning ttl");
    m1s = 180;
    m2s = 180;
    m1->setSpeed(m1s);
    m2->setSpeed(m2s);
    unsigned long start = millis();
    if (sense == 0)
    {
        m1->run(FORWARD);
        m2->run(BACKWARD);
    }
    else
    {
        m1->run(BACKWARD);
        m2->run(FORWARD);
    }
    if (reason == 0)
    {
        while (millis() - start < 2000)
        {
            update_status();
        }
        //Serial.println(stat);
        while(millis() - start >= 2000 && stat != 2)
        {
            update_status();
            //Serial.println(stat);
            //Serial.println(millis());
        }
    }
    else
    {
        while(stat != 2)
        {
            update_status();
            //Serial.println(stat);
            //Serial.println(millis());
        }
    }
    Serial.println("found line!");
    m1->run(FORWARD);
    m2->run(FORWARD);
}



void forward(float d, int m_speed, bool searching=0) // 1 if we're looking for cubes, 0 if just navigation
{ 
    if (d<2) return;
    
    short int index=-1;
    short int last_index = -1;

    
    m1->run(FORWARD);
    m2->run(FORWARD);
    m1->setSpeed(m_speed);
    m2->setSpeed(m_speed);
    //m2->setSpeed((int) m_speed*0.85);

    float deltat = d/100/(18* m_speed/255.0 *3.14/30.0 * wheel_R)*1000;

    unsigned long start = millis();
    unsigned long blink_start = millis();
    while (millis()-start < deltat && (cube_found == 1 || !searching) && (distance_data[sweep_len-1] > 8 || !searching))
    {
        last_index = index;
        index = ((int)(millis()-start*1000))/sample_period;
        if (last_index != index)
            update_distance_data(2*sweep_len);

        if (searching) 
        {
          Serial.print("F: ");         Serial.print(cube_found); Serial.print(" ");
          Serial.println(distance_data[sweep_len-1]);
          check_not_lost(); 
        }
    }
    
    if (cube_found == 2)
        return;
    m1->run(RELEASE);
    m2->run(RELEASE);

}

void check_found()
{
    short int threshold = 10;

    if (distance_data[sweep_len-1] == -100)// or distance_data[sweep_len-1][1] > cutoff)
        return 0;


    if ((distance_data[sweep_len-5] - distance_data[sweep_len-1] > threshold &&
        distance_data[sweep_len-6] - distance_data[sweep_len-2] > threshold &&
        distance_data[sweep_len-7] - distance_data[sweep_len-3] > threshold &&
        distance_data[sweep_len-8] - distance_data[sweep_len-4] > threshold))
    {
        if (distance_data[sweep_len-1] < 80) cube_found = 1;
    }
}

void check_not_lost()
{
    float threshold = 10;
    if (distance_data[sweep_len-1] - distance_data[sweep_len-5] > threshold &&
        distance_data[sweep_len-2] - distance_data[sweep_len-6] > threshold)
        cube_found = 2;
}


void update_distance_data(int index)
{
    if (index < 0) return;
    
    if (index >= sweep_len)  //full
    {
        for (int i=0; i<sweep_len-1; i++)
        {
            distance_data[i]=distance_data[i+1];
        }
        distance_data[sweep_len-1] = get_distance();
    }
    else
    {
        distance_data[index] = get_distance();
    }

    // Printing the data for debug
    
    
}


void initialize_distance_data()
{
    for (int i=0; i<sweep_len; i++)
        distance_data[i]=-100;  //distance
}

bool sweep(int range, int motor_speed)
{  
    bool Right;
    unsigned long sweep_start = millis();

    unsigned long last_reset = millis(); //time in seconds from the start to the last time sweeping changed direction


    short int index=-1;
    short int last_index = -1;

    bool rotation_direction = true;

    m1->run(FORWARD);
    m2->run(BACKWARD);
    m1->setSpeed(motor_speed);
    m2->setSpeed(motor_speed);

    Right = true;
    while (cube_found != 1)
    {

        while ((millis() - last_reset < range) && cube_found != 1)
        {
            last_index = index;
            index = (millis()-sweep_start)/sample_period; // integer number of sample periods since the start
            
            if (index != last_index)     // if one more sample period has passed, record new data
                update_distance_data(index);  
            check_found(); 
            Serial.print("T: "); Serial.print(cube_found); Serial.print(" ");
            Serial.println(distance_data[sweep_len-1]);

        }
        last_reset = millis();

        if (rotation_direction)
        {    
            if (Right)
            {
                rotation_direction = false;
                m1->run(BACKWARD);
                m2->run(FORWARD);
            }
            else 
            {
                Right = true;
                if (cube_found == 0) cube_right_side = true;
            }
        }
    
        else
        {
            if (!Right)
            {
                rotation_direction = true;
                m1->run(FORWARD);
                m2->run(BACKWARD);
            }
            else
            {
                Right = false;
                if (cube_found == 0) cube_right_side = false;
            }
        }

         // This will change the cube_found to 1, therefore exiting the while
    }
    //delay(100);
    m1->run(RELEASE);
    m2->run(RELEASE);

    return rotation_direction;
    
}


void go_to_cube(float d_to_go)
{
    if (d_to_go > 8)
        forward(d_to_go, 120, 1);

    if (cube_found == 2)
        return;
    
    cube_found = 3;
}

void back_to_delivery()
{
    unsigned long start = millis();
    m1->setSpeed(120);
    m2->setSpeed(120);
    m1->run(BACKWARD);
    m2->run(BACKWARD);

    while (millis()-start < 2000) {}

    twist_to_line(!cube_right_side);
    
    return;
}
 
void correct_grab()
{
    m1->setSpeed(120);
    m2->setSpeed(120);
    m1->run(BACKWARD);
    m2->run(FORWARD);
    unsigned long start = millis();
    while (millis() - start < 600) {}

    
    bool clockwise = sweep(1000, 100);
    
    delay(3000);
    Serial.println(clockwise);
    if (clockwise)
    {
        m1->run(FORWARD);
        m2->run(BACKWARD);
    }
    else
    {
        m2->run(FORWARD);
        m1->run(BACKWARD);
    }
    m1->setSpeed(120);
    m2->setSpeed(120);
    
    start = millis();
    while (millis() - start < 280) {}

    m1->run(RELEASE);
    m2->run(RELEASE);
    

    forward(distance_data[sweep_len-1]-1, 110);
}
