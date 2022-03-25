// Libraries for the motor shield
#include <Wire.h>
#include <Adafruit_MotorShield.h>
#include "utility/Adafruit_MS_PWMServoDriver.h"

// Declaring variables used for controlling the 4 motors
Adafruit_MotorShield AFMS = Adafruit_MotorShield(); 
Adafruit_DCMotor *m1 = AFMS.getMotor(1);
Adafruit_DCMotor *m2 = AFMS.getMotor(2);
Adafruit_DCMotor *m_grab = AFMS.getMotor(3);
Adafruit_DCMotor *m_lift = AFMS.getMotor(4);

// Distance sensor digital pins
#define echoPin 3 
#define trigPin 2 

// LEDs and start button digital pins
#define redLed 11
#define blueLed 8
#define yellowLed 7 
#define Button 13

// Line sensors analog pins
int analogPin_L = A0;
int analogPin_C = A1;
int analogPin_R = A2;

// Colour sensing variables
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

//Line following variables
// variables to store the analog values read
int reading_L=0;
int reading_C=0;
int reading_R=0;

// Boolean variables for the status of each line sensor
bool left, centre, right;

// Thresholds for the line followers
int limit_low;
int limit_high;

// Motor speeds
int ns = 180;   
int m1s = ns;
int m2s = ns;

// Variables used for navigation
short int stat=2;
short int prevstat;
short int loststat;
short int crosses = 0;
unsigned long tlost = 0;

// Variables used for blinking the amber LED
unsigned long blink_start;
bool yellow_led_status = LOW;


int update_status(bool left, bool centre, bool right);

/// This function is used by Herbie to decide whether he should go forward or steer
void follow_line(void);

/// This function converts the analog reading of a line sensor into a boolean state
bool line_sensor(int value, bool current);

/// This function tells Herbie to get ready to move
void initialise_motors();

/// Herbie decides what to do after he realises he has changed line status
void lfsteer(short int x);

void lost(void);

/// Returns the read distance in metres 
float get_distance(void);

/// Herbie twists for a certain angle
void twist_to_target(float theta_target);

/// Converts a motor speed value into the angular velocity of twisting in deg/s
float omega(int m_speed);

///Takes readings from the LDR to decide whether red or not red and illuminates correct LED
void check_colour();

/// Uses line sensors to allow Herbie to twist until a line on the board is detected
/// sense -> clockwise/anticlockwise, reason = lost/navigating - function varies slightly depending on why twist_to_line is called
void twist_to_line(bool sense, bool reason = 0);

/// Moves forward at a given distance and a given speed
void forward(float d, int m_speed);

/// Picks up the cube
void grab();

/// Puts the cube down
void release();

void go_to_delivery();

/// Returns Herbie to the start box after delivery
void go_home();


void setup()
{
    // Initializing the digital pins
    pinMode(trigPin, OUTPUT); 
    pinMode(echoPin, INPUT); 

    pinMode(redLed, OUTPUT);
    pinMode(blueLed, OUTPUT);
    pinMode(yellowLed, OUTPUT);
    pinMode(Button, INPUT);

    digitalWrite(redLed, LOW);
    digitalWrite(blueLed, LOW);
    digitalWrite(yellowLed, LOW);
        
    // Initializing the serial communication
    Serial.begin(9600);
    Serial.println("hola");

    // The code waits for the start button to be pressed
    bool Button_state = 0;
    while(!button_pressed)
    {
        Button_state = digitalRead(Button);
        if (Button_state == HIGH)
            button_pressed = true;
    }
    Serial.println("go");

    // Motors start running
    initialise_motors();


}

void loop() 
{

    limit_low = 60;
    limit_high = 70;

    m1->run(FORWARD);
    m2->run(FORWARD);
    blink_start = millis(); //amber flashing light
    while (crosses < 3) // follows line until the central cross on the collection half of the board
    {
        follow_line();
    }
    yellow_led_status = LOW;  
    digitalWrite(yellowLed, LOW); // switches off amber light

    crosses = 0; // reset cross counter
    float distance = get_distance(); // sensing distance to cube

    while (distance > 7.0 || distance == 0.0) // continues to follow line until close to the cube
    {
        follow_line();
        distance = get_distance();
        Serial.println(distance);
    };
    forward(distance-0.7, 120); // approaches the cube
    m1->run(RELEASE); //stops
    m2->run(RELEASE);

    check_colour();

    grab();

    twist_to_line(1); // to return to the main line that runs across the board
    crosses = 1; 
    m1->run(FORWARD);
    m2->run(FORWARD);

    // Keep going forward up to the cross in the delivery half
    while(crosses < 3)
    {
        follow_line();
    }

    // Depending on the colour, go to the correct delivery place
    go_to_delivery();

    release();
    go_home();
}


void go_to_delivery()
{
    // From the cross, twist to find the red or blue line leading to the delivery square
    twist_to_line(!colour);

    // Stop when the line followers reach the edge of the blue square
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

void go_home()
{
    // Twist 180 deg, anticlockwise for red, clockwise for blue
    if (colour) twist_to_target(-180); 
    else twist_to_target(180);  


    // Herbie returns to the main line
    forward(16, 180);
    limit_low = 70;
    limit_high = 80;
    twist_to_line(!colour);

    // Herbie follows the main line until he finds home
    crosses = 1;
    while (crosses < 2)
    {
        follow_line();
    }

    // Herbe enters home
    forward(22, 180);
    m1->run(RELEASE);
    m2->run(RELEASE);
    delay(60000);
}


void grab()
{
    unsigned long start = millis();

    // Lower the grabber for 3 seconds at full power
    m_lift->setSpeed(255);
    m_lift->run(FORWARD);
    while (millis()-start < 3000) {}

    // Keep the grabber down at a lower power (without this, the bands would pull it straight back up)
    m_lift->setSpeed(150);

    // Grab at full power for 8 seconds
    m_grab->setSpeed(255);
    m_grab->run(FORWARD);
    start = millis();
    while (millis()-start < 8000) {}

    // Tell the main code that the cube is grabbed
    grabbed = true;

    // Lift the grabber up for 3 seconds. A low power is enough
    start = millis();
    m_lift->setSpeed(80);
    m_lift->run(BACKWARD);
    while (millis()-start < 3000) {}
    m_lift->run(RELEASE);

    // Keep the grabber squeezing, but at a low power
    m_grab->setSpeed(100);

    // Count the cube
    if (colour) nred++;
    else nblue++;
}

void release()
{
    // Lower the grabber for 3 seconds at full power
    unsigned long start = millis();
    m_lift->setSpeed(255);
    m_lift->run(FORWARD);
    while (millis()-start < 3000) {}

    // Keep the grabber down at a lower power (without this, the bands would pull it straight back up)
    m_lift->setSpeed(150);

    // Release the grabber at full speed for 8.5 seconds.
    m_grab->setSpeed(255);
    m_grab->run(BACKWARD);
    start = millis();
    while (millis()-start < 8500) {}
    m_grab->run(RELEASE);

    // Tell the main code that the cube is released
    grabbed = false;

    // Lift the grabber up for 3 seconds. A low power is enough
    start = millis();
    m_lift->setSpeed(80);
    m_lift->run(BACKWARD);
    while (millis()-start < 3000) {}
    m_lift->run(RELEASE);

}


void follow_line()
{
    // Blink the amber LED
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

    // Herbie checks if he's still on line. If he's not he steers
    update_status();
    if (stat != prevstat)
    {
        lfsteer(stat);
    }

    // If he gets lost he must find the line back
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
    Serial.println("hi"); // to indicate motors sucessfully initialised
}

int update_status(void)
/* 
 *  0 0 0 - lost maybe              (returns 0) (run lost algr after 001 or 100)
 *  0 0 1 - drifted to left a lot   (returns 1) (steer right)
 *  0 1 0 - good                    (returns 2) 
 *  0 1 1 - drifted left a bit      (returns 3) (steer right just a bit otherwise it's chaotic)
 *  1 0 0 - drifted right a lot     (returns 4) (steer left)
 *  1 0 1 - keep going              (returns 5) (should very rarely happen)
 *  1 1 0 - drifted right a bit     (returns 6) (steer left just a bit)
 *  1 1 1 - cross or gone sideways  (returns 7) 
 */
{
    // Get line sensor data
    reading_L = analogRead(analogPin_L);
    reading_C = analogRead(analogPin_C);
    reading_R = analogRead(analogPin_R);
    
    // Convert data into booleans
    left = line_sensor(reading_L, left);
    centre = line_sensor(reading_C, centre);
    right = line_sensor(reading_R, right);
    
    // Convert booleans into a current status
    short int value;
    prevstat = stat; 
    value = left*4+centre*2+right;
    if (value!=5) stat = value;

    return value;
}

bool line_sensor(int value, bool current)
{
    // Use 2 limits, a high and a low one, to avoid rapid switching between 0 and 1 if the value varies around one limit
    if (current and value < limit_low) return false;

    if (!current and value > limit_high) return true;

    return current;  
}

void lfsteer(short int x)
{
    // Small right
    if (x == 3)
    {
        m1s = (int) ns*1.15;
        m2s = (int) ns*0.85;
    }
    // Big right
    else if (x == 1)
    {
        m1s = (int) ns*1.2;
        m2s = (int) ns*0.8;
    }
    // Small left
    else if (x == 6)
    {
        m1s = (int) ns*0.85;
        m2s = (int) ns*1.15;
    }
    // Big left
    else if (x == 4)
    {
        m1s = (int) ns*0.8;
        m2s = (int) ns*1.2;
    }
    // Forward
    else if (x == 2)
    {
        m1s = ns;
        m2s = ns;
    }
    // Line cross
    else if (x == 7 && prevstat != 0)
    {
        // Count how many crosses have been found
        crosses = crosses + 1;
        Serial.print("Crosses: "); Serial.println(crosses); // to monitor progress

        // Speed up for the ramp
        if (crosses == 2)
            ns = 210;

        // Slow down after the ramp
        if (crosses == 3)
            ns = 120;

        m1s = ns;
        m2s = ns;
    }

    // Lost
    else if (x == 0)
    {
        m1s = ns;
        m2s = ns;
        // Save the time and the state before getting lost. These are helpful for finding the line again
        tlost = millis(); 
        loststat = prevstat;
    }
    m1->setSpeed(m1s);
    m2->setSpeed(m2s);
    
    /*if (stat == 0) Serial.println("maybe lost");
    if (stat == 1) Serial.println("STEER RIGHT!!!");
    if (stat == 2) Serial.println("Keep going!! :)");
    if (stat == 3) Serial.println("steer right");
    if (stat == 4) Serial.println("STEER LEFT!!!");
    if (stat == 5) Serial.println("nothing");
    if (stat == 6) Serial.println("steer left!");
    if (stat == 7) Serial.println("twist!");*/ //used a lot for debugging line following
}

void lost(void)
{
    unsigned long talert = millis(); // sets a time at which being lost is confirmed (3 seconds after initial 0 status)
    // carries out twist to line with a given direction based on the last line sensor reading before being lost
    if (loststat == 1 or loststat == 3)
    {
        twist_to_line(0, 1); // clockwise for drifted left
    }
    else if (loststat == 4 or loststat == 6)
    {
        twist_to_line(1, 1); // anticlockwise for drifted right
    }
    else // reverses if it is not clear which direction to turn in
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
    if (stat == 0) // cuts power to motors if still lost - would be needed here
    {
      m1->run(RELEASE);
      m2->run(RELEASE);
    }
}


float get_distance() 
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

void twist_to_target(float theta_target)
{
    m1s=180;
    m2s=180;
    m1->setSpeed(m1s);
    m2->setSpeed(m2s);
    unsigned long start = millis();

    // Positive angle means clockwise
    if (theta_target >= 0)
    {
        m1->run(FORWARD);
        m2->run(BACKWARD);   
    }

    // Negative angle means anticlockwise
    else
    {
        theta_target*=-1;
        m1->run(BACKWARD);
        m2->run(FORWARD);   
    }

    // Keep turning for a certain time
    while (millis() - start < theta_target*1000/omega(m1s)) {}
}

float omega(int m_speed)
{
    return 18* m_speed/255.0 * 6 *wheel_R/half_width;
}

void check_colour() // red - true, blue - false
{
    unsigned long start = millis();
    int threshold = 470;

    // Assume cube is blue
    colour = 0;

    // Try to see if it's red for 3 seconds
    while (millis()-start < 3000 and colour == 0)
    {
        // Get colour sensor data
        reading_red=analogRead(3);

        // Compare with the threshold
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

    // Keep the LED on for 5 seconds
    while (millis()-start < 5000) {}
    digitalWrite(redLed, LOW);
    digitalWrite(blueLed, LOW);    
}

void twist_to_line(bool sense, bool reason=0) // 0 clockwise, 1 anticlockwise; 0 navigational, 1 lost
{
    m1s = 180;
    m2s = 180;
    m1->setSpeed(m1s);
    m2->setSpeed(m2s);
    unsigned long start = millis(); //to keep track of spinning time
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
    if (reason == 0) // when twist to line is used in a navigational context, the robot is already on a line
    // so there needs to be a short delay between the start of the twist and the start of line sensing so that
    // Herbie does not latch onto the line that it was already on
    {
        while (millis() - start < 2000)
        {
            update_status();
        }
        while(millis() - start >= 2000 && stat != 2)
        {
            update_status();
        }
    }
    else // when twist to line is being used to find the nearest line, this time delay is not needed
    {
        while(stat != 2)
        {
            update_status();
        }
    }
    Serial.println("found line!");
    m1->run(FORWARD);
    m2->run(FORWARD);
}

void forward(float d, int m_speed) 
{
    m1->run(FORWARD);
    m2->run(FORWARD);
    m1->setSpeed(m_speed);
    m2->setSpeed(m_speed);
    Serial.println("going forward");

    // Go forward for a certain time
    float deltat = d/100/(18* m_speed/255.0 *3.14/30.0 * wheel_R)*1000;

    unsigned long start = millis();
    while (millis()-start < deltat) {}

    m1->run(RELEASE);
    m2->run(RELEASE);

}
