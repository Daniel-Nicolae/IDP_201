
int reading_red, reading_blue;
bool grabbed=false;
short int colour = 0; // red = 1, blue = 2

void check_colour()
{
    unsigned long start = millis();
    colour = 0;
    while (millis()-start < 3000 and colour == 0)
    {
        if(reading_red -reading_blue > 10)
        {
            colour = 1;
            digitalWrite(2, HIGH);
            Serial.println("red");
        }
        else if(reading_blue -reading_red > 10)
        {
            colour = 2;
            digitalWrite(3, HIGH);
            Serial.println("blue");
        }
    }
    if (colour)
    {
        start = millis();
        while (millis()-start < 5000)
        {}
        digitalWrite(2, LOW);
        digitalWrite(3, LOW);
    }
    
}

void setup() 
{
      
    Serial.begin(9600);
    Serial.println("hi");

    pinMode(2, OUTPUT);
    pinMode(3, OUTPUT);
    digitalWrite(2, LOW);
    digitalWrite(3, LOW);


}

void loop() 
{
    reading_blue=analogRead(4) - 822;
    reading_red=analogRead(5) - 614;

    
    check_colour();
    
/*
    Serial.print(reading_blue);
    Serial.print(" ");
    Serial.print(reading_red);
    Serial.print("\n"); */
}
