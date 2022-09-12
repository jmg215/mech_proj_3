//Jon Grote

//definitely need the gamepad module
#define INCLUDE_GAMEPAD_MODULE
#define INCLUDE_TERMINAL_MODULE
#define INCLUDE_PINMONITOR_MODULE
#define CUSTOM_SETTINGS
#define INCLUDE_LEDCONTROL_MODULE

//must have the dabble library to communicate through the application to the blutooth
#include <Dabble.h>
#include <QTRSensors.h>
#include <stdlib.h>

QTRSensors qtr;

const uint8_t SensorCount = 8;
uint16_t sensorValues[SensorCount];

bool gStartMeasuring = false;

//Left wheel direction
const int LDIR = 41;
//right wheel direction
const int RDIR = 42;
//left pwm
const int LPWM = 45;
//right pwm
const int RPWM = 46;
// era pin
const int era = 18;
//ela pin
const int ela = 19;
//the dutycycle for all motor inputs
//want 70
float dutyCycle = 255 * (13.5 / 100.00); //.13.5
// make greater than 1 and watch for weird behavior
//use volatile to indicate to the compiler that the value of these variables may change without its awarness
volatile long eracount = 0;
volatile long elacount = 0;

const int rtrigPin = 34;
const int rechoPin = 35;
const int ltrigPin = 36;
const int lechoPin = 37;

//here we go
//this is the best for half speed
//double KpR = 0.09, KiR = 0.03, KdR = 1.1;

/*
another core part of the PID control setup, these are the proportional, integral, and derivative constants */
//double KpR = 0.09, KiR = 0.03, KdR = 1.1;
//double KpR = 0.029, KiR = 0.010, KdR = 0.0008795;
double KpR = 0.0010, KiR = 0.00, KdR = 0.002795;
//double KpR = 0.025, KiR = 0.007, KdR = 0.0008795;//dead battery
int lastError = 0;

const int buttonPin = 11;
const int buttonPin2 = 12;
bool gonogo = false;
int Lvar;
int Rvar;
int Rfar;

//int error;

void setup()
{
    pinMode(LED_BUILTIN, OUTPUT);
    // configure the sensors
    qtr.setTypeRC();
    qtr.setSensorPins((const uint8_t[]){A0, A1, A2, A3, A4, A5, A6, A7}, SensorCount);

    attachInterrupt(digitalPinToInterrupt(ela), leftwheel, CHANGE);
    //setup pin 18 to trigger an ISR when a change is detected
    attachInterrupt(digitalPinToInterrupt(era), rightwheel, CHANGE);

    //both the inputs for era and ela are INPUT_PULLUP
    pinMode(era, INPUT_PULLUP);
    pinMode(ela, INPUT_PULLUP);

    //set up pinModes here

    pinMode(LDIR, OUTPUT);
    pinMode(RDIR, OUTPUT);
    pinMode(LPWM, OUTPUT);
    pinMode(RPWM, OUTPUT);
    pinMode(buttonPin, INPUT_PULLUP);
    pinMode(buttonPin2, INPUT_PULLUP);
    pinMode(rtrigPin, OUTPUT);
    pinMode(rechoPin, INPUT);
    pinMode(ltrigPin, OUTPUT);
    pinMode(lechoPin, INPUT);

    //being Serial communication between the mega and computer and the mega and the Blutooth using Dabble library
    Serial.begin(115200);
    //Dabble.begin(9600);
} //######### end setup routine ##############

void loop()
{

    //Dabble.processInput();

    double Rposition = distance(rtrigPin, rechoPin);
    double Lposition = distance(ltrigPin, lechoPin);
    delay(100);
    Serial.print(Lposition);
    Serial.print(" ");
    Serial.println(Rposition);

    if (digitalRead(buttonPin2) == LOW && digitalRead(buttonPin) == LOW)
    {
        CalibrateSensorArray();
    }
    if (digitalRead(buttonPin2) == LOW)
    {
        gStartMeasuring = true;
    }
    else if (digitalRead(buttonPin) == LOW)
    {
        gStartMeasuring = false;
    }
    // else // w/ this line commented out, robot went on its own without dabble.cross()
    // {
    //     gStartMeasuring = false;
    // }
    if (gStartMeasuring == true)
    {
        // read calibrated sensor values and obtain a measure of the line position
        // from 0 to 7000 (for a white line, use readLineWhite() instead)
        uint16_t position = qtr.readLineBlack(sensorValues);

        /*
        this is the core of the PID control funtionality. set point is the position of 3500,
        and the uint16_t "position" is the process variable.
        base speed is 40% of the duty cycle */
        int error = 3500 - position;
        int P = error;
        int I = I + error;
        int D = error - lastError;
        lastError = error;
        int speed = P * KpR + I * KiR + D * KdR;
        int X;
        int Y;

        float Rbase = 255 * .07; //255/2 .4
        float Lbase = 255 * .07; //.07
        float RWspeed = Rbase - speed;
        float LWspeed = Lbase + speed;
        float RWmax = dutyCycle;
        float LWmax = dutyCycle;

        /*
        
        A series of speed evaluation procedures. never allow a speed over the max speed, 
        which is the duty cycle. if a speed is negative, allow for motor reversal. motor direction
        is indicated by variables X and Y, and can be set to 1 (HIGH - backwards)
        and 0 (LOW - forwards) */
        if (RWspeed > RWmax)
        {
            RWspeed = RWmax;
            X = 0;
            Y = 0;
        }
        if (LWspeed > LWmax)
        {
            LWspeed = LWmax;
            X = 0;
            Y = 0;
        }
        if (RWspeed < 0)
        {
            RWspeed = fabs(RWspeed);
            if (RWspeed > RWmax)
            {
                RWspeed = RWmax;
            }
            X = 1;
            Y = 0;
        }
        if (LWspeed < 0)
        {
            LWspeed = fabs(LWspeed);
            if (LWspeed > LWmax)
            {
                LWspeed = LWmax;
            }
            X = 0;
            Y = 1;
        }
        //if the ultrasonic sensors pick up data such that there is an open gap to the right
        //and a wall in front, turn right and don't hit the wall! all position values in cm
        if (Rposition > 20 && Lposition > 12 && Lposition < 250)
        {
            Rvar++;
            Serial.print(" lvar");Serial.println(Rvar);
            if (Rvar >= 2)
            {
                analogWrite(RPWM, 0);
                analogWrite(LPWM, 0);
                delay(10);
                digitalWrite(RDIR, LOW); //low is forwards
                digitalWrite(LDIR, LOW);
                analogWrite(RPWM, dutyCycle);
                analogWrite(LPWM, dutyCycle);
                //delay(1350); // dead battery value
                //delay(950);
                eracount = 0;

                //this is a measured distance of ~7 inches. necessary to go the correct distance and put 
                //the line sensor positioned over the line closest to its middle
                while(eracount < 507)
                {
                digitalWrite(RDIR, LOW); //low is forwards
                digitalWrite(LDIR, LOW);
                analogWrite(RPWM, dutyCycle);
                analogWrite(LPWM, dutyCycle);  
                }
                analogWrite(RPWM, 0);
                analogWrite(LPWM, 0);
                eracount = 0;
                TurnRight();
            }

            eracount = 0;

        } 
        //the turn left routine. for all turning routines there is a count needed to ensure the 
        //positions read weren't noise. here 2 counts or more. 
        else if (Rposition < 20 && Lposition <= 13 && Lposition > 4)//12 for dead battery
        {
            Lvar++;
            Serial.print(" lvar");Serial.println(Lvar);
            if (Lvar >= 2)
            {
                eracount = 0;
                TurnLeft();
            }
            eracount = 0;
        }
        else if (Rposition > 20 && Lposition < .31)
        {
            //essentially this is for the times when the front (left) sensor craps out with noise, but there is
            //actually a right hand turn that must be navigated. last 2 corners of the maze in particular
            Rfar++;
            Serial.print(" rfar");Serial.println(Rfar);
            if (Rfar > 4)
            {
                
                analogWrite(RPWM, 0);
                analogWrite(LPWM, 0);
                delay(10);
                digitalWrite(RDIR, LOW); //low is forwards
                digitalWrite(LDIR, LOW);
                analogWrite(RPWM, dutyCycle);
                analogWrite(LPWM, dutyCycle); 
                //delay(1350); // dead battery
                //delay(950);
                eracount = 0;
                while(eracount < 498)
                {
                digitalWrite(RDIR, LOW); //low is forwards
                digitalWrite(LDIR, LOW);
                analogWrite(RPWM, dutyCycle);
                analogWrite(LPWM, dutyCycle);  
                }
                analogWrite(RPWM, 0);
                analogWrite(LPWM, 0);
                eracount = 0;
                TurnRight();
            }
        }
        else
        {
            Rvar = 0;
            Lvar = 0;
            Rfar = 0;
            LewisHamilton(RWspeed, LWspeed, X, Y);
            eracount = 0;
        }
    }
    else
    {
        analogWrite(RPWM, 0);
        analogWrite(LPWM, 0);
    }
}

void CalibrateSensorArray()
{
    digitalWrite(LED_BUILTIN, HIGH); // turn on Arduino's LED to indicate we are in calibration mode
    // analogRead() takes about 0.1 ms on an AVR.
    // 0.1 ms per sensor * 4 samples per sensor read (default) * 8 sensors
    // * 10 reads per calibrate() call = ~32 ms per calibrate() call.
    // Call calibrate() 400 times to make calibration take about 13 seconds.
    for (uint16_t i = 0; i < 450; i++)
    {
        qtr.calibrate();
    }
    digitalWrite(LED_BUILTIN, LOW); // turn off Arduino's LED to indicate we are through with calibration
}
/*
this is the main drive function. Lewis Hamilton, while I am not a fan of him as a driver,
he is someone I have a lot of respect for. tied with Michael Schumacher
with 7x F1 world champion, and the winningest driver in F1 history. */
void LewisHamilton(float right, float left, int RD, int LD)
{
    digitalWrite(RDIR, RD);
    digitalWrite(LDIR, LD);
    analogWrite(RPWM, right);
    analogWrite(LPWM, left);
}
//interrupts for the isr
void leftwheel()
{
    elacount++;
}
void rightwheel()
{
    eracount++;
}

/*Lewis Hamilton Drives for Mercedes-AMG. what else whould i name this? anyway,
this is the code that dictates sharp left-handed corners. also drives forward
ever so slightly to move the sensor away from the previous line on onto the next. 
i should point out Lewis isn't my favorite driver. not even close. I just respect his talent
and dedication. my favorite driver is....
*/
void TurnLeft()
{

    while (eracount < 350) //should turn this thing 90 degrees CCW
    {
        digitalWrite(RDIR, LOW); //low is forwards
        digitalWrite(LDIR, HIGH);
        analogWrite(RPWM, dutyCycle);
        analogWrite(LPWM, dutyCycle);
    }
    eracount = 0;
    while (eracount < 507) // go forwards a specified distance of ~7 inches
    {
        digitalWrite(RDIR, LOW); //low is forwards
        digitalWrite(LDIR, LOW);
        analogWrite(RPWM, dutyCycle);
        analogWrite(LPWM, dutyCycle);
    }
    // analogWrite(RPWM, 0);
    // analogWrite(LPWM, 0);
    // delay(10);
    // digitalWrite(RDIR, LOW); //low is forwards
    // digitalWrite(LDIR, LOW);
    // analogWrite(RPWM, dutyCycle);
    // analogWrite(LPWM, dutyCycle);
    // delay(100);
    // analogWrite(RPWM, 0);
    // analogWrite(LPWM, 0);
}
/*
...Daniel Ricciardo. Dan the man. following in the tradition of the great
Aussie drivers Mark Webber Alan Jones and Sir Jack Brabham, Daniel is a legend. 7x race winner, 
31 podiums, 3x pole sitter, and 2x 3rd place finsihes in the F1 championship, and a smile 
that lights up a room. Also, he drives for McLaren
 */
void TurnRight()
{

    while (eracount < 340) //should turn this thing 90 degrees CW
    {
        digitalWrite(RDIR, HIGH); //low is forwards
        digitalWrite(LDIR, LOW);
        analogWrite(RPWM, dutyCycle);
        analogWrite(LPWM, dutyCycle);
    }
    eracount = 0;
    while (eracount < 507)
    {
        digitalWrite(RDIR, LOW); //low is forwards
        digitalWrite(LDIR, LOW);
        analogWrite(RPWM, dutyCycle);
        analogWrite(LPWM, dutyCycle);
    }
    // analogWrite(RPWM, 0);
    // analogWrite(LPWM, 0);
    // delay(10);
    // digitalWrite(RDIR, LOW); //low is forwards
    // digitalWrite(LDIR, LOW);
    // analogWrite(RPWM, dutyCycle);
    // analogWrite(LPWM, dutyCycle);
    // delay(950);
    // analogWrite(RPWM, 0);
    // analogWrite(LPWM, 0);
}

double distance(int trigPin, int echoPin)
{
    long duration;
    // Write a pulse to the HC-SR04 Trigger Pin
    digitalWrite(trigPin, LOW);
    delayMicroseconds(2);
    digitalWrite(trigPin, HIGH);
    delayMicroseconds(50);
    digitalWrite(trigPin, LOW);

    // Measure the response from the HC-SR04 Echo Pin
    duration = pulseIn(echoPin, HIGH);
    // Determine distance from duration
    // Use 343 metres per second as speed of sound
    return duration * 0.034 / 2;
}