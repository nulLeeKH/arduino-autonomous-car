/*
version : v1.0.0-alpha

MIT License

Copyright (c) 2019 Kyung-ha Lee <i_am@nulleekh.com>

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
 */

#include <SoftwareSerial.h>
#include <Servo.h>

Servo servo;

#define W_LEFT 0
#define W_RIGHT 1
#define D_FRONT 0
#define D_BACK 1
//Macros to command a function

#define M1_PWM 5
#define M1_DIR1 7
#define M1_DIR2 8
//Motor 1 connection (left)

#define M2_PWM 6
#define M2_DIR1 11
#define M2_DIR2 12
//Motor 2 connection (right)

#define SERVO_PIN 9         //Steering (Surbo Motor Connection Pin)
#define TD_LEFT  0          //Left turn angle [-90 to 90]
#define TD_RIGHT  0         //Right turn angle [-90 to 90]
#define GD_LEFT 0           //Correction angle of the mail direction during central driving [-90 to 90]
#define GD_RIGHT 0          //Angle of correction of left side shape during central driving [-90 to 90]
#define TD_MIDDLE 0         //Straight Angle [-90-90]
#define S_HIGH 0            //Outer wheel speed when driving [0-255]
#define S_SLOW 0            //Inner wheel speed when driving [0-255]
#define S_SLOWER 0          //Deceleration at low speed [0-255]
#define S_BREAK 0           //Brake strength [0-255]
#define T_BREAK 0           //Brake duration

#define FC_TRIG 13          //Front center sensor (TRIG pin)
#define FC_ECHO 10          //Front centre sensor (ECHO pin)
#define FL_TRIG A4          //Front left sensor (TRIG pin)
#define FL_ECHO A3          //Front left sensor (ECHO pin)
#define FR_TRIG 3           //Front left sensor (ECHO pin)
#define FR_ECHO 4           //Front left sensor (ECHO pin)
#define L_TRIG  A2          //Left Sensor (TRIG pin)
#define L_ECHO  A1          //Left Sensor (ECHO pin)
#define R_TRIG  A0          //Right sensor (TRIG pin)
#define R_ECHO  A5          //Right sensor (ECHO pin)

#define FC_WARNING 0        //Front Central Turning Distance
#define FR_WARNING 0        //Front right turn distance
#define FL_WARNING 0        //Front left turn distance
#define FC_PREPARE 0        //Front Central Slow Drive

#define T_BACK 0            //Reverse distance when turning
#define T_FRONT 0           //Direct distance when turning

#define INTIAL_STRAIGHT 0   //Direct distance during initial execution
#define INTIAL_DEGREE 0     //Turn direction for initial execution (left : 0; right : 1)
#define INTIAL_TURN 0       //Rotation angle at initial execution
#define T_TURN 0            //Body angle rotation duration
#define T_START 0           //Right turn at initial execution
//Settings for driving

class Driving{
        public:

        float GetDistance(int trig, int echo) {
            digitalWrite(trig, LOW);
            delayMicroseconds(4);
            digitalWrite(trig, HIGH);
            delayMicroseconds(20);
            digitalWrite(trig, LOW);
            unsigned long duration = pulseIn(echo, HIGH, 5000);
            return duration * 0.17;
        }   //Distance calculation

        void TurnSteering(int angle) {
            servo.write(angle + 90 - 3);
        }   //Adjust steering rotation angle

        void WheelGo(int lr, int fb, int num) {
            if (lr == W_LEFT) {
                if (fb == D_FRONT) {
                    digitalWrite(M1_DIR1, HIGH);
                    digitalWrite(M1_DIR2, LOW);
                    analogWrite(M1_PWM, num);
                } else {
                    digitalWrite(M1_DIR1, LOW);
                    digitalWrite(M1_DIR2, HIGH);
                    analogWrite(M1_PWM, num);
                }
            } else {
                if (fb == D_FRONT) {
                    digitalWrite(M2_DIR1, HIGH);
                    digitalWrite(M2_DIR2, LOW);
                    analogWrite(M2_PWM, num);
                } else {
                    digitalWrite(M2_DIR1, LOW);
                    digitalWrite(M2_DIR2, HIGH);
                    analogWrite(M2_PWM, num);
                }
            }
        }   //Lateral wheel rotation control

        void WheelStop() {
            WheelGo(W_LEFT, D_BACK, S_BREAK);
            WheelGo(W_RIGHT, D_BACK, S_BREAK);
            delay(T_BREAK);
            //Reverse propulsion
            analogWrite(M1_PWM, 0);
            analogWrite(M2_PWM, 0);
            //Wheel stop
        }   //break

        void Straight_f(float fcdis, float rdis, float ldis) {
            if (ldis == 0) {
                TurnSteering(GD_LEFT);
                WheelGo(W_LEFT, D_FRONT, S_SLOW);
                WheelGo(W_RIGHT, D_FRONT, S_HIGH);
            } else if (rdis == 0) {
                TurnSteering(GD_RIGHT);
                WheelGo(W_LEFT, D_FRONT, S_HIGH);
                WheelGo(W_RIGHT, D_FRONT, S_SLOW);
            } else if (rdis < ldis) {
                TurnSteering(GD_LEFT);
                WheelGo(W_LEFT, D_FRONT, S_SLOW);
                WheelGo(W_RIGHT, D_FRONT, S_HIGH);
            } else {
                TurnSteering(GD_RIGHT);
                WheelGo(W_LEFT, D_FRONT, S_HIGH);
                WheelGo(W_RIGHT, D_FRONT, S_SLOW);
            }
        }   //Centralized by a straight line - smooth driving

        void Straight_s(float fcdis, float rdis, float ldis) {
            if (ldis == 0) {
                TurnSteering(GD_LEFT);
                WheelGo(W_LEFT, D_FRONT, S_SLOW - S_SLOWER);
                WheelGo(W_RIGHT, D_FRONT, S_HIGH - S_SLOWER);
            } else if (rdis == 0) {
                TurnSteering(GD_RIGHT);
                WheelGo(W_LEFT, D_FRONT, S_HIGH - S_SLOWER);
                WheelGo(W_RIGHT, D_FRONT, S_SLOW - S_SLOWER);
            } else if (rdis < ldis) {
                TurnSteering(GD_LEFT);
                WheelGo(W_LEFT, D_FRONT, S_SLOW - S_SLOWER);
                WheelGo(W_RIGHT, D_FRONT, S_HIGH - S_SLOWER);
            } else {
                TurnSteering(GD_RIGHT);
                WheelGo(W_LEFT, D_FRONT, S_HIGH - S_SLOWER);
                WheelGo(W_RIGHT, D_FRONT, S_SLOW - S_SLOWER);
            }
        }   //Centralized by straight ahead - Slow down when approaching obstacles

        void TurnLeft() {
            TurnSteering(TD_RIGHT);
            WheelGo(W_LEFT, D_BACK, S_HIGH);
            WheelGo(W_RIGHT, D_BACK, S_HIGH);
            delay(T_BACK);
            TurnSteering(TD_LEFT);
            WheelGo(W_LEFT, D_FRONT, S_SLOW);
            WheelGo(W_RIGHT, D_FRONT, S_HIGH);
            delay(T_FRONT);
            TurnSteering(TD_MIDDLE);
        }   //Turn to the left

        void TurnRight() {
            TurnSteering(TD_LEFT);
            WheelGo(W_LEFT, D_BACK, S_HIGH);
            WheelGo(W_RIGHT, D_BACK, S_HIGH);
            delay(T_BACK);
            TurnSteering(TD_RIGHT);
            WheelGo(W_LEFT, D_FRONT, S_HIGH);
            WheelGo(W_RIGHT, D_FRONT, S_SLOW);
            delay(T_FRONT);
            TurnSteering(TD_MIDDLE);
        }   //Turn to the right

        void TurnBody(int tt) {
            WheelStop();
            if (INTIAL_DEGREE == 0) {
                for (int i = 0; i < tt; i++) {
                    TurnSteering(TD_RIGHT);
                    WheelGo(W_LEFT, D_BACK, S_HIGH);
                    WheelGo(W_RIGHT, D_BACK, S_HIGH);
                    delay(T_BACK);
                    TurnSteering(TD_LEFT);
                    WheelGo(W_LEFT, D_FRONT, S_SLOW);
                    WheelGo(W_RIGHT, D_FRONT, S_HIGH);
                    delay(T_FRONT);
                }
            } else {
                for (int i = 0; i < tt; i++) {
                    TurnSteering(TD_LEFT);
                    WheelGo(W_LEFT, D_BACK, S_HIGH);
                    WheelGo(W_RIGHT, D_BACK, S_HIGH);
                    delay(T_TURN);
                    TurnSteering(TD_RIGHT);
                    WheelGo(W_LEFT, D_FRONT, S_HIGH);
                    WheelGo(W_RIGHT, D_FRONT, S_SLOW);
                    delay(T_TURN);
                }
            }
            TurnSteering(TD_MIDDLE);
        }   //Body rotation
};

void setup() {
    pinMode(M1_PWM, OUTPUT);
    pinMode(M1_DIR1, OUTPUT);
    pinMode(M1_DIR2, OUTPUT);

    pinMode(M2_PWM, OUTPUT);
    pinMode(M2_DIR1, OUTPUT);
    pinMode(M2_DIR2, OUTPUT);

    pinMode(FC_TRIG, OUTPUT);
    pinMode(FC_ECHO, INPUT);
    pinMode(FR_TRIG, OUTPUT);
    pinMode(FR_ECHO, INPUT);
    pinMode(FL_TRIG, OUTPUT);
    pinMode(FL_ECHO, INPUT);
    pinMode(R_TRIG, OUTPUT);
    pinMode(R_ECHO, INPUT);
    pinMode(L_TRIG, OUTPUT);
    pinMode(L_ECHO, INPUT);

    Serial.begin(9600);

    servo.attach(SERVO_PIN);

    servo.write(TD_MIDDLE);

    Driving driving;

    for (int i = 0; i < INTIAL_STRAIGHT; i++) {
        float fcdis = driving.GetDistance(FC_TRIG, FC_ECHO);
        float rdis = driving.GetDistance(R_TRIG, R_ECHO);
        float ldis = driving.GetDistance(L_TRIG, L_ECHO);
        driving.Straight_f(fcdis, rdis, ldis);
        delay(3);
    }

    driving.TurnBody(INTIAL_TURN);

    driving.TurnSteering(TD_LEFT);

    driving.WheelGo(W_LEFT, D_FRONT, S_SLOW);
    driving.WheelGo(W_RIGHT, D_FRONT, S_HIGH);

    delay(T_START);
}   //Initial routine

void loop() {
    Driving driving;

    float fcdis = driving.GetDistance(FC_TRIG, FC_ECHO);
    float frdis = driving.GetDistance(FR_TRIG, FR_ECHO);
    float fldis = driving.GetDistance(FL_TRIG, FL_ECHO);
    float rdis = driving.GetDistance(R_TRIG, R_ECHO);
    float ldis = driving.GetDistance(L_TRIG, L_ECHO);

    if ((0 < fcdis && fcdis < FC_WARNING) || (0 < frdis && frdis < FR_WARNING) || (0 < fldis && fldis < FL_WARNING)) {
        if (fldis == 0) driving.TurnLeft();
        else if (frdis == 0) driving.TurnRight();
        else if (frdis < fldis)
            driving.TurnLeft();
        else driving.TurnRight();
    }
    else if (0 < fcdis && fcdis < FC_PREPARE) driving.Straight_s(fcdis, rdis, ldis);
    else driving.Straight_f(fcdis, rdis, ldis);

    delay(3);
}   //Main routine
