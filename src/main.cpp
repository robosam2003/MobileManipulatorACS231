#include <Arduino.h>
#include <Servo.h>

// Pi to 20 digits
#define PI 3.14159265358979323846


Servo S1;
Servo S2;
Servo S3;

#define S1_PWM 9
#define S2_PWM 10
#define S3_PWM 11

#define POT_PIN_1 A0
#define POT_PIN_2 A1
#define POT_PIN_3 A2

unsigned long timestamp;
unsigned long elapsedTime;
unsigned long controlRate = 15; // ms

double currentAngle1 = 0;
double currentAngle2 = 0;
double currentAngle3 = 0;

#define increment 0.017 // radians per 15ms

void setup() {
    timestamp = millis();
    S1.attach(S1_PWM);
    S2.attach(S2_PWM);
    S3.attach(S3_PWM);

    Serial.begin(9600);

    pinMode(POT_PIN_1, INPUT);
    pinMode(POT_PIN_2, INPUT);
    pinMode(POT_PIN_3, INPUT);


}

double floatMap(double x, double in_min, double in_max, double out_min, double out_max) {
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

double rateLimit(double desiredAngle, double &currentAngle) {
    double positionCommand = 0;
    if (desiredAngle > currentAngle) {
        positionCommand = min(desiredAngle, currentAngle + increment);
        currentAngle = positionCommand;
    }
    else if (desiredAngle < currentAngle) {
        positionCommand = max(desiredAngle, currentAngle - increment);
        currentAngle = positionCommand;
    }
    else {
        positionCommand = desiredAngle;
        currentAngle = positionCommand;
    }
    return positionCommand;
}


void loop() {
    elapsedTime = millis() - timestamp;
    if (elapsedTime >= controlRate) {
        int pot1 = analogRead(POT_PIN_1);
        int pot2 = analogRead(POT_PIN_2);
        int pot3 = analogRead(POT_PIN_3);


        double desiredAngle1 = floatMap(pot1, 0, 1023, -PI / 2, PI / 2);
        double desiredAngle2 = floatMap(pot2, 0, 1023, -PI / 2, PI / 2);
        double desiredAngle3 = floatMap(pot3, 0, 1023, -PI / 2, PI / 2);

        // print the desired angles
        Serial.print("Desired Angle1: ");
        Serial.print(desiredAngle1);
        Serial.print(" Desired Angle2: ");
        Serial.print(desiredAngle2);
        Serial.print(" Desired Angle3: ");
        Serial.println(desiredAngle3);

        // position limiting for safety
        if (desiredAngle2 < -0.9) {
            desiredAngle2 = -0.9;
        }
        double theta3Max = 1.9 * currentAngle2 + 0.79;
        if (desiredAngle3 > theta3Max) {
            desiredAngle3 = theta3Max;
        }

        int servoPulseWidth1 = floatMap(rateLimit(desiredAngle1, currentAngle1), -PI / 2, PI / 2, 500, 2500);
        int servoPulseWidth2 = floatMap(rateLimit(desiredAngle2, currentAngle2), -PI / 2, PI / 2, 500, 2500);
        int servoPulseWidth3 = floatMap(desiredAngle3, -PI / 2, PI / 2, 500, 2500);


        S1.writeMicroseconds(servoPulseWidth1);
        S2.writeMicroseconds(servoPulseWidth2);
        S3.writeMicroseconds(servoPulseWidth3);



    }

}