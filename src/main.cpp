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

double armLengths[3] = {115, 80, 130};
double armAngles[3] = {0};


#define increment 0.017 // radians per 15ms


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

void cartesianToSpherical(double coords[3], double lengths[3], double angles[3]) {
    double l0 = lengths[0];
    double l1 = lengths[1];
    double l2 = lengths[2]; // 115, 80, 130
    double x = coords[0];
    double y = coords[1];
    double z = coords[2];

    // Length from base to end effector
    double l = sqrt(pow(x, 2) + pow(y, 2) + pow(z, 2));

    // Base angle
    angles[0] = atan2(z, x);

    double theta2_prime = acos( (l1*l1 + l2*l2 - l*l) / (2*l1*l2) );
    double a = atan2(y, x);
    double b = asin( (sin(theta2_prime)*lengths[2]) / l );
    angles[1] = a + b;
    angles[2] = -(PI - theta2_prime);

    Serial.println(l);
    Serial.println(angles[0]);
    Serial.println(angles[1]);
    Serial.println(angles[2]);
    Serial.println();
}



void goToPosition(double coords[3]) {
        cartesianToSpherical(coords, armLengths, armAngles);


        // print the desired angles
//        Serial.print("Desired Angle1: ");
//        Serial.print(angles[0]);
//        Serial.print(" Desired Angle2: ");
//        Serial.print(angles[1]);
//        Serial.print(" Desired Angle3: ");
//        Serial.println(angles[2]);

        int servoPulseWidth1 = floatMap(armAngles[0], -PI / 2, PI / 2, 500, 2500);
        int servoPulseWidth2 = floatMap(armAngles[1], -PI / 2, PI / 2, 500, 2500);
        int servoPulseWidth3 = floatMap(armAngles[2], -PI / 2, PI / 2, 500, 2500);

        S1.writeMicroseconds(servoPulseWidth1);
        S2.writeMicroseconds(servoPulseWidth2);
        S3.writeMicroseconds(servoPulseWidth3);


}

void setup() {
    timestamp = millis();
    S1.attach(S1_PWM);
    S2.attach(S2_PWM);
    S3.attach(S3_PWM);

    Serial.begin(9600);


}

double coords[3] = {120, 120, 0};
//  Lab 2
void loop() {
    goToPosition(coords);
}


//
//void loop() {
//
////    S1.write(0);
////    S2.write(0);
////    S3.write(0);
//
//}