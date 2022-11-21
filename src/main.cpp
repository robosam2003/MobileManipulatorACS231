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
    double x = coords[0], y = coords[1]-lengths[0], z = coords[2];
    double l = sqrt(x*x + y*y + z*z);
    angles[2] = PI - acos( (l*l - lengths[2]*lengths[2] - lengths[1]*lengths[1]) / (-2*lengths[1]*lengths[2]) );

    double a = atan2(y, x);
    double b = asin( (sin(angles[2])*lengths[2]) / l );
    angles[1] = a + b;
    //angles[0] = atan2(z, x);
}

double lengths[3] = {115, 80, 130};
double angles[3] = {0};


void goToPosition(double coords[3]) {
    elapsedTime = millis() - timestamp;
    if (elapsedTime >= controlRate) {
        cartesianToSpherical(coords, lengths, angles);
        S1.write(angles[0] * RAD_TO_DEG);
        S2.write(angles[1] * RAD_TO_DEG);
        S3.write(angles[2] * RAD_TO_DEG);

//        int pot1 = analogRead(POT_PIN_1);
//        int pot2 = analogRead(POT_PIN_2);
//        int pot3 = analogRead(POT_PIN_3);
//        double desiredAngle1 = floatMap(pot1, 0, 1023, -PI / 2, PI / 2);
//        double desiredAngle2 = floatMap(pot2, 0, 1023, -PI / 2, PI / 2);
//        double desiredAngle3 = floatMap(pot3, 0, 1023, -PI / 2, PI / 2);

        // print the desired angles
        Serial.print("Desired Angle1: ");
        Serial.print(angles[0]);
        Serial.print(" Desired Angle2: ");
        Serial.print(angles[1]);
        Serial.print(" Desired Angle3: ");
        Serial.println(angles[2]);

        // position limiting for safety
        if (angles[1] < -0.9) {
            angles[1] = -0.9;
        }
        double theta3Max = 1.9 * currentAngle2 + 0.79;
        if (angles[2] > theta3Max) {
            angles[2] = theta3Max;
        }

        int servoPulseWidth1 = floatMap(rateLimit(angles[0], currentAngle1), -PI / 2, PI / 2, 500, 2500);
        int servoPulseWidth2 = floatMap(rateLimit(angles[2], currentAngle2), -PI / 2, PI / 2, 500, 2500);
        int servoPulseWidth3 = floatMap(angles[2], -PI / 2, PI / 2, 500, 2500);


        S1.writeMicroseconds(servoPulseWidth1);
        S2.writeMicroseconds(servoPulseWidth2);
        S3.writeMicroseconds(servoPulseWidth3);

    }
}

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

double coords[3] = {120, 120, 0};
//  Lab 2
void loop() {
    goToPosition(coords);
    delay(100);
}


//
//void loop() {
//
////    S1.write(0);
////    S2.write(0);
////    S3.write(0);
//
//}