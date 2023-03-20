#include <Arduino.h>
#include <Servo.h>
#include "SparkFun_TB6612.h"
// Pi to 20 digits
#define PI 3.14159265358979323846

// Servos
Servo S1;
Servo S2;
Servo S3;

#define S1_PWM 9
#define S2_PWM 10
#define S3_PWM 11

#define LEFT_MTR_I1 1
#define LEFT_MTR_I2 0
#define LEFT_MTR_PWM 4
#define LEFT_MTR_E1 2
#define LEFT_MTR_E2 3

#define RIGHT_MTR_I1
#define RIGHT_MTR_I2
#define RIGHT_MTR_PWM
#define RIGHT_MTR_E1 20
#define RIGHT_MTR_E2 21

#define STBY_PIN 5

long int left_encoder_count = 0;

//#define POT_PIN_1 A0
//#define POT_PIN_2 A1
//#define POT_PIN_3 A2

unsigned long timestamp;
unsigned long elapsedTime;
unsigned long controlRate = 50; // ms

double currentAngle1 = 0;
double currentAngle2 = 0;
double currentAngle3 = 0;

double armLengths[3] = {65.5, 120, 160};
double armAngles[3] = {0};

Motor LeftMotor(LEFT_MTR_I1, LEFT_MTR_I2, LEFT_MTR_PWM, 1, STBY_PIN); //


#define increment 0.017 // radians per 15ms


double floatMap(double x, double in_min, double in_max, double out_min, double out_max) {
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

void cartesianToSpherical(double coords[3], double lengths[3], double angles[3]) {

    // had to remap
    // -> means replaces
    // y -> x,  z -> y,  x -> z
    double l0 = lengths[0];
    double l1 = lengths[1];
    double l2 = lengths[2];
    double x = -coords[0];
    double y = coords[1];
    double z = coords[2]-l0;

    // Length from base to end effector
    double l = sqrt( pow(x, 2) + pow(y, 2) + pow(z, 2) );
    if (l > l1 + l2) {
        // out of reach
        Serial.println("Out of reach");
        return;
    }
    // Base angle
    angles[0] = atan2(x, y);

    double theta2_prime = acos( (l1*l1 + l2*l2 - l*l) / (2*l1*l2) );
    //Serial.println(theta2_prime);
    double a = atan2(z, y);
    //Serial.println(a);
    double b = asin( (sin(theta2_prime)*lengths[2]) / l );
    //Serial.println(b);
    angles[1] = a + b;
    angles[2] = -(PI - theta2_prime);

//    Serial.println(l);
//    Serial.println(angles[0]);
//    Serial.println(angles[1]);
//    Serial.println(angles[2]);
    // forward kinematics
//    Serial.print("FORWARD KINEMATICS:  ");
//    Serial.print(l1*cos(angles[1])+ l2*cos(angles[1] + angles[2]));
//    Serial.print(",   ");
//    Serial.print(l0+l1*sin(angles[1])+ l2*sin(angles[1] + angles[2]));
//    Serial.println("\n");

}


void goToPosition(double coords[3]) {
    cartesianToSpherical(coords, armLengths, armAngles);
//    double tolerance = 0.001;
//    while(abs(armAngles[0] - currentAngle1) > tolerance || abs(armAngles[1] - currentAngle2) > tolerance || abs(armAngles[2] - currentAngle3) > tolerance) {

//        int servoPulseWidth1 = floatMap(rateLimit(armAngles[0], currentAngle1), -PI / 2, PI / 2, 500, 2500);
//        int servoPulseWidth2 = floatMap(rateLimit(armAngles[1], currentAngle2), -PI / 2, PI / 2, 500, 2500);
//        int servoPulseWidth3 = floatMap(rateLimit(armAngles[2], currentAngle3), PI / 2, -PI / 2, 500, 2500);
        int servoPulseWidth1 = floatMap(armAngles[0], -PI / 2, PI / 2, 500, 2500);
        int servoPulseWidth2 = floatMap(armAngles[1]-PI/2, -PI / 2, PI / 2, 500, 2500);
        int servoPulseWidth3 = floatMap(armAngles[2]+PI/2, -PI / 2, PI / 2, 500, 2500);

        S1.writeMicroseconds(servoPulseWidth1);
        S2.writeMicroseconds(servoPulseWidth2);
        S3.writeMicroseconds(servoPulseWidth3);

        //delay(controlRate);
        //Serial.println("ONE LOOP");
    //}
}

void home() {
    double homePosition[3] = {0, 210, 115};
    goToPosition(homePosition);
}

void drawStraightLine(double initialPosition[], double endPosition[]) {
    goToPosition(initialPosition);
    // Vector between two points
    double v[3] = {endPosition[0] - initialPosition[0],
                   endPosition[1] - initialPosition[1],
                   endPosition[2] - initialPosition[2]};
    // normalise the vector
    double magnitude_v = sqrt(pow(v[0], 2) + pow(v[1], 2) + pow(v[2], 2));
    double v_hat[3] = {v[0]/magnitude_v, v[1]/magnitude_v, v[2]/magnitude_v};

    // step size
    double stepSize = 0.1;
    // number of steps
    int n = magnitude_v / stepSize;

    for (int i = 0; i < n; i++) {
        // Step along the line.
        double position[3] = {initialPosition[0] + i*stepSize*v_hat[0],
                              initialPosition[1] + i*stepSize*v_hat[1],
                              initialPosition[2] + i*stepSize*v_hat[2]};
        goToPosition(position);
        delay(stepSize); // or some scalar.
    }

}

void left_e1_ISR() {
    int e1 = digitalRead(LEFT_MTR_E1);
    int e2 = digitalRead(LEFT_MTR_E2);
    if (e2) { // e2 == 1
        (e1) ? left_encoder_count-- : left_encoder_count++;
    } else { // e2 == 0
        (e1) ? left_encoder_count++ : left_encoder_count--;
    }
};

void left_e2_ISR() {
    int e1 = digitalRead(LEFT_MTR_E1);
    int e2 = digitalRead(LEFT_MTR_E2);
    if (e1) { // e1 == 1
        (e2) ? left_encoder_count++ : left_encoder_count--;
    } else { // e1 == 0
        (e2) ? left_encoder_count-- : left_encoder_count++;
    }
};

void setup() {
    timestamp = millis();
    S1.attach(S1_PWM);
    S2.attach(S2_PWM);
    S3.attach(S3_PWM);

    Serial.begin(9600);

    pinMode(LEFT_MTR_E1, INPUT);
    pinMode(LEFT_MTR_E2, INPUT);

    attachInterrupt(digitalPinToInterrupt(LEFT_MTR_E1), left_e1_ISR, CHANGE);
    attachInterrupt(digitalPinToInterrupt(LEFT_MTR_E2), left_e2_ISR, CHANGE);


}

double positions[3][3] = {{0, 90,  250},
                          {0, 180, 250},
                          {0, 112, 70}};

void Assignment_4_robot_arm() {
    goToPosition(positions[0]);
    delay(1000);
    drawStraightLine(positions[0], positions[1]);
    drawStraightLine(positions[1], positions[2]);
    drawStraightLine(positions[2], positions[0]);
//    drawStraightLine(positions[0], positions[3]);
//    drawStraightLine(positions[3], positions[4]);
//    drawStraightLine(positions[4], positions[5]);
//    drawStraightLine(positions[5], positions[0]);
    while (1);
    Serial.println("---------------------------------\n\n\n");


}
int i = 0;
void loop() {
    while(abs(left_encoder_count/3840) < i) {
        LeftMotor.drive(-255);
        Serial.println((double)left_encoder_count/3840);
    }
    LeftMotor.brake();
    delay(1000);
    i++;
//    LeftMotor.drive(255);
//    for (int i = 0; i < 100; i++) {
//        Serial.println(left_e1_count);
//        Serial.println(left_e2_count);
//        delay(10);
//    }
//    LeftMotor.brake();
//    delay(1000);
//    LeftMotor.drive(-255);
//    for (int i = 0; i < 100; i++) {
//        Serial.println(left_e1_count);
//        Serial.println(left_e2_count);
//        delay(10);
//    }
//    LeftMotor.brake();
//    delay(1000);
//    delay(100);
//    LeftMotor.drive(-128, 1000);
//    delay(100);
//    digitalWrite(LED_BUILTIN, HIGH);
//    delay(1000);
//    digitalWrite(LED_BUILTIN, LOW);


}




//void loop() {
//    // Homing sequence
//    S1.write(90);
//    S2.write(90);
//    S3.write(180);
//
//}






