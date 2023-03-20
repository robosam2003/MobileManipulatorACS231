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

#define LEFT_MTR_I1 6
#define LEFT_MTR_I2 7
#define LEFT_MTR_PWM 4
#define LEFT_MTR_E1 2
#define LEFT_MTR_E2 3

#define RIGHT_MTR_I1 22
#define RIGHT_MTR_I2 24
#define RIGHT_MTR_PWM 8
#define RIGHT_MTR_E1 20
#define RIGHT_MTR_E2 21

#define STBY_PIN 5

long int left_encoder_count = 0;
long int right_encoder_count = 0;


#define SENSOR1 A0
#define SENSOR2 A1
#define SENSOR3 A2 //line sensor pins
#define LINE_SENSED_THRESHOLD 900 //max value output from line sensors which will be considered as detecting a line,
//lower to decrease sensitivity


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

int sensor1Low;
int sensor2Low;
int sensor3Low; //booleans for testing whether the sensor is detecting a line (low) or not (high)
int horizontalLineCount = 0;

Motor LeftMotor(LEFT_MTR_I1, LEFT_MTR_I2, LEFT_MTR_PWM, 1, STBY_PIN); //
Motor RightMotor(RIGHT_MTR_I1, RIGHT_MTR_I2, RIGHT_MTR_PWM, 1, STBY_PIN); //

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
}

void left_e2_ISR() {
    int e1 = digitalRead(LEFT_MTR_E1);
    int e2 = digitalRead(LEFT_MTR_E2);
    if (e1) { // e1 == 1
        (e2) ? left_encoder_count++ : left_encoder_count--;
    } else { // e1 == 0
        (e2) ? left_encoder_count-- : left_encoder_count++;
    }
}

int line_sensor(){
    int sensorReading1 = analogRead(SENSOR1);
    int sensorReading2 = analogRead(SENSOR2);
    int sensorReading3 = analogRead(SENSOR3);

    if (sensorReading1 > LINE_SENSED_THRESHOLD){
        //Serial.println("Sensor 1 reads LOW");
        sensor1Low = 1;
    }
    else{
        //Serial.println("Sensor 1 reads HIGH");
        sensor1Low = 0;
    }

    if (sensorReading2 > LINE_SENSED_THRESHOLD){
        //Serial.println("Sensor 2 reads LOW");
        sensor2Low = 1;
    }
    else{
        //Serial.println("Sensor 2 reads HIGH");
        sensor2Low = 0;
    }

    if (sensorReading3 > LINE_SENSED_THRESHOLD){
        //Serial.println("Sensor 3 reads LOW");
        sensor3Low = 1;
    }
    else{
        //Serial.println("Sensor 3 reads HIGH");
        sensor3Low = 0;
    }

    if (sensor1Low && sensor2Low && sensor3Low){
        Serial.println("Horizontal line found"); // 5
        return 5;
    }
    else if (sensor1Low && sensor2Low){
        Serial.println("Line is slightly to the right"); // 1
        return 1;
    }
    else if (sensor2Low && sensor3Low){
        Serial.println("Line is slightly to the left"); // -1
        return -1;
    }
    else if (sensor1Low){
        Serial.println("Line is to the right"); // 2
        return 2;
    }
    else if (sensor2Low){
        Serial.println("Line is in the middle"); // 0
        return 0;
    }
    else if (sensor3Low){
        Serial.println("Line is to the left"); // -2
        return -2;
    }
    else{
        Serial.println("Line cannot be seen"); // -5
        return -5;
    }
}



void right_e1_ISR() {
    int e1 = digitalRead(RIGHT_MTR_E1);
    int e2 = digitalRead(RIGHT_MTR_E2);
    if (e2) { // e2 == 1
        (e1) ? right_encoder_count-- : right_encoder_count++;
    } else { // e2 == 0
        (e1) ? right_encoder_count++ : right_encoder_count--;
    }
};

void right_e2_ISR() {
    int e1 = digitalRead(RIGHT_MTR_E1);
    int e2 = digitalRead(RIGHT_MTR_E2);
    if (e1) { // e1 == 1
        (e2) ? right_encoder_count++ : right_encoder_count--;
    } else { // e1 == 0
        (e2) ? right_encoder_count-- : right_encoder_count++;
    }
};

void setup() {
    timestamp = millis();
    S1.attach(S1_PWM);
    S2.attach(S2_PWM);
    S3.attach(S3_PWM);

    Serial.begin(9600);

    // Left motor encoders
    pinMode(LEFT_MTR_E1, INPUT);
    pinMode(LEFT_MTR_E2, INPUT);

    pinMode(SENSOR1, INPUT);
    pinMode(SENSOR2, INPUT);
    pinMode(SENSOR3, INPUT);

    attachInterrupt(digitalPinToInterrupt(LEFT_MTR_E1), left_e1_ISR, CHANGE);
    attachInterrupt(digitalPinToInterrupt(LEFT_MTR_E2), left_e2_ISR, CHANGE);

    // Right motor encoders
    pinMode(RIGHT_MTR_E1, INPUT);
    pinMode(RIGHT_MTR_E2, INPUT);

    attachInterrupt(digitalPinToInterrupt(RIGHT_MTR_E1), right_e1_ISR, CHANGE);
    attachInterrupt(digitalPinToInterrupt(RIGHT_MTR_E2), right_e2_ISR, CHANGE);


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

void robot_forward(int time) {
    // Closed loop control
    int start = millis();
    // Reset encoder counts
    left_encoder_count = 0;
    right_encoder_count = 0;

    float left_speed = 64;
    float right_speed = 64;

    double integral = 0;

    double P = 0.001;
    double I = 0;


    // Set motor speeds
    while(true) {

        double error = ((double) (left_encoder_count - right_encoder_count) ) / 10;
        integral += error;
        right_speed += P * error + I * integral;
        left_speed -= P * error + I * integral;
//        if (left_encoder_count > right_encoder_count) {
//            right_speed += P * error + I * integral;
//            left_speed -= P * error + I * integral;
//        }
//        else if (left_encoder_count < right_encoder_count) {
//            left_speed += P * error + I * integral;
//            right_speed -= P * error + I * integral;
//        }
//        delay(10);
        // print encoders and speeds

        Serial.println(String(error));
        LeftMotor.drive((int) left_speed);
        RightMotor.drive((int) right_speed);
//        Serial.print(error);
    }


/*    int mtr_increment = 5;
    long int start = millis();
    // using the encoders to drive the robot forward, the robot needs to drive each motor at the same speed.
    while(millis()-start < time) { // would be line sensor interrupt or so
        long int old_left_encoder_count = left_encoder_count;
        long int old_right_encoder_count = right_encoder_count;

        while (left_encoder_count < old_left_encoder_count + mtr_increment) {
            Serial.println(String(left_encoder_count) + " " + String(right_encoder_count));
            LeftMotor.drive(255);
        }
        LeftMotor.drive(0);
//        LeftMotor.brake();
        while (right_encoder_count < old_right_encoder_count + mtr_increment) {
            Serial.println(String(left_encoder_count) + " " + String(right_encoder_count));
            RightMotor.drive(255);
        }
        RightMotor.drive(0);
//        RightMotor.brake();

    }
    LeftMotor.brake();
    RightMotor.brake();*/

}

//
//void robot_backwards(int time) {
//    int mtr_increment = 10;
//    long int start = millis();
//    // using the encoders to drive the robot forward, the robot needs to drive each motor at the same speed.
//    while(millis()-start < time) { // would be line sensor interrupt or so
//        long int old_left_encoder_count = left_encoder_count;
//        long int old_right_encoder_count = right_encoder_count;
//
//        while (left_encoder_count > old_left_encoder_count - mtr_increment) {
//            Serial.println(String(left_encoder_count) + " " + String(right_encoder_count));
//            LeftMotor.drive(-255);
//        }
////        LeftMotor.brake();
//        while (right_encoder_count > old_right_encoder_count - mtr_increment) {
//            Serial.println(String(left_encoder_count) + " " + String(right_encoder_count));
//            RightMotor.drive(-153);
//        }
////        RightMotor.brake();
//
//    }
//    LeftMotor.brake();
        while (right_encoder_count < old_right_encoder_count + mtr_increment) {
            Serial.println(String(left_encoder_count) + " " + String(right_encoder_count));
            RightMotor.drive(-153);
        }
//    RightMotor.brake();
//}

void robot_backwards(int time) {
    // Closed loop control
    int start = millis();
    // Reset encoder counts
    left_encoder_count = 0;
    right_encoder_count = 0;

    float left_speed = 128;
    float right_speed = 128;

    double integral = 0;
    double P = 0.001;
    double I = 0;

    // Set motor speeds
    while (millis() - start < time) {
        LeftMotor.drive((int) -left_speed);
        RightMotor.drive((int) -right_speed);
        double error = ((double) (left_encoder_count - right_encoder_count)) / 10;
        integral += error;
        right_speed += P * error + I * integral;
        left_speed -= P * error + I * integral;
//        if (left_encoder_count > right_encoder_count) {
//            right_speed += P * error + I * integral;
//            left_speed -= P * error + I * integral;
//        }
//        else if (left_encoder_count < right_encoder_count) {
//            left_speed += P * error + I * integral;
//            right_speed -= P * error + I * integral;
//        }
//        delay(10);
        // print encoders and speeds

        Serial.println(String(left_speed) + ", " + String(right_speed));
//        Serial.print(error);
    }
}

void robot_spin_cw(int degrees) {
    int rotation = degrees * 3840 / 360;
    int some_other_condition = 1; // related to the line sensors
    while ((left_encoder_count < rotation && right_encoder_count < rotation) || (some_other_condition) ) {
        LeftMotor.drive(255);
        RightMotor.drive(-153);
    }
}

void robot_spin_ccw(int degrees) {
    int rotation = degrees * 3840 / 360;
    int some_other_condition = 1; // related to the line sensors
    while ((left_encoder_count < rotation && right_encoder_count < rotation) || (some_other_condition) ) {
        LeftMotor.drive(-255);
        RightMotor.drive(153);
    }
}

void robot_stop(int del) {
    LeftMotor.brake();
    RightMotor.brake();
    delay(del);
}

void move_according_to_line_sensor(int line_pos){ // line_pos value got from line_sensor() function
    switch(line_pos){
        case -2:
            //turn a lot left
            robot_spin_ccw(25);
            robot_forward(200);
            break;
        case -1:
            //turn a bit left
            robot_spin_ccw(10);
            robot_forward(200);
            break;
        case 1:
            //turn a bit right
            robot_spin_cw(10);
            robot_forward(200);
            break;
        case 2:
            //turn a lot right
            robot_spin_cw(25);
            robot_forward(200);
            break;
        case -5:
            //find the line
            robot_forward(100);//until case 5
            break;
        case 5:
            //start/stop/right angle
            horizontalLineCount++;
            //first 5 should turn to the right, then go straight
            if (horizontalLineCount == 1){
                robot_spin_ccw(90);
                robot_forward(500);
            }
            //second 5 should turn to the left, then go straight, then turn left again, then go straight
            if (horizontalLineCount == 2){
                robot_stop();
                robot_spin_ccw(90);
                robot_forward(1000);
                robot_stop();
                robot_spin_ccw(90);
                robot_forward(500);
            }
            //third 5 should turn to the left, then go straight
            if (horizontalLineCount == 3){
                robot_spin_ccw(90);
                robot_forward(100);
            }
            //fourth 5 should stop
            if (horizontalLineCount == 4){
                robot_stop();
            }
            break;
        default:
            //go straight
            robot_forward(200);
            break;
    }
}

int i = 0;
void loop() {
    robot_forward(120000);
//    robot_stop(1000);
//    robot_backwards(1000);
//    robot_stop(1000);
//    robot_stop(1000);
//    robot_backwards(1000);
//    robot_stop(1000);
//    robot_spin_cw(90);




//    long left_start = millis();
//    while((left_encoder_count/3840) < i) {
//        LeftMotor.drive(255);
//        Serial.println((double)left_encoder_count/3840);
//    }
//    LeftMotor.brake();
//    Serial.println("Left motor took: " + String(millis() - left_start) + "ms");
//    delay(1000);
//
//    long right_start = millis();
//    while((right_encoder_count/3840) < i) {
//        RightMotor.drive(152);
//        Serial.println((double)right_encoder_count/3840);
//    }
//    RightMotor.brake();
//    Serial.println("Right motor took: " + String(millis() - right_start) + "ms");
//    delay(1000);
//    i++;



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






