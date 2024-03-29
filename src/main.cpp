#include <Arduino.h>
#include <Servo.h>
#include "SparkFun_TB6612.h"
// Pi to 20 digits#define PI 3.14159265358979323846

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

double VBase = 3000.0; // encoder counts per second



#define SENSOR1 A0
#define SENSOR2 A1
#define SENSOR3 A2 //line sensor pins
#define LINE_SENSED_THRESHOLD 900 //max value output from line sensors which will be considered as detecting a line,
//lower to decrease sensitivity

#define WHEEL_DIAMETER_M 0.064 //meters - ACCURATE
#define WHEEL_ROBOT_RADIUS_M 0.094 //meters - ACCURATE

#define COUNTS_PER_REVOLUTION 3850.0 //encoder counts per revolution

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

#define increment 0.1 // radians per 15ms

// Robot arm functions -----------------------------------------------------------------------------------------------
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

    Serial.println(l);
    Serial.println(angles[0]);
    Serial.println(angles[1]);
    Serial.println(angles[2]);
//     forward kinematics
    Serial.print("FORWARD KINEMATICS:  ");
    Serial.print(l1*cos(angles[1])+ l2*cos(angles[1] + angles[2]));
    Serial.print(",   ");
    Serial.print(l0+l1*sin(angles[1])+ l2*sin(angles[1] + angles[2]));
    Serial.println("\n");
//    void fold_arm() {
//        S1.write(80);
//        S2.write(175);
//        S3.write(0);
//    }

}

void servosToAngles(double angles[3]) {
        //    double tolerance = 0.001;
//    while(abs(armAngles[0] - currentAngle1) > tolerance || abs(armAngles[1] - currentAngle2) > tolerance || abs(armAngles[2] - currentAngle3) > tolerance) {

//        int servoPulseWidth1 = floatMap(rateLimit(armAngles[0], currentAngle1), -PI / 2, PI / 2, 500, 2500);
//        int servoPulseWidth2 = floatMap(rateLimit(armAngles[1], currentAngle2), -PI / 2, PI / 2, 500, 2500);
//        int servoPulseWidth3 = floatMap(rateLimit(armAngles[2], currentAngle3), PI / 2, -PI / 2, 500, 2500);
        int servoPulseWidth1 = floatMap(angles[0]-0.15, -PI / 2, PI / 2, 500, 2500);
        int servoPulseWidth2 = floatMap(angles[1]-PI/2, -PI / 2, PI / 2, 500, 2500);
        int servoPulseWidth3 = floatMap(angles[2]+PI/2, -PI / 2, PI / 2, 500, 2500);

        S1.writeMicroseconds(servoPulseWidth1);
        S2.writeMicroseconds(servoPulseWidth2);
        S3.writeMicroseconds(servoPulseWidth3);
}

void goToPosition(double coords[3]) {
    cartesianToSpherical(coords, armLengths, armAngles);
    servosToAngles(armAngles);
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
    double stepSize = 1;
    // number of steps
    int n = magnitude_v / stepSize;

    for (int i = 0; i < n; i++) {
        // Step along the line.
        double position[3] = {initialPosition[0] + i*stepSize*v_hat[0],
                              initialPosition[1] + i*stepSize*v_hat[1],
                              initialPosition[2] + i*stepSize*v_hat[2]};
        goToPosition(position);
        delay((static_cast<int>(stepSize)));
    }

}

double positions[3][3] = {{0, 90,  250},
                          {0, 180, 250},
                          {0, 112, 70}};
// arm lengths - {65.5, 120, 160}

double foldedPosition[3] = {0, 0, 125};






void fold_arm() {
    // forward kinematics on these angles to get the position
    goToPosition(foldedPosition);
    delay(500);

    // then move to that position.
    S1.write(80);
    delay(500);
    S3.write(0);
    delay(500);
    S2.write(175);
    delay(500);


}


double positions2[6][3] = {{0, 90,  250}, // away
                           {0, 200, 250}, // top
                           {0, 130, 60}, //bottom
                            {100, 160, 120}, //right
                            {0, 150, 120}, // middle
                            {-100, 175, 120}}; //left

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
//    while (1);
    Serial.println("---------------------------------\n\n\n");


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

void foldedToDrawing() {

    double foldedAngles[3] = {80, 175, 0};
    currentAngle1 = foldedAngles[0];
    currentAngle2 = foldedAngles[1];
    currentAngle3 = foldedAngles[2];
    double drawingAngles[3];

    cartesianToSpherical(positions2[0], armLengths, drawingAngles);
    double tolerance = 0.001;

    // rate limit each individually
    while(abs(drawingAngles[0] - currentAngle1) > tolerance) {
        int servoPulseWidth1 = floatMap(rateLimit(drawingAngles[0], currentAngle1)-0.15, -PI / 2, PI / 2, 500, 2500);
        S1.writeMicroseconds(servoPulseWidth1);
        delay(15);
    }
    while(abs(drawingAngles[1] - currentAngle2) > tolerance) {
        int servoPulseWidth2 = floatMap(rateLimit(drawingAngles[1], currentAngle2)-PI/2, -PI / 2, PI / 2, 500, 2500);
        S2.writeMicroseconds(servoPulseWidth2);
        delay(15);
    }
    while(abs(drawingAngles[2] - currentAngle3) > tolerance) {
        int servoPulseWidth3 = floatMap(rateLimit(drawingAngles[2], currentAngle3)+PI/2, PI / 2, -PI / 2, 500, 2500);
        S3.writeMicroseconds(servoPulseWidth3);
        delay(15);
    }


//    while(abs(drawingAngles[0] - currentAngle1) > tolerance || abs(drawingAngles[1] - currentAngle2) > tolerance || abs(drawingAngles[2] - currentAngle3) > tolerance) {
//
////        int servoPulseWidth1 = floatMap(rateLimit(drawingAngles[0], currentAngle1), -PI / 2, PI / 2, 500, 2500);
////        int servoPulseWidth2 = floatMap(rateLimit(drawingAngles[1], currentAngle2), -PI / 2, PI / 2, 500, 2500);
////        int servoPulseWidth3 = floatMap(rateLimit(drawingAngles[2], currentAngle3), PI / 2, -PI / 2, 500, 2500);
//        int servoPulseWidth1 = floatMap(rateLimit(drawingAngles[0], currentAngle1)-0.15, -PI / 2, PI / 2, 500, 2500);
//        int servoPulseWidth2 = floatMap(rateLimit(drawingAngles[1], currentAngle2)-PI/2, -PI / 2, PI / 2, 500, 2500);
//        int servoPulseWidth3 = floatMap(rateLimit(drawingAngles[2], currentAngle3)+PI/2, -PI / 2, PI / 2, 500, 2500);
//
//
//        S1.writeMicroseconds(servoPulseWidth1);
//        S2.writeMicroseconds(servoPulseWidth2);
//        S3.writeMicroseconds(servoPulseWidth3);
//        Serial.println("Current angles:");
//        Serial.println(currentAngle1);
//        Serial.println(currentAngle2);
//        Serial.println(currentAngle3);
//        Serial.println("WE ENTERED THIS LOOP - folded to drawing");
//        delay(15);
//    }

}

void drawingToFolded() {

    double drawingAngles[3];
    cartesianToSpherical(positions2[0], armLengths, drawingAngles);

    double foldedAngles[3] = {80, 175, 0};

    currentAngle1 = drawingAngles[0];
    currentAngle2 = drawingAngles[1];
    currentAngle3 = drawingAngles[2];

    double tolerance = 0.001;
    while(abs(foldedAngles[0] - currentAngle1) > tolerance || abs(foldedAngles[1] - currentAngle2) > tolerance || abs(foldedAngles[2] - currentAngle3) > tolerance) {

        int servoPulseWidth1 = floatMap(rateLimit(foldedAngles[0], currentAngle1), -PI / 2, PI / 2, 500, 2500);
        int servoPulseWidth2 = floatMap(rateLimit(foldedAngles[1], currentAngle2), -PI / 2, PI / 2, 500, 2500);
        int servoPulseWidth3 = floatMap(rateLimit(foldedAngles[2], currentAngle3), PI / 2, -PI / 2, 500, 2500);

        S1.writeMicroseconds(servoPulseWidth1);
        S2.writeMicroseconds(servoPulseWidth2);
        S3.writeMicroseconds(servoPulseWidth3);
        delay(15);
    }

}

void Assignment_6_robot_arm() {
//    goToPosition(positions2[0]);
//    drawStraightLine(foldedPosition, positions2[0]);
    fold_arm();
//    delay(1000);
//    foldedToDrawing();
//    delay(1000);

    //vertical line triangle
    drawStraightLine(foldedPosition, positions2[0]);
    drawStraightLine(positions2[0], positions2[1]);
    drawStraightLine(positions2[1], positions2[2]);
    drawStraightLine(positions2[2], positions2[0]);

    //horizontal line triangle
    drawStraightLine(positions2[0], positions2[3]);
    drawStraightLine(positions2[3], positions2[4]);
    drawStraightLine(positions2[4], positions2[5]);
    drawStraightLine(positions2[5], positions2[0]);
    drawStraightLine(positions2[0], foldedPosition);
//    drawStraightLine(positions2[0], foldedPosition);
//    delay(1000);
//    drawingToFolded();
    fold_arm();
    Serial.println("---------------------------------\n\n\n");


}


// Mobile robot functions -----------------------------------------------------------------------------------------------
byte read_line_sensors(){
    int sensorReading1 = analogRead(SENSOR1);
    int sensorReading2 = analogRead(SENSOR2);
    int sensorReading3 = analogRead(SENSOR3);

    int value = ((sensorReading1 > LINE_SENSED_THRESHOLD) << 2) |
                ((sensorReading2 > LINE_SENSED_THRESHOLD) << 1) |
                (sensorReading3 > LINE_SENSED_THRESHOLD);
    return value;
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

//void follow_line(double distance_to_travel_m) {
//
//    left_encoder_count = 0;
//    right_encoder_count = 0;
//
//    int left_speed = 60;
//    int right_speed = 60;
//
//
//    double rotation = COUNTS_PER_REVOLUTION * (abs(distance_to_travel_m) / (PI * WHEEL_DIAMETER_M));
//
//    double Kp = 3;
//    double minor_gain = 1;
//    double major_gain = 2;
//
//    byte line_sensor_reading = read_line_sensors();
//
//    while (line_sensor_reading != 0b111 && (abs(left_encoder_count) < rotation || abs(right_encoder_count) < rotation)) {
//        switch (line_sensor_reading) {
//            case (0b000):
////                LeftMotor.drive(0);
////                RightMotor.drive(0);
//                break;
//            case (0b010):
//                break;
//            case (0b011):
//                left_speed += Kp * minor_gain;
//                right_speed -= Kp * minor_gain;
//                break;
//            case(0b001):
//                left_speed += Kp * major_gain;
//                right_speed -= Kp * major_gain;
//                break;
//            case (0b110):
//                left_speed -= Kp * minor_gain;
//                right_speed += Kp * minor_gain;
//                break;
//            case (0b100):
//                left_speed -= Kp * major_gain;
//                right_speed += Kp * major_gain;
//                break;
//
//        }
//
//        LeftMotor.drive(left_speed);
//        RightMotor.drive(right_speed);
//        delay(10);
//        line_sensor_reading = read_line_sensors();
//    }
//
//}

void robot_brake(int del) {
    LeftMotor.brake();
    RightMotor.brake();
    delay(del);
}

void robot_forward(double distance_to_travel_m, bool until_line, bool stop_on_horizontal) {
    Serial.println("ROBOT FORWARDS");

    // Closed loop control
    long loop_start = millis();
    // Reset encoder counts
    left_encoder_count = 0;
    right_encoder_count = 0;

    double left_speed = 0;
    double right_speed = 0;

    long old_left_encoder_count = left_encoder_count;
    long old_right_encoder_count = right_encoder_count;

    double rotation = COUNTS_PER_REVOLUTION * (abs(distance_to_travel_m) / (PI * WHEEL_DIAMETER_M));

    int loop_time_ms = 10;
    delay(loop_time_ms);
    byte line_sensor_reading = read_line_sensors();


    // 4000 is the MAX for some reason
    double reference_speed_left = (VBase * loop_time_ms * 1e-3); // encoder counts per loop_time_ms
    double reference_speed_right = (VBase * loop_time_ms * 1e-3); // encoder counts per loop_time_ms
    double Kp = 0.1;
    if (!until_line) {
        while ((abs(left_encoder_count) < rotation) && (abs(right_encoder_count) < rotation)) {
            loop_start = millis();
            double left_error = reference_speed_left - (left_encoder_count - old_left_encoder_count);
            double right_error = reference_speed_right - (right_encoder_count - old_right_encoder_count);
            long pos_diff = left_encoder_count - right_encoder_count;
            old_left_encoder_count = left_encoder_count;
            old_right_encoder_count = right_encoder_count;

            // Update speeds - proportional control
            left_speed += Kp * left_error;
            right_speed += Kp *  right_error;

            // saturate speed
            left_speed = constrain(left_speed, -255, 255);
            right_speed = constrain(right_speed, -255, 255);

            (abs(left_encoder_count < rotation)) ? LeftMotor.drive((int)left_speed) : LeftMotor.drive(0); // should we brake here?
            (abs(right_encoder_count < rotation)) ? RightMotor.drive((int)right_speed) : RightMotor.drive(0);

//        Serial.println(String(left_encoder_count) + " " + String(right_encoder_count) + " " + String(rotation));
//        Serial.print(String(left_error) + " " + String(right_error) + " ");
//            Serial.println(String(left_speed) + " " + String(right_speed));
            Serial.println(String(reference_speed_left) + " " + String(reference_speed_right));
//        Serial.println(pos_diff);


            line_sensor_reading = read_line_sensors();
            int a = 400;
            double b = 1.5;
            switch (line_sensor_reading) {
                case (0b000):
//                    reference_speed_left = (VBase * loop_time_ms * 1e-3);
//                    reference_speed_right = (VBase * loop_time_ms * 1e-3);
                    break;
                case (0b010):
                    reference_speed_left = (VBase * loop_time_ms * 1e-3);
                    reference_speed_right = (VBase * loop_time_ms * 1e-3);
                    break;
                case (0b011):
                    reference_speed_left = ((VBase + a) * loop_time_ms * 1e-3);
                    reference_speed_right =((VBase - a) * loop_time_ms * 1e-3);
                    break;
                case(0b001):
                    reference_speed_left =  ((VBase + a*b) * loop_time_ms * 1e-3);
                    reference_speed_right = ((VBase - a*b) * loop_time_ms * 1e-3);
                    break;
                case (0b110):
                    reference_speed_left =  ((VBase - a) * loop_time_ms * 1e-3);
                    reference_speed_right = ((VBase + a) * loop_time_ms * 1e-3);
                    break;
                case (0b100):
                    reference_speed_left = ((VBase - a*b) * loop_time_ms * 1e-3);
                    reference_speed_right =((VBase + a*b) * loop_time_ms * 1e-3);
                    break;
                case (0b111): // if we see the horizontal line, stop
                    if (stop_on_horizontal){
                        robot_brake(100);
                        return;
                    }
            }
            delay(loop_time_ms - (millis() - loop_start));

        }
    }
    else {
        while (!read_line_sensors()) { // until any sensors are triggered.
            loop_start = millis();
            long left_error = reference_speed_left - (left_encoder_count - old_left_encoder_count);
            long right_error = reference_speed_right - (right_encoder_count - old_right_encoder_count);
            long pos_diff = left_encoder_count - right_encoder_count;
            old_left_encoder_count = left_encoder_count;
            old_right_encoder_count = right_encoder_count;

            // Update speeds - proportional control
            left_speed += Kp * (double) left_error;
            right_speed += Kp * (double) right_error;

            // saturate speed
            left_speed = constrain(left_speed, -255, 255);
            right_speed = constrain(right_speed, -255, 255);

            LeftMotor.drive(left_speed);
            RightMotor.drive(right_speed);

//        Serial.println(String(left_encoder_count) + " " + String(right_encoder_count) + " " + String(rotation));
//        Serial.print(String(left_error) + " " + String(right_error) + " ");
            Serial.println(String(left_speed) + " " + String(right_speed));
//        Serial.println(pos_diff);
            delay(loop_time_ms - (millis() - loop_start));
        }
    }

    robot_brake(100);
}

void robot_backwards(double distance_to_travel_m) {
    Serial.println("ROBOT BACKWARDS");
    // Closed loop control
    long loop_start = millis();
    // Reset encoder counts
    left_encoder_count = 0;
    right_encoder_count = 0;

    double left_speed = 0;
    double right_speed = 0;

    long old_left_encoder_count = left_encoder_count;
    long old_right_encoder_count = right_encoder_count;

    double rotation = -COUNTS_PER_REVOLUTION * (abs(distance_to_travel_m) / (PI * WHEEL_DIAMETER_M));

    int loop_time_ms = 10;
    delay(loop_time_ms);

    // 4000 is the MAX for some reason
    long reference_speed = (long)(-2000.0 * loop_time_ms*1e-3); // encoder counts per loop_time_ms
    double Kp = 0.1;

    long start = millis();
    while ((abs(left_encoder_count) < abs(rotation)) && (abs(right_encoder_count) < abs(rotation))) { // until any sensors are triggered.
        loop_start = millis();
        long left_error = reference_speed -  (left_encoder_count - old_left_encoder_count);
        long right_error = reference_speed - (right_encoder_count - old_right_encoder_count);
        long pos_diff = left_encoder_count - right_encoder_count;
        old_left_encoder_count = left_encoder_count;
        old_right_encoder_count = right_encoder_count;

        // Update speeds - proportional control
        left_speed += Kp * (double)left_error;
        right_speed += Kp * (double)right_error;

        // saturate speed
        left_speed = constrain(left_speed, -255, 255);
        right_speed = constrain(right_speed, -255, 255);

        (abs(left_encoder_count)>rotation) ? LeftMotor.drive(left_speed) : LeftMotor.drive(0); // should we brake here?
        (abs(right_encoder_count)>rotation) ? RightMotor.drive(right_speed) : RightMotor.drive(0);

//        Serial.println(String(left_encoder_count) + " " + String(right_encoder_count) + " " + String(rotation));
//        Serial.print(String(left_error) + " " + String(right_error) + " ");
        Serial.println(String(left_speed) + " " + String(right_speed));
//        Serial.println(pos_diff);
        delay(loop_time_ms-(millis()-loop_start));
    }
    LeftMotor.brake();
    RightMotor.brake();
}

void robot_spin_cw(int degrees, bool until_line) {
    left_encoder_count = 0;
    right_encoder_count = 0;

    double distance_to_travel = (2.0*PI*WHEEL_ROBOT_RADIUS_M * (degrees / 360.0));
    double rotation = COUNTS_PER_REVOLUTION * (distance_to_travel / (PI*WHEEL_DIAMETER_M));
//    double rotation = COUNTS_PER_REVOLUTION * (degrees / 360.0);

    long loop_start = millis();

    double left_speed = 0;
    double right_speed = 0;

    long old_left_encoder_count = left_encoder_count;
    long old_right_encoder_count = right_encoder_count;

    int loop_time_ms = 10;
    delay(loop_time_ms);

    // 4000 is the MAX for some reason
    long reference_speed_left = (long)(2000.0 * loop_time_ms*1e-3); // encoder counts per loop_time_ms
    long reference_speed_right = (long)(-2000.0 * loop_time_ms*1e-3); // encoder counts per loop_time_ms
    double Kp = 0.1;

    if (!until_line) {
        while ((abs(left_encoder_count) < rotation || abs(right_encoder_count) < rotation)) {
            loop_start = millis();
            long left_error = reference_speed_left -  (left_encoder_count - old_left_encoder_count);
            long right_error = reference_speed_right - (right_encoder_count - old_right_encoder_count);
            long pos_diff = left_encoder_count - right_encoder_count;
            old_left_encoder_count = left_encoder_count;
            old_right_encoder_count = right_encoder_count;

            // Update speeds - proportional control
            left_speed += Kp * (double)left_error;
            right_speed += Kp * (double)right_error;

            // saturate speed
            left_speed = constrain(left_speed, -255, 255);
            right_speed = constrain(right_speed, -255, 255);

            (abs(left_encoder_count)<rotation) ? LeftMotor.drive(left_speed) : LeftMotor.drive(0); // should we brake here?
            (abs(right_encoder_count)<rotation) ? RightMotor.drive(right_speed) : RightMotor.drive(0);

//        Serial.println(String(left_encoder_count) + " " + String(right_encoder_count) + " " + String(rotation));
//        Serial.print(String(left_error) + " " + String(right_error) + " ");
            Serial.println(String(left_speed) + " " + String(right_speed));
//        Serial.println(pos_diff);
            delay(loop_time_ms-(millis()-loop_start));
        }
    }
    else {
        while(read_line_sensors() != 0b010){
            loop_start = millis();
            long left_error = reference_speed_left -  (left_encoder_count - old_left_encoder_count);
            long right_error = reference_speed_right - (right_encoder_count - old_right_encoder_count);
            long pos_diff = left_encoder_count - right_encoder_count;
            old_left_encoder_count = left_encoder_count;
            old_right_encoder_count = right_encoder_count;

            // Update speeds - proportional control
            left_speed += Kp * (double)left_error;
            right_speed += Kp * (double)right_error;

            // saturate speed
            left_speed = constrain(left_speed, -255, 255);
            right_speed = constrain(right_speed, -255, 255);

            (abs(left_encoder_count)<rotation) ? LeftMotor.drive(left_speed) : LeftMotor.drive(0); // should we brake here?
            (abs(right_encoder_count)<rotation) ? RightMotor.drive(right_speed) : RightMotor.drive(0);

//        Serial.println(String(left_encoder_count) + " " + String(right_encoder_count) + " " + String(rotation));
//        Serial.print(String(left_error) + " " + String(right_error) + " ");
            Serial.println(String(left_speed) + " " + String(right_speed));
//        Serial.println(pos_diff);
            delay(loop_time_ms-(millis()-loop_start));
        }
    }
    LeftMotor.brake();
    RightMotor.brake();
}

void robot_spin_ccw(int degrees, bool until_line) {
    left_encoder_count = 0;
    right_encoder_count = 0;

    double distance_to_travel = (2.0*PI*WHEEL_ROBOT_RADIUS_M * (degrees / 360.0));
    double rotation = COUNTS_PER_REVOLUTION * (distance_to_travel / (PI*WHEEL_DIAMETER_M));
//    double rotation = COUNTS_PER_REVOLUTION * (degrees / 360.0);

    long loop_start = millis();

    double left_speed = 0;
    double right_speed = 0;

    long old_left_encoder_count = left_encoder_count;
    long old_right_encoder_count = right_encoder_count;

    int loop_time_ms = 10;
    delay(loop_time_ms);

    // 4000 is the MAX for some reason
    long reference_speed_left = (long)(-2000.0 * loop_time_ms*1e-3); // encoder counts per loop_time_ms
    long reference_speed_right = (long)(2000.0 * loop_time_ms*1e-3); // encoder counts per loop_time_ms
    double Kp = 0.1;

    if (!until_line) {
        while ((abs(left_encoder_count) < rotation || abs(right_encoder_count) < rotation)) {
            loop_start = millis();
            long left_error = reference_speed_left -  (left_encoder_count - old_left_encoder_count);
            long right_error = reference_speed_right - (right_encoder_count - old_right_encoder_count);
            long pos_diff = left_encoder_count - right_encoder_count;
            old_left_encoder_count = left_encoder_count;
            old_right_encoder_count = right_encoder_count;

            // Update speeds - proportional control
            left_speed += Kp * (double)left_error;
            right_speed += Kp * (double)right_error;

            // saturate speed
            left_speed = constrain(left_speed, -255, 255);
            right_speed = constrain(right_speed, -255, 255);

            (abs(left_encoder_count)<rotation) ? LeftMotor.drive(left_speed) : LeftMotor.drive(0); // should we brake here?
            (abs(right_encoder_count)<rotation) ? RightMotor.drive(right_speed) : RightMotor.drive(0);

//        Serial.println(String(left_encoder_count) + " " + String(right_encoder_count) + " " + String(rotation));
//        Serial.print(String(left_error) + " " + String(right_error) + " ");
            Serial.println(String(left_speed) + " " + String(right_speed));
//        Serial.println(pos_diff);
            delay(loop_time_ms-(millis()-loop_start));
        }
    }
    else {
        while(read_line_sensors() != 0b010){
            loop_start = millis();
            long left_error = reference_speed_left -  (left_encoder_count - old_left_encoder_count);
            long right_error = reference_speed_right - (right_encoder_count - old_right_encoder_count);
            long pos_diff = left_encoder_count - right_encoder_count;
            old_left_encoder_count = left_encoder_count;
            old_right_encoder_count = right_encoder_count;

            // Update speeds - proportional control
            left_speed += Kp * (double)left_error;
            right_speed += Kp * (double)right_error;

            // saturate speed
            left_speed = constrain(left_speed, -255, 255);
            right_speed = constrain(right_speed, -255, 255);

            LeftMotor.drive(left_speed) ; // should we brake here?
            RightMotor.drive(right_speed);

//        Serial.println(String(left_encoder_count) + " " + String(right_encoder_count) + " " + String(rotation));
//        Serial.print(String(left_error) + " " + String(right_error) + " ");
            Serial.println(String(left_speed) + " " + String(right_speed));
//        Serial.println(pos_diff);
            delay(loop_time_ms-(millis()-loop_start));
        }
    }
    LeftMotor.brake();
    RightMotor.brake();
}




void setup() {
    timestamp = millis();
    S1.attach(S1_PWM);
    S2.attach(S2_PWM);
    S3.attach(S3_PWM);

    Serial.begin(115200);

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

//void loop() {
//    fold_arm();
//    delay(1000);
////    Assignment_6_robot_arm();
////    delay(1000);
//
//}



void loop() {
     //this is just testing stuff
//    robot_spin_cw(90, false);
//    robot_brake(500);
//    robot_forward(1, false); // bottom
//    robot_brake(500);
//    robot_forward(WHEEL_ROBOT_RADIUS_M, false); // bottom.
//    robot_spin_ccw(90, true);
//    follow_line();
//
//    robot_spin_ccw(90, true);
//    robot_brake(1000);
//    robot_backwards(0.1);
    int a = 1000; // ms

    // Start
    fold_arm();
    delay(6000);

    robot_spin_cw(85, false); delay(a);
    robot_forward(0, true, false);  delay(a);// BOTTOM: drive, until line.
    robot_forward(WHEEL_ROBOT_RADIUS_M, false, false); delay(a); // BOTTOM: inch forward a bit
    robot_spin_ccw(0, true); delay(a);// turn until line
    robot_forward(1.0, false, true); delay(a);// RIGHT: drive 0.97m, following line.
    Assignment_6_robot_arm();
    delay(500);
    fold_arm();
    robot_backwards(0.1);
    delay(1000);
    robot_spin_ccw(85, false); delay(a); // turn 90 degrees.
    robot_forward(0, true, false); delay(a); // TOP: drive until line
    robot_forward(WHEEL_ROBOT_RADIUS_M, false, false); delay(a);// TOP: inch forward a bit
    robot_spin_ccw(0, true); delay(a);// turn until line
    robot_forward(1.0, false, true); delay(a);// LEFT: drive 0.7m, following line.
    robot_brake(a);
    while(1); // STOP
}



void testingLoop() {
//    robot_forward(0.3);
//    robot_brake(1000);
//    robot_backwards(0.3);
//    robot_brake(1000);

//    LeftMotor.drive(100);
//    RightMotor.drive(100);
//    delay(1000);
//    LeftMotor.brake();
//    RightMotor.brake();
//    delay(1000);
//
//    LeftMotor.drive(-100);
//    RightMotor.drive(-100);
//    delay(1000);
//    LeftMotor.brake();
//    RightMotor.brake();


    robot_spin_cw(90, false);
    robot_brake(1000);
    robot_spin_ccw(90, false);
    robot_brake(1000);

    // test encoders
//    robot_forward(1);
//    Serial.println(String(left_encoder_count) + " " + String(right_encoder_count));
//    delay(10);

}


//    Serial.println(read_line_sensors(), BIN);
//    delay(10);

//void loop() {
//    // Homing sequence
//    S1.write(90);
//    S2.write(90);
//    S3.write(180);
//}