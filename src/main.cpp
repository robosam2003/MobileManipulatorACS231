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
unsigned long controlRate = 50; // ms

double currentAngle1 = 0;
double currentAngle2 = 0;
double currentAngle3 = 0;

double armLengths[3] = {120, 80, 130};
double armAngles[3] = {0};



#define increment 0.017 // radians per 15ms



// Number positions
double numberPositions[10][3] = {
  {-150, 70, 70}, // 0
  {-125, 110, 40}, // 1
  {-95, 130, 30}, // 2
  {-60, 130, 20}, // 3
  {-20, 140, 20}, // 4
  {15,  140, 20}, // 5
  {55,  140, 20}, // 6
  {90,  130, 30}, // 7
  {130,  120, 45}, // 8
  {150,  90, 60}  // 9
};


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
        angles[0] = 0;
        angles[1] = 0;
        angles[2] = 0;
        return;
    }
    // Base angle
    angles[0] = atan2(x, y);

    double theta2_prime = acos( (l1*l1 + l2*l2 - l*l) / (2*l1*l2) );
    Serial.println(theta2_prime);
    double a = atan2(z, y);
    Serial.println(a);
    double b = asin( (sin(theta2_prime)*lengths[2]) / l );
    Serial.println(b);
    angles[1] = a + b;
    angles[2] = -(PI - theta2_prime);

    Serial.println(l);
    Serial.println(angles[0]);
    Serial.println(angles[1]);
    Serial.println(angles[2]);
    // forward kinematics
    Serial.print("FORWARD KINEMATICS:  ");
    Serial.print(l1*cos(angles[1])+ l2*cos(angles[1] + angles[2]));
    Serial.print(",   ");
    Serial.print(l0+l1*sin(angles[1])+ l2*sin(angles[1] + angles[2]));
    Serial.println("\n");

}



void goToPosition(double coords[3]) {
    cartesianToSpherical(coords, armLengths, armAngles);
//    double tolerance = 0.001;
//    while(abs(armAngles[0] - currentAngle1) > tolerance || abs(armAngles[1] - currentAngle2) > tolerance || abs(armAngles[2] - currentAngle3) > tolerance) {

//        int servoPulseWidth1 = floatMap(rateLimit(armAngles[0], currentAngle1), -PI / 2, PI / 2, 500, 2500);
//        int servoPulseWidth2 = floatMap(rateLimit(armAngles[1], currentAngle2), -PI / 2, PI / 2, 500, 2500);
//        int servoPulseWidth3 = floatMap(rateLimit(armAngles[2], currentAngle3), PI / 2, -PI / 2, 500, 2500);
        int servoPulseWidth1 = floatMap(armAngles[0], -PI / 2, PI / 2, 500, 2500);
        int servoPulseWidth2 = floatMap(armAngles[1], -PI / 2, PI / 2, 500, 2500);
        int servoPulseWidth3 = floatMap(armAngles[2], PI / 2, -PI / 2, 500, 2500);

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

void setup() {
    timestamp = millis();
    S1.attach(S1_PWM);
    S2.attach(S2_PWM);
    S3.attach(S3_PWM);

    Serial.begin(9600);
}

//  Lab 2
void loop() {
    double coords[3] = {0, 180, 160};


    // number sequencing
//    home();
//    delay(1000);
//    int sequence[5] = {6, 0, 3, 9, 3};
//    for (int i : sequence) {
//        goToPosition(numberPositions[i]);
//        delay(2000);      mostafa
//
//        home();
//        delay(2000);
//    }
//
//    delay(4000);

// Straight line bit
    while (coords[2] >= 160-101 ) {
        // print coords
        Serial.print("X: ");
        Serial.print(coords[0]);
        Serial.print(" Y: ");
        Serial.print(coords[1]);
        Serial.print(" Z: ");
        Serial.println(coords[2]);

        goToPosition(coords);
        coords[2] -= (double)100/11;
        delay(500);
    }
    Serial.println("---------------------------------\n\n\n");
    delay(1000);



//    while (coords[0] >= -100 ) {
//        // print coords
//        Serial.print("X: ");
//        Serial.print(coords[0]);
//        Serial.print(" Y: ");
//        Serial.print(coords[1]);
//        Serial.print(" Z: ");
//        Serial.println(coords[2]);
//
//        goToPosition(coords);
//        coords[0] -= (double)100/11;
//        delay(500);
//    }
//    Serial.println("---------------------------------\n\n\n");
//    delay(1000);

}




//
//void loop() {
//
//    S1.write(0);
//    S2.write(0);
//    S3.write(0);
//
//}






