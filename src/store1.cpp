//#include <Arduino.h>
//#include <Servo.h>
//
//Servo S1;
//Servo S2;
//Servo S3;
//
//#define S1_PWM 9
//#define S2_PWM 10
//#define S3_PWM 11
//
//#define POT_PIN_1 A0
//#define POT_PIN_2 A1
//#define POT_PIN_3 A2
//
//
//void setup() {
//// write your initialization code here
//    S1.attach(S1_PWM);
//    S2.attach(S2_PWM);
//    S3.attach(S3_PWM);
//
//    Serial.begin(9600);
//
//    pinMode(POT_PIN_1, INPUT);
//    pinMode(POT_PIN_2, INPUT);
//    pinMode(POT_PIN_3, INPUT);
//}
//
//void loop() {
//// write your code here
//    int pot1 = analogRead(POT_PIN_1);
//    int pot2 = analogRead(POT_PIN_2);
//    int pot3 = analogRead(POT_PIN_3);
//
//    Serial.print("Pot1: ");
//    Serial.print(pot1);
//    Serial.print(" Pot2: ");
//    Serial.print(pot2);
//    Serial.print(" Pot3: ");
//    Serial.println(pot3);
//
//    S1.write(map(pot1, 0, 1023, 0, 180));
//    S2.write(map(pot2, 0, 1023, 0, 180));
//    S3.write(map(pot3, 0, 1023, 0, 180));
//
//    delay(100);
//}

// double rateLimit(double desiredAngle, double &currentAngle) {
//    double positionCommand = 0;
//    if (desiredAngle > currentAngle) {
//        positionCommand = min(desiredAngle, currentAngle + increment);
//        currentAngle = positionCommand;
//    }
//    else if (desiredAngle < currentAngle) {
//        positionCommand = max(desiredAngle, currentAngle - increment);
//        currentAngle = positionCommand;
//    }
//    else {
//        positionCommand = desiredAngle;
//        currentAngle = positionCommand;
//    }
//    return positionCommand;
//
//}