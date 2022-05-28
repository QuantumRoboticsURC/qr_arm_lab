//actualizacion 14 mayo
#include <Arduino.h>
#include <Servo.h>
#include <Adafruit_NeoPixel.h>

#define pinLeftServo 9
#define pinRightServo 6
#define pinGripper 10
#define pinCamara 5


// Pin to the Status LED
#define pinStatusLed 11
#define NUMPIXELS 64 //Número de píxeles
#define DELAYVAL 1 //timpo de espera en ms 
Adafruit_NeoPixel pixels(NUMPIXELS, pinStatusLed, NEO_GRB + NEO_KHZ800);

Servo leftServo; // the servo used to control a camera that can look around the robot
Servo rightServo;
Servo gripperServo;
Servo camServo;
int last_color = 0;
long int start;
long int now;
int flag = 0;

struct DATA {
    float servo1;
    float servo2;
    float servo3;
    float joint3;    
    float movement;
} received;

void setup() {
    Serial.begin(9600);
    /*leftServo.attach(pinLeftServo);
    rightServo.attach(pinRightServo);
    gripperServo.attach(pinGripper);
    camServo.attach(pinCamara);
    gripperServo.writeMicroseconds(1500);
    camServo.write(180);
    leftServo.write(0);
    rightServo.write(170);
    pixels.begin(); // Inicializamos el objeto "pixeles"*/
}

void loop() {
    /*pixels.clear(); // Apagamos todos los LEDs
        if (flag == 0){
      start = millis ();
      flag = 1;
    }
    now = millis();
    if (now - start > 2500){
            float value  = float(analogRead(A7)*25.0/1023);
      Serial.println("v" + String(value));
      flag = 0;
    }*/
    
    if (Serial.available() >= sizeof(uint8_t)) {
        delayMicroseconds(10);
        uint8_t cmd = (uint8_t) Serial.read();
        // SERVO MOTOR COMMAND
        switch(cmd){
          case 0:
            Serial.readBytes((char *) &received.joint3, sizeof(float));
            Serial.print("joint3\n");
            Serial.println((int)received.joint3);
            int grados=map((int)received.joint3,-90,90,1740,1180);
            Serial.println(grados);            
            /*myservo.writeMicroseconds(grados);//
            Serial.print("joint3");*/
          break;
          case 1:
            Serial.readBytes((char *) &received.servo1, sizeof(float));
            Serial.print("servo1");
          break;
          case 2:
            Serial.readBytes((char *) &received.servo2, sizeof(float));
            Serial.print("servo2");
          break;
          case 3:
            Serial.readBytes((char *) &received.servo3, sizeof(float));
            Serial.print("servo3");
          break;
          case 4:
            Serial.readBytes((char *) &received.movement, sizeof(float));
            Serial.print("centrifuga");
          break;
        }
  /*        if (cmd == 0) {
            Serial.readBytes((char *) &received.joint3, sizeof(float));
            Serial.print("joint3");
            /*int left_angle=(int)received.servo;
            int right_angle=170-(int)received.servo;
            Serial.print("Left Servo: ");
            Serial.print(left_angle);
            Serial.print("Right Servo: ");
            Serial.print(right_angle);
            leftServo.write(left_angle);
            rightServo.write(right_angle);*/
        //}
        
        /*// STATUS COMMAND
        else if (cmd == 2) {
            Serial.readBytes((char *) &received.status, sizeof(float));
            if ((int) received.status == 1){
                last_color = 1;
                Serial.print("Blue: Teleoperation (Manually driving)");
                for(int i=0; i<NUMPIXELS; i++) {
                    pixels.setPixelColor(i, pixels.Color(0, 0, 255));
                    pixels.show();
                    delay(DELAYVAL);
                }
            }
            else if ((int) received.status == 2){
                last_color = 2;
                Serial.print("Red: Autonomous operation");
                for(int i=0; i<NUMPIXELS; i++) {
                    pixels.setPixelColor(i, pixels.Color(255, 0, 0));
                    pixels.show();
                    delay(DELAYVAL);
                }
            }
            else if ((int) received.status == 3){
                Serial.print("# Flashing Green: Successful arrival at a post or passage through a gate");
                for(int i=0; i<10; i++) {
                    for(int i=0; i<NUMPIXELS; i++) {
                    pixels.setPixelColor(i, pixels.Color(0, 0, 0));
                    pixels.show();
                    delay(DELAYVAL);
                    }
                   
                    for(int i=0; i<NUMPIXELS; i++) {
                    pixels.setPixelColor(i, pixels.Color(0, 255, 0));
                    pixels.show();
                    delay(DELAYVAL);
                    }
                    delay(100);
                }
              if (last_color==1){
                for(int i=0; i<NUMPIXELS; i++) {
                    pixels.setPixelColor(i, pixels.Color(0, 0, 255));
                    pixels.show();
                    delay(DELAYVAL);
                }
              }
              else if (last_color==2){
                 for(int i=0; i<NUMPIXELS; i++) {
                    pixels.setPixelColor(i, pixels.Color(255, 0, 0));
                    pixels.show();
                    delay(DELAYVAL);
                }
              }
             
            }
        }
        
        else if (cmd == 3) {
            Serial.readBytes((char *) &received.gripper, sizeof(float));
            int gripper_angle=(int)received.gripper;
            Serial.print("Gripper_Angle: ");
            Serial.print(gripper_angle);
            gripperServo.writeMicroseconds(gripper_angle);
        }
        
        else if (cmd == 4){
            Serial.readBytes((char *) &received.cam, sizeof(float));
            int cam_angle=(int)received.cam;
            Serial.print("Cam Angle: ");
            Serial.print(cam_angle);
            camServo.write(cam_angle);
        }*/
    }
}
