//actualizacion 14 mayo
#include <Arduino.h>
#include <Servo.h>

#define pinJoint3 10
#define pinServo1 5
#define pinServo2 6
#define pinServo3 9
#define pinStep   2
#define pinDir    3

int const t_sacudida=1;

Servo joint3Servo;
Servo servo1Servo;
Servo servo2Servo;
Servo servo3Servo;
int valCentrifuga = 0;

struct DATA {
    float servo1;
    float servo2;
    float servo3;
    float joint3;    
    float movement;
} received;

void setup() {
    Serial.begin(9600);
    
    servo1Servo.attach(pinServo1);
    servo1Servo.write(90);
    
    servo2Servo.attach(pinServo2);
    servo2Servo.write(90);

    servo3Servo.attach(pinServo3);
    servo3Servo.write(90);

    joint3Servo.attach(pinJoint3);
    joint3Servo.writeMicroseconds(map(-155,-180,90,2020,1180));

 
    pinMode(pinDir, OUTPUT);
    pinMode(pinStep, OUTPUT);
    
    
}

void loop() {
    
    if (Serial.available() >= sizeof(uint8_t)) {
        delayMicroseconds(10);
        uint8_t cmd = (uint8_t) Serial.read();
        // SERVO MOTOR COMMAND
        
          if (cmd == 0) {
            Serial.readBytes((char *) &received.joint3, sizeof(float));
            Serial.print("joint3\n");
            Serial.println((int)received.joint3);
            //180 deg = 560 ticks  
            //45 deg = 140 tickks
            //15 deg = 46
            //int grados=map((int)received.joint3,-90,90,1740,1180);
            int grados=map((int)received.joint3,-180,90,2020,1180);
            if ((int)received.joint3 >= -155 && (int)received.joint3 <= 90){

              Serial.println(grados);            
              joint3Servo.writeMicroseconds(grados);//
              Serial.println("joint3");
              Serial.println();
            }
        }
        
          else if (cmd == 1){
            Serial.readBytes((char *) &received.servo1, sizeof(float));
            int angle=(int)received.servo1;
             if (angle >= 0 && angle <= 60){
                servo1Servo.write(angle);
             }
            
        }
           else if (cmd == 2){
            Serial.readBytes((char *) &received.servo2, sizeof(float));
            int angle=(int)received.servo2;
            if (angle >= 0 && angle <= 70){
               servo2Servo.write(angle);
            }
        }
        
           else if (cmd == 3){
            Serial.readBytes((char *) &received.servo3, sizeof(float));
            int angle=(int)received.servo3;
            if (angle >= 0 && angle <= 72){
              servo3Servo.write(angle);
            }
        }
        
           else if (cmd == 4){
            Serial.readBytes((char *) &received.movement, sizeof(float));
            Serial.println("entre a centrifuga");
            int value=(int)received.movement;
            valCentrifuga = value;            


            
        }
        
    }
    if (valCentrifuga == 1){
                   digitalWrite(pinDir,LOW);

                    digitalWrite(pinStep,HIGH);
                    delay(t_sacudida);
                    
                    digitalWrite(pinStep,LOW);
                    delay(t_sacudida);
                  
                 
            }
            
            else if (valCentrifuga == -1){
              Serial.println("entre a 2");
                  digitalWrite(pinDir,  HIGH);
                  digitalWrite(pinStep, HIGH);
                  delay(t_sacudida);
                 
                  digitalWrite(pinStep, LOW);
                  delay(t_sacudida);
            }
            
            else if (valCentrifuga == 0){
                Serial.println("entre a 0");
                  digitalWrite(pinDir,  LOW);
                  digitalWrite(pinStep, LOW);
                  delay(t_sacudida);                                  
            }
}
