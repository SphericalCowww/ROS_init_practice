#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();
#define SERVOMIN  125
#define SERVOMAX  575
uint8_t servoIdx = 0;

void setup() {
  Serial.begin(115200);
  Serial.setTimeout(1);
  Serial.println("Servo Test:");
  
  pwm.begin();
  pwm.setPWMFreq(60);  // Analog servos run at ~60 Hz

  pwm.setPWM(servoIdx, 0, (SERVOMAX+SERVOMIN)/2);  // Channel 0, ~1.5ms pulse
}

void loop() {
  int controlInputInt = 0;
  if (Serial.available()) {
    controlInputInt = Serial.readString().toInt();
    if (controlInputInt == 0) {
      for (int ang = 10; ang <= 170; ang += 5) {
        pwm.setPWM(servoIdx, 0, angleToPulse(ang));
        delay(100);
      }
    }
    else {
      pwm.setPWM(servoIdx, 0, angleToPulse(0));
    }
  }
}

int angleToPulse(int ang){
   int pulse = map(ang, 0, 180, SERVOMIN, SERVOMAX);
   Serial.print(" angle: "); Serial.print(ang);
   Serial.print(" pulse: "); Serial.println(pulse);
   return pulse;
}
