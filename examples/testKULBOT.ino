#include <KULBOT.h>
KULBOT Rob;
int servo_position[9];
    float increment[9];
    unsigned long final_time;
    unsigned long partial_time;
    int speed_1;
void setup() {

Rob.KULBOT_INIT();
//Rob.KULBOT_RGB_INIT(100);
Rob.KULBOT_SERVO_INIT();
Rob.KULBOT_SENSOR_INIT();

  Rob.KULBOT_MOTORENCODER_INIT();
  delay(2000);
  Rob.KULBOT_MOTORENCODER_RUN1(0,50,0);
  Rob.KULBOT_MOTORENCODER_RUN1(1,70,0);
  delay(120000);
   Rob.KULBOT_MOTORENCODER_RUN1(0,0,0);
  Rob.KULBOT_MOTORENCODER_RUN1(1,0,0);
}
void loop() {
  
  
  Rob.KULBOT_SERVO_SET_ANGLE(7,90);
  Rob.KULBOT_SERVO_SET_ANGLE(8,90);
  Rob.KULBOT_SERVO_SET_ANGLE(3,90);
  Rob.KULBOT_SERVO_SET_ANGLE(4,90);
}
