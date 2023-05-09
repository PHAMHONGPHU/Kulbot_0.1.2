/*
The firmware is developed by 3S JSC.
@ Written by HungDang
*/
#include "Arduino.h"
/* -------------------------------WIFI AP--------------------------------- */
 #define AP_NAME_SSID_DEFAULT (String)"KulRobot"
 #define AP_CHANEL_WIFI         (uint8_t)10
 #define AP_SSID_HIDEN_WIFI     (uint8_t)1
 #define AP_MAX_CONNECTION_WIFI (uint8_t)1
/* -------------------------------RGB LED--------------------------------- */
#define WS2812_Brightness_MAX (uint8_t)100
#define WS2812_Brightness_MIN (uint8_t)0
#define RGB_LED_COUNT    (uint8_t)9
#define RGB_COLOR_RED    (uint8_t)0
#define RGB_COLOR_ORANGE (uint8_t)1 
#define RGB_COLOR_YELLOW (uint8_t)2
#define RGB_COLOR_GREEN  (uint8_t)3
#define RGB_COLOR_BLUE   (uint8_t)4
#define RGB_COLOR_INDIGO (uint8_t)5
#define RGB_COLOR_VIOLET (uint8_t)6
#define RGB_COLOR_WHITE  (uint8_t)7
#define RGB_COLOR_BLACK  (uint8_t)8
/* -------------------------------Motor--------------------------------- */
#define _MOTOR_MODE_TIMER      (uint8_t)0
#define _MOTOR_MODE_ROTATION   (uint8_t)1
#define DRIVE_MOTOR_FREQ       (uint32_t)100000
#define DRIVE_PWM_MOTOR_ADDR   (byte)0x40
#define PWM_FREQ               (uint16_t)31000
#define LED_PWM_CHANEL1        (uint8_t)0
#define LED_PWM_CHANEL2        (uint8_t)1
#define PWM_RESOLUTION         (uint8_t)10
#define PWM_MAX_DUTY_CYCLE     (uint16_t)1023
#define PWM_MIN_DUTY_CYCLE     (uint16_t)0
#define MAX_SPEED_MOTOR        (uint8_t)100
#define MIN_SPEED_MOTOR        (uint8_t)0

#define DRIVE_PWM_FREQ            (uint8_t) 60
#define DRIVE_OscillatorFrequency (uint32_t)27000000
#define DRIVE_MAX_DUTY_CYCLE      (int)4096
#define DRIVE_MIN_DUTY_CYCLE      (int)0

#define DRIVE_MOTORA  (uint8_t)0
#define DRIVE_MOTORB  (uint8_t)1
#define DRIVE_MOTORAB (uint8_t)2
#define DRIVE_MOVE_FORWARD  (uint8_t)0
#define DRIVE_MOVE_BACKWARD (uint8_t)1
#define DIR_MA1     (uint8_t)3
#define DIR_MA2     (uint8_t)4
#define DIR_MB1     (uint8_t)5
#define DIR_MB2     (uint8_t)6
#define ENCODER_PPM (uint8_t)135

/* -------------------------------Servo--------------------------------- */
#define SERVO_MIN_PULSE   (int)125
#define SERVO_MAX_PULSE   (int)575
#define SERVO_ANGLE_MAX   (uint8_t)180
#define SERVO_ANGLE_MIN   (uint8_t)0
#define _SERVO_PORT_1     (uint8_t)8
#define _SERVO_PORT_2     (uint8_t)9
#define _SERVO_PORT_3     (uint8_t)10
#define _SERVO_PORT_4     (uint8_t)11
#define _SERVO_PORT_5     (uint8_t)12
#define _SERVO_PORT_6     (uint8_t)13
#define _SERVO_PORT_7     (uint8_t)14
#define _SERVO_PORT_8     (uint8_t)15
/* -------------------------------Sensor--------------------------------- */
#define _SENSOR_PORT_1  (uint8_t)0
#define _SENSOR_PORT_2  (uint8_t)1
#define _SENSOR_PORT_3  (uint8_t)2
#define _SENSOR_PORT_4  (uint8_t)3
#define _SENSOR_PORT_5  (uint8_t)4
#define _SENSOR_PORT_6  (uint8_t)5
#define _SENSOR_PORT_7  (uint8_t)6
#define _SENSOR_PORT_8  (uint8_t)7

#define _DRIVE_SENSOR_ADDRESS (byte)0x70
#define _DRIVE_SENSOR_FREQ (uint32_t)400000
/* -------------------------------LCD--------------------------------- */
#define _DRIVE_LCD_ADDRESS    (byte)0x3F  //3F
#define _DRIVE_LCD_NUM_CELLS  (uint8_t)16
#define _DRIVE_LCD_NUM_COLUMN (uint8_t)2
/* -------------------------------Line sensor--------------------------------- */
#define _LINE_SENSOR_ADDRESS (byte)0x3E  // 3E
#define _LINE_SENSOR_LEFT    (uint8_t)0
#define _LINE_SENSOR_RIGHT   (uint8_t)1
/* -------------------------------Touch sensor--------------------------------- */
#define _TOUCH_SENSOR_ADDRESS (byte)0x38
/* -------------------------------IR sensor--------------------------------- */
#define _IR_SENSOR_ADDRESS  (byte)0x39  //0x39
#define _IR_COLOR_RED       (uint8_t)0
#define _IR_COLOR_GREEN     (uint8_t)1
#define _IR_COLOR_BLUE      (uint8_t)2
#define _IR_COLOR_YELLOW    (uint8_t)3
#define _IR_COLOR_CYN       (uint8_t)4
#define _IR_COLOR_VIOLET    (uint8_t)5
#define _IR_COLOR_WHITE     (uint8_t)6
/* -------------------------------DHT12 sensor--------------------------------- */
/* -------------------------------Sonar sensor--------------------------------- */
#define _SONAR_SENSOR_ADDRESS     (byte)0x10
#define _SONAR_SENSOR_CONFIG_READ (byte)0x01
/* -------------------------------trafic light--------------------------------- */
#define _TRAFFIC_SENSOR_ADDRESS (byte)0x3A
#define _red_traffic            (uint8_t)0
#define _green_traffic          (uint8_t)1
#define _yellow_traffic         (uint8_t)2
#define _status_traffic_on      (bool)1
#define _status_traffic_off     (bool)0
/* -------------------------------button led--------------------------------- */
#define _BUTTON_LED_ADDRESS  (byte)0x3c
#define _LEFT_BUTTON_LED     (uint8_t)0
#define _RIGHT_BUTTON_LED    (uint8_t)1
#define _BUTTON_COLOR_RED     (uint8_t)0
#define _BUTTON_COLOR_GREEN   (uint8_t)1
#define _BUTTON_COLOR_BLUE    (uint8_t)2
#define _BUTTON_COLOR_YELLOW  (uint8_t)3
#define _BUTTON_COLOR_CYN     (uint8_t)4
#define _BUTTON_COLOR_VIOLET  (uint8_t)5
#define _BUTTON_COLOR_WHITE   (uint8_t)6
/* -------------------------------Gryro sensor--------------------------------- */
#define _GRYRO_SENSOR_ADDRESS (byte)0x68
#define _get_GRYRO_ANGLE_X (uint8_t)0
#define _get_GRYRO_ANGLE_Y (uint8_t)1
#define _get_GRYRO_ANGLE_Z (uint8_t)2
#define _get_GRYRO_ACC_X   (uint8_t)3
#define _get_GRYRO_ACC_Y   (uint8_t)4
#define _get_GRYRO_ACC_Z   (uint8_t)5
#define _get_GRYRO_X       (uint8_t)6
#define _get_GRYRO_Y       (uint8_t)7
#define _get_GRYRO_Z       (uint8_t)8
#define _get_GRYRO_ACC_ANGLE_X  (uint8_t) 9
#define _get_GRYRO_ACC_ANGLE_Y  (uint8_t) 10
#define _get_GRYRO_ACC_ANGLE_Z  (uint8_t) 11
/* -------------------------------Color sensor--------------------------------- */
#define _COLOR_SENSOR_ADDRESS (byte)0x29


#define _COLOR_IS_RED     (bool)0
#define _COLOR_IS_ORANGE  (bool)0
#define _COLOR_IS_YELLOW  (bool)0
#define _COLOR_IS_GREEN   (bool)0
#define _COLOR_IS_BLUE    (bool)0
#define _COLOR_IS_INDINGO (bool)0
#define _COLOR_IS_VIOLET  (bool)0
#define _COLOR_IS_WHILE   (bool)0
#define _COLOR_IS_BLACK   (bool)0
/* -------------------------------volume sensor--------------------------------- */
#define _VOLUME_SENSOR_ADDRESS (byte)0x48
#define _VOLUME_MAX_VALUE (uint8_t)100
#define _VOLUME_MIN_VALUE (uint8_t)0
/* -------------------------------soil hum sensor--------------------------------- */
#define _SOIL_HUM_SENSOR_ADDRESS (byte)0x48
/* -------------------------------light sensor--------------------------------- */
#define _LIGHT_SENSOR_ADDRESS (byte)0x48
/* -------------------------------Gas sensor--------------------------------- */
#define _GAS_SENSOR_ADDRESS (byte)0x48
/* -------------------------------Joystick sensor--------------------------------- */
#define _JOYSTICK_SENSOR_ADDRESS (byte)0x48
#define _Joystick_X              (uint8_t)0
#define _Joystick_Y              (uint8_t)1
