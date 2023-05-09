/*
The firmware is developed by 3S JSC.
@ Written by HungDang
*/
#include "Arduino.h"
#include "WiFi.h"
#include "ESPmDNS.h"
#include "pin.h"
#include "define.h"
#include "Wire.h"
#include "FS.h"
#include "SD.h"
#include "SimplePID.h"

#include "PID.h"
#include "RGB.h"
#include "PCA9685.h"
#include "LiquidCrystal_I2C.h"
#include "TCA9548A.h"
#include "PCF8574.h"
#include "DHT12.h"
#include "PCF8591.h"



#ifndef KULBOT_H
#define KULBOT_H
    

class KULBOT
{
  public :
    int servo_position[9];
    float increment[9];
    unsigned long final_time;
    unsigned long partial_time;
   /* KULBOT INIT  */
  void  KULBOT_INIT(void);
  /* WIFI INIT */
  void KULBOT_WIFI_STA_INIT(String SSID,String PASWORD,bool static_IP);
  void KULBOT_GET_IP_STA_WIFI(uint8_t ip1, uint8_t ip2,uint8_t ip3,uint8_t ip4);
  void KULBOT_GET_GATEWAY_STA_WIFI(uint8_t gt1, uint8_t gt2,uint8_t gt3,uint8_t gt4);
  void KULBOT_GET_SUBNET_STA_WIFI(uint8_t sb1, uint8_t sb2,uint8_t sb3,uint8_t sb4);
  /* Control led ws2812  */
  void KULBOT_RGB_INIT(uint8_t WS2812_Brightness);
  void KULBOT_RGB_ON(uint8_t LED_NUM,uint8_t color);
  void KULBOT_RGB_OFF(uint8_t LED_NUM);
  void KULBOT_RGB_ALL_ON(uint8_t color);
  void KULBOT_RGB_ALL_OFF();
  void KULBOT_RGB_SET_HUE(uint8_t led ,uint8_t r,uint8_t g,uint8_t b);
  void KULBOT_RGB_AlLSET_HUE(uint8_t r,uint8_t g,uint8_t b);
  /* Control motor  */
   void KULBOT_MOTORENCODER_INIT();
   void KULBOT_MOTORENCODER_RUN1(int motor,uint8_t speed,uint8_t dir);
   void KULBOT_MOTORENCODER_RUN2(int motor,uint8_t speed,uint8_t dir,unsigned long timer);
   void KULBOT_MOTORENCODER_RUN3(int motor,uint8_t speed,uint8_t dir,int degree);
  //  void KULBOT_MOTORENCODER_RUN4(int motor,int dir,float rotation);
   
  /* Control servo  */
  void KULBOT_SERVO_SET_ANGLE(uint8_t servo,uint8_t angle);
  void KULBOT_MOVE_SERVO(int timer, int servo_target[]);
  void KULBOT_SERVO_INIT(void);
  /* initialize sensor  */
  void KULBOT_SENSOR_INIT();
  void KULBOT_DHT_SENSOR_INIT(uint8_t port);
  void KULBOT_LINE_SENSOR_INIT(uint8_t port);
  void KULBOT_IR_SENSOR_INIT(uint8_t port);
  void KULBOT_TOUCH_SENSOR_INIT(uint8_t port);
  void KULBOT_TRAFFIC_LIGHT_INIT(uint8_t port);
  void KULBOT_BUTTON_LED_INIT(uint8_t port);
  void KULBOT_GRYRO_SENSOR_INIT(uint8_t port);
  void KULBOT_COLOR_SENSOR_INIT(uint8_t port);
  void KULBOT_VOLUME_INIT(uint8_t port);
  void KULBOT_SOIL_HUM_SENSOR_INIT(uint8_t port);
  void KULBOT_LIGHT_SENSOR_INTIT(uint8_t port);
  void KULBOT_GAS_SENSOR_INIT(uint8_t port);
  void KULBOT_JOYSTICK_SENSOR_INIT(uint8_t port);
  

  /*  sensor  */
  bool KULBOT_GET_LINE_SENSOR(uint8_t port,int line);
  bool KULBOT_GET_IR_SENSOR(uint8_t port);
  bool KULBOT_GET_TOUCH_SENSOR(uint8_t port);
  bool KULBOT_GET_BUTTON_LED(uint8_t port,int button);
  
  uint8_t  KULBOT_GET_SONAR_SENSOR(uint8_t port);
  uint8_t  KULBOT_GET_TEMP_DHT_SENSOR(uint8_t port);
  uint8_t  KULBOT_GET_HUM_DHT_SENSOR(uint8_t port);
  double   KULBOT_GET_GRYRO_SENSOR(uint8_t port,uint8_t data_get);
  uint8_t  KULBOT_GET_VOLUME_SENSOR(uint8_t port);
  uint8_t  KULBOT_GET_SOIL_HUM_SENSOR(uint8_t port);
  uint8_t  KULBOT_GET_LIGHT_SENSOR(uint8_t port);
  uint8_t  KULBOT_GET_GAS_SENSOR(uint8_t port);
  uint8_t  KULBOT_GET_JOYSTICK_SENSOR(uint8_t port,uint8_t type_get);
  uint16_t KULBOT_GET_COLOR_SENSOR(uint8_t port, uint8_t _data);  
  
  void KULBOT_SET_TRAFFIC_LIGHT(uint8_t port,uint8_t light,bool status);
  void KULBOT_SET_IR_SENSOR_LED(uint8_t port,uint8_t color);
  void KULBOT_SET_BUTTON_LED(uint8_t port,uint8_t color);
  /*  sensor  */

  /* LCD  */
  void KULBOT_LCD_INIT(uint8_t port);
  void KULBOT_LCD_PRINT_STRING(uint8_t port,uint8_t column,uint8_t cell,String data);
  void KULBOT_LCD_PRINT_NUMBER(uint8_t port,uint8_t column,uint8_t cell,long number);
  void KULBOT_LCD_CLEAR(uint8_t port);
  /* LCD  */
  /* SD CARD */
  /*void KULBOT_SD_CARD_INIT();
  void KULBOT_SD_CARD_WRITE(fs::FS &fs, const char * path, const char * data);
  void KULBOT_SD_CARD_READ(fs::FS &fs, const char * path);
  void KULBOT_SD_CARD_APPEN(fs::FS &fs, const char * path, const char * data);
  void KULBOT_SD_CARD_DELETE(fs::FS &fs, const char * path);*/

  /* SD CARD */
  private:
  /* system start  */
  void KULBOT_SYSTEM_START(void);
  void KULBOT_ENABLE_TIMER_SYSTEM(void);
  void KULBOT_WIFI_AP_CONFIG_SYSTEM(void);
  void KULBOT_BLINKY_SYSTEM(void);
  /*  RBG LED selection   */
  uint8_t _LED_CONV(uint8_t led);
  /*  motor  */
  void PWM_MOTORA(uint8_t speed);
  void PWM_MOTORB(uint8_t speed);
  void MOTOR_RUN_FORWARD(int motor);
  void MOTOR_RUN_BACKWARD(int motor);
  void MOTOR_STOP(int motor);
  void MOTOR_BRAKE(int motor);
  void setMotor(int dir, int pwmVal);
  /*  servo  */
  uint8_t _selection_servo_port(uint8_t port_servo);
  int _angleToPulse(uint8_t angle);
   /* read sensor  */
  uint8_t _selection_sensor_port(uint8_t sensor_port);
  uint8_t _read_byte_value_sensor(byte addr);
  void _write_byte_value_sensor(byte addr ,byte byteData);
  /* set Ir led  */
  void IR_LED_RED_CONTROL(bool status);
  void IR_LED_GREEN_CONTROL(bool status);
  void IR_LED_BLUE_CONTROL(bool status);
  void _IR_TURN_ON_RED();
  void _IR_TURN_ON_BLUE();
  void _IR_TURN_ON_GREEN();
  void _IR_TURN_ON_YELLOW();
  void _IR_TURN_ON_CYN();
  void _IR_TURN_ON_VIOLET();
  void _IR_TURN_ON_WHITLE();
 /* set Button led  */
  void BUTTON_LED_RED_CONTROL(bool status);
  void BUTTON_LED_GREEN_CONTROL(bool status);
  void BUTTON_LED_BLUE_CONTROL(bool status);

  void _BUTTON_LED_ON_RED();
  void _BUTTON_LED_ON_BLUE();
  void _BUTTON_LED_ON_GREEN();
  void _BUTTON_LED_ON_YELLOW();
  void _BUTTON_LED_ON_CYN();
  void _BUTTON_LED_ON_VIOLET();
  void _BUTTON_LED_ON_WHITLE();

/* Variable private KulBot  */
/* RGB  */ 
uint8_t COLOR_ARRAY_RED[3]={255,0,0};
uint8_t COLOR_ARRAY_ORANGE[3]={255,127,0};
uint8_t COLOR_ARRAY_YELLOW[3]={255,255,0};
uint8_t COLOR_ARRAY_GREEN[3]={0,255,0};
uint8_t COLOR_ARRAY_BLUE[3]={0,0,255};
uint8_t COLOR_ARRAY_INDIGO[3]={75,0,130};
uint8_t COLOR_ARRAY_VIOLET[3]={143,0,255};
uint8_t COLOR_ARRAY_WHITE[3]={255,255,255};
uint8_t COLOR_ARRAY_BLACK[3]={0,0,0};
uint8_t data_color_num[9]={RGB_COLOR_RED,RGB_COLOR_ORANGE,RGB_COLOR_YELLOW,RGB_COLOR_GREEN,RGB_COLOR_BLUE,RGB_COLOR_INDIGO,RGB_COLOR_VIOLET,RGB_COLOR_WHITE,RGB_COLOR_BLACK};
/* 
 port  */ 
uint8_t _num_servo_port;
/* Sensor port  */ 
uint8_t _port_sensor_selection;
/* Variable Sensor   */ 
 bool _value_line_sensor;
 bool _value_touch_sensor;
 bool _value_ir_sensor;
 uint8_t _value_temp_sensor;
 uint8_t _value_hum_sensor;
 uint8_t _value_sonar_sensor;
 bool _value_button_led = false;
 double _gryro_data_get;
 uint16_t _color_clear,_color_red,_color_green,_color_blue;
 uint16_t  _get_color_read;

 uint8_t _volume_read_value;
 uint8_t _soil_hum_read_value;
 uint8_t _light_read_value;
 uint8_t _gas_read_value;
 uint8_t _joystick_read_value;
};

#endif
