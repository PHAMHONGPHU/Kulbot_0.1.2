/*
The firmware is developed by 3S JSC.
@ Written by HungDang
*/
#include "Arduino.h"
#include "SPIFFS.h"
#include "FS.h"
#define FORMAT_SPIFFS_IF_FAILED true
const char *path_file_config ="/WifiAPCongfig.txt";
String SPIFFS_READFILE(fs::FS &fs, const char * path);
void SPIFFS_WRITEFILE(fs::FS &fs, const char * path, const char * message);
void SPIFFS_INIT()
{
   if(!SPIFFS.begin(FORMAT_SPIFFS_IF_FAILED)){
     // Serial.println("SPIFFS Mount Failed");
      return;
   }
}
String SPIFFS_READFILE(fs::FS &fs, const char * path)
{
  String WifiAP;
  File file = fs.open(path,FILE_READ);
  while(file.available()) WifiAP=file.readStringUntil('#');
  file.close();
  return WifiAP;
}
void SPIFFS_WRITEFILE(fs::FS &fs, const char * path, const char * message)
{
   File file = fs.open(path, FILE_WRITE);
   if(!file) return;
   else file.print(message);
   file.close();
}