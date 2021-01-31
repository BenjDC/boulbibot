#include <M5Stack.h>
//#include <M5StackUpdater.h>
#include <Avatar.h>
#include <Arduino.h>
#include <ESP8266SAM.h>  //https://github.com/earlephilhower/ESP8266SAM
#include <AudioOutputI2S.h>  //https://github.com/earlephilhower/ESP8266Audio
#include <tasks/LipSync.h>

using namespace m5avatar;

Avatar avatar;

AudioOutputI2S *out = NULL;
ESP8266SAM *sam = NULL;


void setup()
{
  M5.begin();
  Wire.begin();
  Serial.begin(115200);

  /*
  if(digitalRead(BUTTON_A_PIN) == 0){
    Serial.println("Will load menu binary");
    //updateFromFS(SD);
    ESP.restart();
  } 
  */ 
  out = new AudioOutputI2S(0, 1, 32);
  sam = new ESP8266SAM;
  
  avatar.init();
  avatar.addTask(lipSync, "lipSync");
  audioLogger = &Serial;
  
}

void loop()
{
  
  out->begin();
  M5.update();

  delay(2000);

  avatar.setTalking(true);  
  sam->Say(out, "stop now !");
  avatar.setTalking(false);
  
  delay(1000);

  avatar.setTalking(true);  
  sam->Say(out, "or I will shoot !");
  avatar.setTalking(false);

  
  
  
  
  out->stop();

  delay(1000000);
  
}