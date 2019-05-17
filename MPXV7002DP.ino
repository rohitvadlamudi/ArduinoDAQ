#include <Wire.h>
#include <Adafruit_ADS1015.h>
Adafruit_ADS1115 ads;     
float multi = 0.015625;// 
float velocity1;
float velocity2;
int velocity3;
int adc2;
float velocity;
void setup() 
{
  Serial.begin(115200);
  Serial.println("Hello!");
  ads.setGain(GAIN_EIGHT);    // 16x gain  +/- 0.256V  1 bit = 0.125mV  0.0078125mV
  ads.begin();
}

void loop() 
{
  
  adc2 = ads.readADC_SingleEnded(2);
  velocity1 = (adc2-32767);
  velocity2 = - velocity1;
  velocity3 = adc2 * multi;
  velocity = sqrt ( 2 * velocity3/1.225);
  Serial.print("adc2:");
  Serial.println(adc2);
  Serial.println(velocity);
  
  delay(1000);
}
