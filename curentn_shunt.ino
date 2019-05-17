#include <Adafruit_ADS1015.h>
#include <Wire.h>
Adafruit_ADS1115 ads;
double multi = 0.0078125F;
double impliedMv;
double ampsPerMv = 4;
void setup() {
  Serial.begin(115200);

  ads.setGain(GAIN_SIXTEEN);
  ads.begin();

}

void loop() {
 int16_t result;
 result = ads.readADC_Differential_0_1();
 impliedMv =result * multi;
 Serial.print("differential:");
 Serial.println(-result);
 Serial.print("(");
 Serial.print(impliedMv);
 Serial.print(") Amps:");
 Serial.println(-impliedMv * ampsPerMv);
 delay(500);
}
