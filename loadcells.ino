#include "HX711.h"

#define DOUT  3
#define CLK  2

HX711 scale(DOUT, CLK);
HX711 scale2(5,4);

float calibration_factor = -12200; //=100g
float output;
float output2;
float calibration_factor2 = -12200; //=100g


void setup() {
  Serial.begin(9600);



  scale.set_scale();
  scale2.set_scale();
  scale.tare();  //Reset the scale to 0
  scale2.tare();
  long zero_factor = scale.read_average(); //Get a baseline reading
  Serial.print("Zero factor: "); //This can be used to remove the need to tare the scale. Useful in permanent scale projects.
  Serial.println(zero_factor);
}

void loop() {

  scale.set_scale(calibration_factor); //Adjust to this calibration factor
  scale2.set_scale(calibration_factor2); //Adjust to this calibration factor

  Serial.print("Reading: ");
  output=scale.get_units();
  output2=scale2.get_units(), 1;
  Serial.print(output, 4);
  Serial.print("           Reading2: ");
  Serial.print(output2);
  Serial.print("xx");
  Serial.print(calibration_factor);
  Serial.print("yy");
  Serial.println(calibration_factor2);


 if(Serial.available())
  {
    char temp = Serial.read();
    if( temp == 'a')
      calibration_factor += 1;
      if(temp == '+' || temp == 's')
      calibration_factor += 10;
        if(temp == '+' || temp == 'd')
      calibration_factor += 100;
        if(temp == '+' || temp == 'f')
      calibration_factor += 1000;
        if(temp == '+' || temp == 'g')
      calibration_factor += 10000;
     if(temp == '-' || temp == 'z')
      calibration_factor -= 1;
       if(temp == '-' || temp == 'x')
      calibration_factor -= 10;
       if(temp == '-' || temp == 'c')
      calibration_factor -= 100;
       if(temp == '-' || temp == 'v')
      calibration_factor -= 1000;
       if(temp == '-' || temp == 'b')
      calibration_factor -= 10000;

      if( temp == 'q')
      calibration_factor2 += 1;
      if(temp == 'w')
      calibration_factor2 += 10;
        if(temp == 'e')
      calibration_factor2 += 100;
        if( temp == 'r')
      calibration_factor += 1000;
        if(temp == 't')
      calibration_factor += 10000;
     if(temp == 'y')
      calibration_factor2 -= 1;
       if(temp == 'u')
      calibration_factor2 -= 10;
       if(temp == 'i')
      calibration_factor2 -= 100;
       if(temp == 'o')
      calibration_factor2 -= 1000;
       if( temp == 'p')
      calibration_factor2 -= 10000;
      
  }
  delay(500);
}
