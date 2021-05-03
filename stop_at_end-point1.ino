
// put this code just after the variable set in the void loop()
void loop() {
  // put your main code here, to run repeatedly:
 int sum_of_line_sensor = 0;
 
for (int i = 0; i < 7; i++)
  {

    //Serial.println(lineSensor[i]);
    sum_of_line_sensor= sum_of_line_sensor + buggy.lineSensor.GetSensorValues(i);
     
  }
while (sum_of_line_sensor > 17000)
//the sum of 7 sensor reading is 17500 in maximum
{
sum_of_line_sensor = 0;
for (int i = 0; i < 7; i++)
  {

    //Serial.println(lineSensor[i]);
    sum_of_line_sensor= sum_of_line_sensor + buggy.lineSensor.GetSensorValues(i);
   
  }
wr_set = wr_set * 0.97 ;
 wl_set = wl_set * 0.97 ;
  buggy.right_motor.setAngularSpeed(wr_set); //max speed = ~14.93 rads/s
  buggy.left_motor.setAngularSpeed(wl_set);
  delay(20);
  }
}
