*
   2  * rosserial Subscriber Example
   3  * Blinks an LED on callback
   4  */
   5 
   6 #include <ros.h>
   7 #include <std_msgs/Empty.h>
   8 
   9 ros::NodeHandle nh;
  10 
  11 void messageCb( const std_msgs::Empty& toggle_msg){
  12   digitalWrite(13, HIGH-digitalRead(13));   // blink the led
  13 }
  14 
  15 ros::Subscriber<std_msgs::Empty> sub("toggle_led", &messageCb );
  16 
  17 void setup()
  18 {
  19   pinMode(13, OUTPUT);
  20   nh.initNode();
  21   nh.subscribe(sub);
  22 }
  23 
  24 void loop()
  25 {
  26   nh.spinOnce();
  27   delay(1);
  28 }
