//String inData;
int incomingByte;

void setup()// run once, when the sketch starts
{
  Serial.begin(9600);   //serial for interface between arduino and lubuntu
  //Serial1.begin(9600);  //serial for sending commands to pololu micro serial controller
}
 
void loop() //Main program loop, executes forever after setup()
{
  //if (Serial.available() > 0) {
  //  incomingByte = Serial.read();
  //  Serial.print("received char: ");
  //  Serial.println(incomingByte, DEC);
    //inData += received;
    //if (received == '\n') {
    //  Serial.print("Arduino Received: ");
    //  Serial.print(inData);
    //  inData = "";
    //}
  //}
  Serial.println("TEST");
  put(7, 500);
  delay(2000);
  put(7,5500);
  delay(2000);
}
 
/*
 Move Servo
*/
void put(int servo, int angle)
{
  //servo is the servo number (typically 0-7)
  //angle is the absolute position from 500 to 5500
  Serial1.write(0xAA);
  Serial1.write(0x0C);
  Serial1.write(0x04);
  Serial1.write(servo);
  //convert the angle data into two 7-bit bytes
  Serial1.write(((angle>>7)&0x3f));
  Serial1.write((angle&0x7f));
}
