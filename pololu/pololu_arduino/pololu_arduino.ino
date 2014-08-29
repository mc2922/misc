String inData = "";
String inDataSingle = "";
unsigned int servo0, servo1, servo2, servo3, servo4, servo5;
unsigned int count = 0;

void setup() {
  Serial.begin(9600);
  Serial1.begin(9600);
}

void loop() {
  while (Serial.available() > 0)
  {
    char received = Serial.read();
    inData += received; 

    if (received == ',') {
      Serial.print("Val Received: ");
      Serial.println(inDataSingle);
      setServo(inDataSingle);
      inDataSingle = "";
      count++;
    } else {
      inDataSingle += received;
    }

    // Process message when new line character is recieved
    if (received == '\n') {
      Serial.print("Final Val Received: ");
      Serial.println(inDataSingle);
      setServo(inDataSingle);
      Serial.print("Line Received: ");
      Serial.print(inData);
      inData = ""; // Clear recieved buffer
      inDataSingle = "";
      count = 0;
    }
  }
}

void setServo(String angle) {
  if (count == 0) {
    servo0 = angle.toInt();
    pushServo(0, servo0);
  } else if (count == 1) {
    servo1 = angle.toInt();
    pushServo(1, servo1);
  } else if (count == 2) {
    servo2 = angle.toInt();
    pushServo(2, servo2);
  } else if (count == 3) {
    servo3 = angle.toInt();
    pushServo(3, servo3);
  } else if (count == 4) {
    servo4 = angle.toInt();
    pushServo(4, servo4);
  } else {
    servo5 = angle.toInt();
    pushServo(5, servo5);
  }
}

void pushServo(unsigned int servo, unsigned int angle) {
  //servo is the servo number (typically 0-7)
  //angle is the absolute position from 500 to 5500
  //Send a Pololu Protocol command
  Serial1.write(0x80);               //start byte
  Serial.print(0x80,HEX);
  Serial1.write(0x01);               //device id
  Serial.print(0x01,HEX);
  Serial1.write(0x04);               //command number
  Serial.print(0x04,HEX);
  Serial1.write(servo);              //servo number
  Serial.print(servo,DEC);
  //Convert the angle data into two 7-bit bytes
  Serial1.write(((angle>>7)&0x3f));  //data1
  Serial.print(((angle>>7)&0x3f),HEX);
  Serial1.write((angle&0x7f));       //data2
  Serial.print((angle&0x7f),HEX);
}
