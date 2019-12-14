int shoot_time = 1500;
int solenoid_pin = 12;

void setup() {
  // put your setup code here, to run once:

  pinMode(12, OUTPUT);
  digitalWrite(12,LOW);

  Serial.begin(9600);
  Serial.write("start\n");

  pinMode(LED_BUILTIN, OUTPUT);
}

void loop() {
  //Serial.write("loop\n");

  if(Serial.available() > 0) {
    Serial.write("got serial\n");
    char inChar = Serial.read();
//    if (isDigit(inChar)) {
//      // convert the incoming byte to a char and add it to the string:
//      inString += (char)inChar;
//    }
    Serial.print(inChar);


    if(inChar == 't') {
    //if(Serial.read() == "throw") {
      Serial.write("Thrown\n");

      digitalWrite(LED_BUILTIN, HIGH);   // turn the LED on (HIGH is the voltage level)
      delay(1000);                       // wait for a second
      digitalWrite(LED_BUILTIN, LOW);    // turn the LED off by making the voltage LOW
      delay(1000);                       // wait for a second

      digitalWrite(solenoid_pin, HIGH); // sets the digital pin 13 on
      delay(shoot_time);            // releases air for set time
      digitalWrite(solenoid_pin, LOW);  // sets the digital pin 13 off
      delay(shoot_time);            // releases air for set time
    } else {
      digitalWrite(solenoid_pin, LOW);
    }
  }
//  digitalWrite(solenoid_pin, HIGH); // sets the digital pin 13 on
//  delay(shoot_time);            // releases air for set time
//  digitalWrite(solenoid_pin, LOW);  // sets the digital pin 13 off
//  delay(shoot_time);            // releases air for set time
}
