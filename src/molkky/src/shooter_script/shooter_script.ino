int shoot_time = 500;
int solenoid_pin = 13;

void setup() {
  // put your setup code here, to run once:

  pinMode(13, OUTPUT);
  digitalWrite(13,LOW);  
  Serial.begin(9600);
  Serial.write("start\n");
  digitalWrite(solenoid_pin, HIGH); // sets the digital pin 13 on
  delay(shoot_time);            // releases air for set time
  digitalWrite(solenoid_pin, LOW);  // sets the digital pin 13 off
  pinMode(LED_BUILTIN, OUTPUT);
}

void loop() {
  //Serial.write("loop\n");
  
  if(Serial.available() > 0) {
    if(Serial.read() == 1) {
    //if(Serial.read() == "throw") {
      Serial.write("Thrown");
      
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
}
