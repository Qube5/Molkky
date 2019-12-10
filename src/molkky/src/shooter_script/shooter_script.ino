int shoot_time = 500;
int solenoid_pin = 13;


void setup() {
  // put your setup code here, to run once:
  pinMode(13, OUTPUT);
  digitalWrite(13,LOW);  
  Serial.begin(9600);

}

void loop() {
  if(Serial.available() > 0) {
    if(Serial.read() == 'throw') {
      digitalWrite(solenoid_pin, HIGH); // sets the digital pin 13 on
      delay(shoot_time);            // releases air for set time
      digitalWrite(solenoid_pin, LOW);  // sets the digital pin 13 off
    } else {
      digitalWrite(solenoid_pin, LOW); 
    }
}
