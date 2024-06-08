//test to check micro vibrator motor works before implementing it into my main project code
//transister pin bas defined here
const int motorPin = 4;

void setup() {
  //motor pin = output
  pinMode(motorPin, OUTPUT);
}

void loop() {
  //motor on
  digitalWrite(motorPin, HIGH);
  delay(1000); 
}
