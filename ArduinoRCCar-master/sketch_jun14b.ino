//buzzer

int buzzerPin = 9; //Define buzzerPin

void setup() {
  pinMode(buzzerPin, OUTPUT); //Set buzzerPin as output
  

}

void loop() {
  beep(20); //Beep every 400 milliseconds
//  delay(400);
}

void beep(unsigned char delayms) { //creating function
  analogWrite(buzzerPin, 20); //Setting pin to high
  delay(delayms); //Delaying
  analogWrite(buzzerPin ,0); //Setting pin to LOW
  delay(delayms); //Delaying
  
}
