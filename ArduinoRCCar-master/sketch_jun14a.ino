// only forward, obst detect

#include <NewPing.h>

#define trigPin 12
#define echoPin 11
float duration, distance;
#define SERIAL_BAUDRATE 9600  // bps

// traction
#define TRACTION_PHASE_PIN 8
#define TRACTION_ENABLE_PIN 6

#define DIRECTION_FORWARD LOW
#define DIRECTION_BACKWARD HIGH


void setup() {
  // put your setup code here, to run once:
    pinMode(TRACTION_PHASE_PIN, OUTPUT);
    pinMode(TRACTION_ENABLE_PIN, OUTPUT);
    
    digitalWrite(TRACTION_PHASE_PIN, LOW);
    digitalWrite(TRACTION_ENABLE_PIN, LOW);

    pinMode(trigPin, OUTPUT);
    pinMode(echoPin, INPUT);


}

void loop() {
  car_forward();
  Obstacle_Detect();
  
}

void Obstacle_Detect()
{
  // Write a pulse to the HC-SR04 Trigger Pin
  
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  
  // Measure the response from the HC-SR04 Echo Pin
 
  duration = pulseIn(echoPin, HIGH);
  
  // Determine distance from duration
  // Use 343 metres per second as speed of sound
  
  distance = (duration / 2) * 0.0343;
  
  // Send results to Serial Monitor

  Serial.print("Distance = ");
//  if (distance >= 400 || distance <= 2) {
//     Serial.println("Out of range");
//  }
//  else {
//    Serial.print(distance);
//    Serial.println(" cm");
//    delay(500);
//  }
    if (distance <= 25) 
    {
      car_stop();
    }
  delay(500);
  
}

void init_serial_interface(void)
{
  Serial.begin(SERIAL_BAUDRATE);  // opens serial port, sets baudrate to 9600 bps
}
void car_stop(void)
{
  digitalWrite(TRACTION_PHASE_PIN, LOW);
  analogWrite(TRACTION_ENABLE_PIN, 0);
}
void car_forward()
{
  digitalWrite(TRACTION_PHASE_PIN, DIRECTION_FORWARD);
  analogWrite(TRACTION_ENABLE_PIN, 140);
}

