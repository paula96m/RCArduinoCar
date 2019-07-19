#include <Servo.h>
Servo steering;

#define SERIAL_BAUDRATE 9600  // bps

// serial commands
#define CMD_STOP 10
#define CMD_DRIVE 11

// steering
#define STEERING_PIN 13

#define STEERING_MIN_POSITION 50  // turning maximum left
#define STEERING_MAX_POSITION 130  // turning maximum right
#define STEERING_CENTER 80
#define STEERING_MIN_POSITION_CMD 0  // turning maximum left
#define STEERING_MAX_POSITION_CMD 100  // turning maximum right
#define STEERING_CENTER_CMD 50

// traction
#define TRACTION_PHASE_PIN 8
#define TRACTION_ENABLE_PIN 6

#define DIRECTION_FORWARD LOW
#define DIRECTION_BACKWARD HIGH


void setup() 
{
    pinMode(TRACTION_PHASE_PIN, OUTPUT);
    pinMode(TRACTION_ENABLE_PIN, OUTPUT);
    pinMode(STEERING_PIN, OUTPUT);
    digitalWrite(TRACTION_PHASE_PIN, LOW);
    digitalWrite(TRACTION_ENABLE_PIN, LOW);
    digitalWrite(STEERING_PIN, LOW);

    steering.attach(STEERING_PIN);
    center_steering();
  
    init_serial_interface();
    clear_serial_buffer();
}

void loop() 
{
  listen_serial();
  delay(50);
}

void init_serial_interface(void)
{
  Serial.begin(SERIAL_BAUDRATE);  // opens serial port, sets baudrate to 9600 bps
}

void clear_serial_buffer(void)
{
  while(Serial.available() > 0)
  {
    Serial.read();
  }
}

void listen_serial(void)
{
  if(Serial.available() != 3)
  {
    clear_serial_buffer();
    return;
  }
  else
  {
    unsigned char cmd = Serial.read();
    unsigned char data_0 = Serial.read();
    unsigned char data_1 = Serial.read();
    execute_command(cmd, data_0, data_1);
    return;
  }
}

void execute_command(unsigned char cmd, unsigned char data_0, unsigned char data_1)
{
  switch(cmd)
  {
    case CMD_STOP:
    {
      car_stop();
      break;
    }
    case CMD_DRIVE:
    {
      car_drive_cmd(data_0, data_1);
      break;
    }
  }
}

void car_drive_cmd(unsigned char traction_speed, unsigned char steering_position)
{
  if(traction_speed == 0)
  {
    car_stop();
    return;
  }

  if(steering_position == STEERING_CENTER_CMD)
  {
    center_steering();
  }
  else
  {
    unsigned char steering_real_position = steering_cmd_to_real(steering_position);
    car_steer(steering_real_position);
  }
  
  unsigned char traction_direction = traction_speed & 0x01;
  if(traction_direction == DIRECTION_FORWARD)
  {
    car_forward(traction_speed);
  }
  else
  {
    car_backward(traction_speed);
  }
}

void car_stop(void)
{
  digitalWrite(TRACTION_PHASE_PIN, LOW);
  analogWrite(TRACTION_ENABLE_PIN, 0);
}

void car_forward(unsigned char speed)
{
  digitalWrite(TRACTION_PHASE_PIN, DIRECTION_FORWARD);
  analogWrite(TRACTION_ENABLE_PIN, speed);
}

void car_backward(unsigned char speed)
{
  digitalWrite(TRACTION_PHASE_PIN, DIRECTION_BACKWARD);
  analogWrite(TRACTION_ENABLE_PIN, speed);
}

void center_steering(void)
{
  steering.write(STEERING_CENTER);
}

void car_steer(unsigned char position)
{
  if(position < STEERING_MIN_POSITION)
  {
    steering.write(STEERING_MIN_POSITION);
    return;
  }
  else if (position > STEERING_MAX_POSITION)
  {
    steering.write(STEERING_MAX_POSITION);
    return;
  }
  else
  {
    steering.write(position);
    return;
  }
}

int steering_cmd_to_real(unsigned char steering_cmd)
{
  // [A, B] --> [a, b]
  int value = steering_cmd;
  int A, B, a, b;
  int real_value;
  if(value <= STEERING_CENTER_CMD)
  {
    A = STEERING_MIN_POSITION_CMD;
    B = STEERING_CENTER_CMD;
    a = STEERING_MIN_POSITION;
    b = STEERING_CENTER;
  }
  else
  {
    A = STEERING_CENTER_CMD;
    B = STEERING_MAX_POSITION_CMD;
    a = STEERING_CENTER;
    b = STEERING_MAX_POSITION;
  }
  real_value = (value - A) * (b - a) / (B - A) + a;
  return real_value;
}
