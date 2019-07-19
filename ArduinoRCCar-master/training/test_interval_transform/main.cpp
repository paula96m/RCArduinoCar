#include <stdio.h>

#define STEERING_MIN_POSITION 50  // turning maximum left
#define STEERING_MAX_POSITION 130  // turning maximum right
#define STEERING_CENTER 80
#define STEERING_MIN_POSITION_CMD 0  // turning maximum left
#define STEERING_MAX_POSITION_CMD 100  // turning maximum right
#define STEERING_CENTER_CMD 50

int steering_cmd_to_real(unsigned char steering_cmd);

int main()
{
    unsigned char x;
    for(unsigned char i = 0; i <= 100; i++)
    {
        x = steering_cmd_to_real(i);
        printf("%d -> %d\t", i, x);
        if(i % 8 == 0 && i != 0)
        {
            printf("\n");
        }
    }
    return 0;
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
