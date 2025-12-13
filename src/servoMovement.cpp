#include <servoPins.h>
#include <servoMovement.h>

double translator(double angle) {
  double temp = minPulse + (angle / 180.0) * (maxPulse - minPulse);
  return temp * 4096 / (1000000 / 50); // Convert to PWM ticks
}

void UpdatePosition(int leg, double A1, double A2, double A3) {
  
  // translate leg angles into servo angles
  A1 = translator(180 - A1);
  A2 = translator(180 - A2);
  A3 = translator(180 - A3);
  //Serial.println(leg);
  switch (leg) {
    case  0:
      pwm1.setPWM(SERVO_PINFR1, 0, A1);
      pwm1.setPWM(SERVO_PINFR2, 0, A2);
      pwm1.setPWM(SERVO_PINFR3, 0, A3);
      //Serial.println("leg 0");
      break;
    case  1:
      pwm1.setPWM(SERVO_PINCR1, 0, A1);
      pwm1.setPWM(SERVO_PINCR2, 0, A2);
      pwm1.setPWM(SERVO_PINCR3, 0, A3);
      //Serial.println("leg 1");
      break;
    case 2:
      pwm1.setPWM(SERVO_PINBR1, 0, A1);
      pwm1.setPWM(SERVO_PINBR2, 0, A2);
      pwm1.setPWM(SERVO_PINBR3, 0, A3);
      //Serial.println("leg 2");
      break;
    case 3:
      pwm2.setPWM(SERVO_PINFL1, 0, A1);
      pwm2.setPWM(SERVO_PINFL2, 0, A2);
      pwm2.setPWM(SERVO_PINFL3, 0, A3);
      //Serial.println("leg 3");
      break;
    case 4:
      pwm2.setPWM(SERVO_PINCL1, 0, A1);
      pwm2.setPWM(SERVO_PINCL2, 0, A2);
      pwm2.setPWM(SERVO_PINCL3, 0, A3);
      ///Serial.println("leg 4");
      break;
    case 5:
      pwm2.setPWM(SERVO_PINBL1, 0, A1);
      pwm2.setPWM(SERVO_PINBL2, 0, A2);
      pwm2.setPWM(SERVO_PINBL3, 0, A3);
      //Serial.println("leg 5");
      break;
    default:
      break;
  } 
}