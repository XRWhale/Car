///////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////
// Mechanum Wheel 을 테스트 합니다.    < XIAO_mechanum.ino >               *
//                                                                        *
// 전진, 후진, 제자리 우회전, 제자리 좌회전, 평행 우방향, 평행 좌 방향       *
// 우상, 우하, 좌하, 좌상 방향으로 이동합니다.                              *
//                                                                        *
//                  +-------------------------+                           *
//      ^    /---\  |                         |  /---\    ^               *
//      F    |   |  |            ^            |  |   |    F               *
//           | A |  |           /|\           |  | B |                    *
//      B    |   |  |          / | \          |  |   |    B               *
//      v    \---/  |            |            |  \---/    v               *
//                  |            |            |                           *
//                  |            |            |                           *
//                  |                         |                           *
//      ^    /---\  |                         |  /---\    ^               *
//      F    |   |  |                         |  |   |    F               *
//           | D |  |                         |  | C |                    *
//      B    |   |  |                         |  |   |    B               *
//      v    \---/  |                         |  \---/    v               *
//                  +-------------------------+                           *
//                                                                        *
// (9 8)  (7 44) (6 5)  (4 3) (2 1) XIAO ESP32 S3                         *                                                                        *
//                                                                        *
//                                                                        *
//                             XIAO ESP32 S3                              *
//  H:B L:F  D-DIR    GPIO1                     5V                        *
//           D-PWM    GPIO2                     GND                       *
//           C-PWM    GPIO3                     3V3                       *
//           C-DIR    GPIO4                     GPIO9    B-DIR  H:F L:B   *
//           B-DIR    GPIO5                     GPIO8    B-PWM            *
//           B-PWM    GPIO6                     GPIO7    A-PWM            *
//                    GPIO43                    GPIO44   A-DIR  H:F L:B   *
//                               TOP VIEW                                 *
// DIR  F: FORWARD                                                        *
//      B: BACKWARD                                                       *
//                                                                        *
// PWM  0:STOP                                                            *
//      255:MAX                                                           *
//                                                                        *
//                                                                        *
///////////////////////////////////////////////////////////////////////////

#define MOTOR_A_P    7   
#define MOTOR_A_D    44 

#define MOTOR_B_P    8  
#define MOTOR_B_D    9 

#define MOTOR_C_P    3 
#define MOTOR_C_D    4 

#define MOTOR_D_P    2
#define MOTOR_D_D    1 

///////////////////////////////////////////////////////////////////////////

void setup() {

  delay(300);

  pinMode(MOTOR_A_D, OUTPUT);
  pinMode(MOTOR_B_D, OUTPUT);
  pinMode(MOTOR_C_D, OUTPUT);
  pinMode(MOTOR_D_D, OUTPUT);

  digitalWrite(MOTOR_A_D, LOW);
  digitalWrite(MOTOR_B_D, LOW);
  digitalWrite(MOTOR_C_D, LOW);
  digitalWrite(MOTOR_D_D, LOW);

  analogWrite(MOTOR_A_P, 0);
  analogWrite(MOTOR_B_P, 0);
  analogWrite(MOTOR_C_P, 0);
  analogWrite(MOTOR_D_P, 0);

  delay(300);

}

void forward(unsigned int speed){

    digitalWrite(MOTOR_A_D, HIGH);       // 전진
    digitalWrite(MOTOR_B_D, HIGH);
    digitalWrite(MOTOR_C_D, LOW);
    digitalWrite(MOTOR_D_D, LOW);

    analogWrite(MOTOR_A_P, speed);
    analogWrite(MOTOR_B_P, speed);
    analogWrite(MOTOR_C_P, speed);
    analogWrite(MOTOR_D_P, speed);

}

void backward(unsigned int speed) {

    digitalWrite(MOTOR_A_D, LOW);        // 후진
    digitalWrite(MOTOR_B_D, LOW);
    digitalWrite(MOTOR_C_D, HIGH);
    digitalWrite(MOTOR_D_D, HIGH);

    analogWrite(MOTOR_A_P, speed);
    analogWrite(MOTOR_B_P, speed);
    analogWrite(MOTOR_C_P, speed);
    analogWrite(MOTOR_D_P, speed);

}

void stop() {                            // 정지

    analogWrite(MOTOR_A_P, 0);
    analogWrite(MOTOR_B_P, 0);
    analogWrite(MOTOR_C_P, 0);
    analogWrite(MOTOR_D_P, 0);

}

void t_l(unsigned int speed){            // Turn Left

    digitalWrite(MOTOR_A_D, LOW);        // 제자리 좌회전
    digitalWrite(MOTOR_B_D, HIGH);
    digitalWrite(MOTOR_C_D, LOW);
    digitalWrite(MOTOR_D_D, HIGH);

    analogWrite(MOTOR_A_P, speed);
    analogWrite(MOTOR_B_P, speed);
    analogWrite(MOTOR_C_P, speed);
    analogWrite(MOTOR_D_P, speed);
}

void t_r(unsigned int speed){             // Turn Right

    digitalWrite(MOTOR_A_D, HIGH);        // 제자리 우회전
    digitalWrite(MOTOR_B_D, LOW);
    digitalWrite(MOTOR_C_D, HIGH);
    digitalWrite(MOTOR_D_D, LOW);

    analogWrite(MOTOR_A_P, speed);
    analogWrite(MOTOR_B_P, speed);
    analogWrite(MOTOR_C_P, speed);
    analogWrite(MOTOR_D_P, speed);
}

void s_r(unsigned int speed){             // Parallel Shift Right
    digitalWrite(MOTOR_A_D, HIGH);        // 우 방향 평행이동
    digitalWrite(MOTOR_B_D, LOW);
    digitalWrite(MOTOR_C_D, LOW);
    digitalWrite(MOTOR_D_D, HIGH);

    analogWrite(MOTOR_A_P, speed);
    analogWrite(MOTOR_B_P, speed);
    analogWrite(MOTOR_C_P, speed);
    analogWrite(MOTOR_D_P, speed);
}

void s_l(unsigned int speed){             // Parallel Shift Left
    digitalWrite(MOTOR_A_D, LOW);         // 좌 방향 평행이동
    digitalWrite(MOTOR_B_D, HIGH);
    digitalWrite(MOTOR_C_D, HIGH);
    digitalWrite(MOTOR_D_D, LOW);

    analogWrite(MOTOR_A_P, speed);
    analogWrite(MOTOR_B_P, speed);
    analogWrite(MOTOR_C_P, speed);
    analogWrite(MOTOR_D_P, speed);
}

///////////////////////////////////////////////////////////////////////////
void loop() {
//

while (HIGH) {

    forward(250);
    delay(700);

    backward(250);
    delay(700);

    s_l(250);
    delay(700);

    s_r(250);
    delay(700);

    stop();

  }
}

///////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////
