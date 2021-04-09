#include <TimerFive.h>
#define ENC1_CHA 2 //INT.0 - Yellow(pin2)
#define ENC1_CHB 3 //INT.1 - Green(pin3)
#define ENC2_CHA 19 //INT.4 - Yellow(pin19)
#define ENC2_CHB 18 //INT.5 - Green(pin18)
#define M1_DIR 4
#define M1_PWM 5
#define M2_DIR 7
#define M2_PWM 6
#define IR1 A0
#define IR2 A1
//for PID control
int e1cnt = 0;
int e1cnt_k = 0, e1cnt_k_1 = 0, d_e1cnt = 0;
float m1speed = 0;
float m1turn = 0;
float m1_ref_spd = 0;
float m1_err_spd = 0;
float m1_err_spd_k_1 = 0;
float m1_derr_spd = 0;
float m1_err_sum = 0;
float m1_ctrl_up = 0;
float m1_ctrl_ui = 0;
float m1_ctrl_ud = 0;
int m1_ctrl_u = 0;
int m1_ipwm_u = 0;

int e2cnt = 0;
int e2cnt_k = 0, e2cnt_k_1 = 0, d_e2cnt = 0;
float m2speed = 0;
float m2turn = 0;
float m2_ref_spd = 0;
float m2_err_spd = 0;
float m2_err_spd_k_1 = 0;
float m2_derr_spd = 0;
float m2_err_sum = 0;
float m2_ctrl_up = 0;
float m2_ctrl_ui = 0;
float m2_ctrl_ud = 0;
int m2_ctrl_u = 0;
int m2_ipwm_u = 0;

bool t5_flag = 0;
float Kp1 = 1.35;
float Ki1 = 0.265;
float Kd1 = 0.1;
float Kp2 = 1.35;
float Ki2 = 0.225;
float Kd2 = 0.08;

int speed_motor = 0;

unsigned int t5_index = 0;
//for serialEvent function
bool u3_rcv_flag = false;
byte u3_rcv_data = 0x30;

void setup() {
  // put your setup code here, to run once:
  pinMode(ENC1_CHA, INPUT_PULLUP);
  pinMode(ENC1_CHB, INPUT_PULLUP);
  pinMode(ENC2_CHA, INPUT_PULLUP);
  pinMode(ENC2_CHB, INPUT_PULLUP);
  pinMode(M1_DIR, OUTPUT); // 모터1 direction
  pinMode(M1_PWM, OUTPUT); // 모터1 PWM 출력
  pinMode(M2_DIR, OUTPUT); // 모터2 direction
  pinMode(M2_PWM, OUTPUT); // 모터2 PWM 출력
  pinMode(IR1, INPUT);
  pinMode(IR2, INPUT);

  //encoder 값을 받기 위한 외부 인터럽트 설정
  attachInterrupt(digitalPinToInterrupt(ENC1_CHA), Enc1chA_ISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC1_CHB), Enc1chB_ISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC2_CHA), Enc2chA_ISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC2_CHB), Enc2chB_ISR, CHANGE);
  //for Serial communication
  Serial.begin(115200);
  Serial3.begin(9600);
  Serial.setTimeout(30);
  //Motor intialize
  digitalWrite(M1_DIR, LOW);                  //LOW --> CCW
  analogWrite(M1_PWM, 0);
  digitalWrite(M2_DIR, LOW);                  //LOW --> CCW
  analogWrite(M2_PWM, 0);
  //Timer5 setup
  Timer5.initialize(10000); //50msec,
  Timer5.attachInterrupt(T5ISR); //T5ISR
}

void loop() {
  if (t5_flag) {
    t5_flag = false;
    t5_index++;
    switch (t5_index) {
      case 0:
        break;
      case 1: //10msec
        server()
        break;
      case 2: //20msec
        if (u3_rcv_flag) {
          u3_rcv_flag = false;
          if (u3_rcv_data == 0x55) { //U
            m1_ref_spd = 25;
            m2_ref_spd = 25;
          }
          else if (u3_rcv_data == 0x52) { //R
            m1_ref_spd = 25;
            m2_ref_spd = 15;
          }
          else if (u3_rcv_data == 0x44) { //D
            m1_ref_spd = 25;
            m2_ref_spd = 25;
          }
          else if (u3_rcv_data == 0x4C) { //L
            m1_ref_spd = 15;
            m2_ref_spd = 25;
          }
          else if (u3_rcv_data == 0x53) { //S
            m1_ref_spd = 0;
            m2_ref_spd = 0;
          }
          break;
        case 3: //30msec
          break;
        case 4: //40msec
          break;
        case 5: //50msec
          t5_index = 0;
          Serial.print(m1speed); Serial.print(",");
          Serial.print(m1turn); Serial.print(",");
          Serial.print(m2speed); Serial.print(",");
          Serial.println(m2turn);
          break;
        default:
          break;
        }


    }






  }
  void go(float mos) {
    mos > 0 ? mos : mos * (-1);    //입력된 mos값을 절대값으로 만든다
    if (t5_flag) {        //50ms마다 제어함수를 갱신한다
      server();       //현재의 속도를 구하는 함수
      Motor1Control(mos * (-1));    // 모터1 제어함수, 모터 1을 cw 방향으로 설정
      Motor2Control(mos);   // 모터2 제어함수, 모터 2를 ccw 방향으로 설정
    }
  }
  void back(float mos) {
    mos > 0 ? mos : mos * (-1);   //입력된 mos값을 절대값으로 만든다
    if (t5_flag) {        //50ms마다 제어함수를 갱신한다
      server();       //현재의 속도를 구하는 함수
      Motor1Control(mos);     //모터1 제어함수, 모터 1을 ccw 방향으로 설정
      Motor2Control(mos * (-1));    //모터2 제어함수, 모터 2를 cw 방향으로 설정
    }
  }
  void server() {
    // Encoder check
    e1cnt_k = e1cnt;  //모터 1 현재 펄스 입력값을 저장
    e2cnt_k = e2cnt;  //모터 2 현재 펄스 입력값을 저장
    d_e1cnt = e1cnt_k - e1cnt_k_1;  //모터 1 펄스 입력값 변화량 = 현재 펄스 입력값 - 과거 펄스 입력값
    d_e2cnt = e2cnt_k - e2cnt_k_1;  //모터 2 펄스 입력값 변화량 = 현재 펄스 입력값 - 과거 펄스 입력값
    // Encoder value to Motor Speed
    m1speed = d_e1cnt * 10 / 11; // 속도 = 변화량 / 시간 * 단위 변환값
    m1turn = (float)e1cnt_k / 1320; // [pulse]/(11*4*30)
    m2speed = d_e2cnt * 10 / 11; // *500/11/50 //이전에는 dt로 나누는 작업을 수행하였으나, 지금은 시간으로 나누는 작업 없이 kd 상수로 통일되었습니다. 원래 나눠주던 50ms를 이렇게 지금 상수로 나눠주는것입니다
    m2turn = (float)e2cnt_k / 1320;// [pulse]/(11*4*30)
    e1cnt_k_1 = e1cnt_k;     //모터 1 과거 펄스 입력값을 계산이 끝난 현재값으로 지정
    e2cnt_k_1 = e2cnt_k;     //모터 2 과거 펄스 입력값을 계산이 끝난 현재값으로 지정
  }

  // Timer5 Interrupt Service Routine
  void T5ISR() {
    t5_flag = true;  //T5 인터럽트가 발동하면 t5_flag의 값을 참으로 만듭니다
  }
  void Motor1Control(float m1_ref_spd) {
    //Error
    m1_err_spd = m1_ref_spd - m1speed;    //현재 에러값 = 목표 속도 - 현재 속도
    m1_derr_spd = m1_err_spd - m1_err_spd_k_1;  //에러값의 변화량 = 현재 에러값 - 과거 에러값
    m1_err_sum = m1_err_sum + m1_err_spd; //에러값의 총합 = 에러값의 총합 + 현재 에러값
    m1_err_spd_k_1 = m1_err_spd;      //과거 에러값 <- 현재 에러값
    //PID-Controller
    m1_ctrl_up = Kp1 * m1_err_spd;      //현재 에러값에 kp상수를 곱한다
    m1_ctrl_ui = Ki1 * m1_err_sum;      //에러값의 총합에 ki상수를 곱한다
    m1_ctrl_ud = Kd1 * m1_derr_spd;     //에러값의 변화량에 kd상수를 곱한다
    m1_ctrl_u = (int)(m1_ctrl_up + m1_ctrl_ud + m1_ctrl_ui);  //구한 p,i,d 값의 총합을 통해 모터를 제어한다
    if (m1_ctrl_u >= 0) {
      digitalWrite(M1_DIR, LOW); //M1_DIR에 LOW값을 주어 모터 1이 ccw방향으로 돌도록 만든다
      if (m1_ctrl_u > 255) m1_ipwm_u = 255; //255 초과시 pwm 최대 입력값인 255로 낮춰준다
      else m1_ipwm_u = (int)m1_ctrl_u;    //결과값을 모터 입력 변수에 할당한다
    }
    else {
      digitalWrite(M1_DIR, HIGH); //M1_DIR에 HIGH값을 주어 모터 1이 cw방향으로 돌도록 만든다
      if (m1_ctrl_u < -255) m1_ipwm_u = 255;  //255 초과시 pwm 최대 입력값인 255로 낮춰준다
      else m1_ipwm_u = (int)m1_ctrl_u * (-1); //결과값의 절대값을 모터 입력 변수에 할당한다
    }
    analogWrite(M1_PWM, m1_ipwm_u);   //motor input
  }
  void Motor2Control(float m2_ref_spd) {
    //Error
    m2_err_spd = m2_ref_spd - m2speed;    //현재 에러값 = 목표 속도 - 현재 속도
    m2_derr_spd = m2_err_spd - m2_err_spd_k_1;  //에러값의 변화량 = 현재 에러값 - 과거 에러값
    m2_err_sum = m2_err_sum + m2_err_spd; //에러값의 총합 = 에러값의 총합 + 현재 에러값
    m2_err_spd_k_1 = m2_err_spd;      //과거 에러값 <- 현재 에러값
    //PID-Controller
    m2_ctrl_up = Kp2 * m2_err_spd;      //현재 에러값에 kp상수를 곱한다
    m2_ctrl_ui = Ki2 * m2_err_sum;      //에러값의 총합에 ki상수를 곱한다
    m2_ctrl_ud = Kd2 * m2_derr_spd;     //에러값의 변화량에 kd상수를 곱한다
    m2_ctrl_u = (int)(m2_ctrl_up + m2_ctrl_ud + m2_ctrl_ui);  //구한 p,i,d 값의 총합을 통해 모터를 제어한다
    if (m2_ctrl_u >= 0) {
      digitalWrite(M2_DIR, LOW); //M2_DIR에 LOW값을 주어 모터 2가 ccw방향으로 돌도록 만든다
      if (m2_ctrl_u > 255) m2_ipwm_u = 255;   //255 초과시 pwm 최대 입력값인 255로 낮춰준다
      else m2_ipwm_u = (int)m2_ctrl_u;      //결과값을 모터 입력 변수에 할당한다
    }
    else {
      digitalWrite(M2_DIR, HIGH);  //M2_DIR에 HIGH값을 주어 모터 2가 cw방향으로 돌도록 만든다
      if (m2_ctrl_u < -255) m2_ipwm_u = 255;    //255 초과시 pwm 최대 입력값인 255로 낮춰준다
      else m2_ipwm_u = (int)m2_ctrl_u * (-1); //결과값의 절대값을 모터 입력 변수에 할당한다
    }
    analogWrite(M2_PWM, m2_ipwm_u);  //motor input
  }


  //for Encoder 1
  void Enc1chA_ISR() {
    if (digitalRead(ENC1_CHA) == HIGH) {
      if (digitalRead(ENC1_CHB) == LOW) e1cnt--;
      //CHA가 HIGH인 상태에서 CHB가 LOW로 내려가면  e1cnt 값 감소
      else e1cnt++;
      //CHA가 HIGH인 상태에서 CHB가 HIGH로 올라가면  e1cnt 값 증가
    }
    else {
      if (digitalRead(ENC1_CHB) == HIGH) e1cnt--;
      //CHA가 LOW인 상태에서 CHB가 HIGH로 올라가면  e1cnt 값 감소
      else e1cnt++;
      //CHA가 LOW인 상태에서 CHB가 LOW로 내려가면  e1cnt 값 증가
    }
  }
  void Enc1chB_ISR() {
    if (digitalRead(ENC1_CHB) == HIGH) {
      if (digitalRead(ENC1_CHA) == HIGH) e1cnt--;
      //CHB가 HIGH인 상태에서 CHA가 HIGH로 올라가면  e1cnt 값 감소
      else e1cnt++;
      //CHB가 HIGH인 상태에서 CHA가 LOW로 내려가면  e1cnt 값 증가
    }
    else {
      if (digitalRead(ENC1_CHA) == LOW) e1cnt--;
      //CHB가 LOW인 상태에서 CHA가 LOW로 내려가면  e1cnt 값 감소
      else e1cnt++;
      //CHB가 LOW인 상태에서 CHA가 HIGH로 올라가면  e1cnt 값 증가

    }
  }
  //for Encdoer 2
  void Enc2chA_ISR() {
    if (digitalRead(ENC2_CHA) == HIGH) {
      if (digitalRead(ENC2_CHB) == LOW) e2cnt--;
      //CHA가 HIGH인 상태에서 CHB가 LOW로 내려가면  e2cnt 값 감소
      else e2cnt++;
      //CHA가 HIGH인 상태에서 CHB가 HIGH로 올라가면  e2cnt 값 증가
    }
    else {
      if (digitalRead(ENC2_CHB) == HIGH) e2cnt--;
      //CHA가 LOW인 상태에서 CHB가 HIGH로 올라가면  e2cnt 값 감소
      else e2cnt++;
      //CHA가 LOW인 상태에서 CHB가 LOW로 내려가면  e2cnt 값 증가
    }
  }

  void Enc2chB_ISR() {
    if (digitalRead(ENC2_CHB) == HIGH) {
      if (digitalRead(ENC2_CHA) == HIGH) e2cnt--;
      //CHB가 HIGH인 상태에서 CHA가 HIGH로 올라가면  e2cnt 값 감소
      else e2cnt++;
      //CHB가 HIGH인 상태에서 CHA가 LOW로 내려가면  e2cnt 값 증가
    }
    else {
      if (digitalRead(ENC2_CHA) == LOW) e2cnt--;
      //CHB가 LOW인 상태에서 CHA가 LOW로 내려가면  e2cnt 값 감소
      else e2cnt++;
      //CHB가 LOW인 상태에서 CHA가 HIGH로 올라가면  e2cnt 값 증가
    }
  }


  void serialEvent3() {
    u3_rcv_flag = true;
    u3_rcv_data = Serial3.read();
  }
