// 실습 과제 2: 4축 로봇암 - PC(robot_server)에서 받은 각도 명령으로 서보 4개 제어
// 지원 보드: Arduino Uno (AVR), ESP32 (보드에 따라 라이브러리/핀 설정이 자동 선택됨)
//
// 프로토콜 (5장): 'a' + base + 'b' + shoulder + 'c' + upperarm + 'd' + forearm + 'e' + '\n'
//   예) "a90b45c120d60e\n"   (각도 0~180)
//
// ---------------------------------------------------------------------------
// 배선 (핀 번호는 아래 "보드별 설정"에서 실제 배선에 맞게 수정)
// ---------------------------------------------------------------------------
//                       Arduino Uno     ESP32 DevKit
//   Base     신호선 ->  D3              GPIO13
//   Shoulder 신호선 ->  D5              GPIO14
//   Upperarm 신호선 ->  D6              GPIO27
//   Forearm  신호선 ->  D9              GPIO26
//
//   서보 전원(빨강)은 보드 5V가 아닌 외부 5V 전원 사용 권장 (서보 4개는 전류가 커서
//   USB 전원만으로는 보드가 리셋되거나 동작이 불안정할 수 있음).
//   외부 전원 GND와 보드 GND는 반드시 연결.
//   ESP32 신호선은 3.3V지만 SG90/MG996R 등 일반 서보는 3.3V 신호로 동작한다.
//
// 필요 라이브러리
//   - Uno  : Servo (Arduino 기본)
//   - ESP32: ESP32Servo (라이브러리 매니저에서 설치)
//
// 시리얼 모니터로 직접 테스트할 때는 9600 baud, 줄 끝을 "새 줄(Newline)"로 설정하고
// a90b45c120d60e 처럼 입력하면 된다.

// ---------------------------------------------------------------------------
// 보드별 설정
// ---------------------------------------------------------------------------
#if defined(ARDUINO_ARCH_AVR)          // Arduino Uno
  #include <Servo.h>
  #define BOARD_NAME    "Arduino Uno"
  #define PIN_BASE      3
  #define PIN_SHOULDER  5
  #define PIN_UPPERARM  6
  #define PIN_FOREARM   9
#elif defined(ARDUINO_ARCH_ESP32)      // ESP32 DevKit (WROOM-32)
  #include <ESP32Servo.h>
  #define BOARD_NAME    "ESP32"
  // 부팅 스트래핑 핀(0, 2, 5, 12, 15), 입력 전용 핀(34~39), 플래시 핀(6~11)은 피함
  // ESP32-S3 / C3 등 다른 칩은 GPIO 구성이 달라서 핀을 다시 골라야 함
  #define PIN_BASE      13
  #define PIN_SHOULDER  14
  #define PIN_UPPERARM  27
  #define PIN_FOREARM   26
#else
  #error "Unsupported board: Arduino Uno 또는 ESP32만 지원"
#endif

#define BAUD_RATE     9600  // PC 서버(robot_server)와 반드시 같아야 함

// 서보 펄스 폭 범위 (0도 ~ 180도). Uno Servo 라이브러리 기본값(544~2400us)으로
// 두 보드를 통일 -> 보드를 바꿔도 같은 각도 명령에 같은 자세가 나온다.
// (ESP32Servo 기본값은 500~2500us라서 지정하지 않으면 각도가 조금 달라짐)
#define SERVO_MIN_US  544
#define SERVO_MAX_US  2400

#define NUM_AXES      4
#define STEP_DELAY_MS 30    // 1도 움직일 때마다 대기 시간. 클수록 천천히 움직임 (30ms -> 10도에 0.3초)
#define MAX_LINE      32

const int   pins[NUM_AXES]  = {PIN_BASE, PIN_SHOULDER, PIN_UPPERARM, PIN_FOREARM};
const char* names[NUM_AXES] = {"Base", "Shoulder", "Upperarm", "Forearm"};

Servo servos[NUM_AXES];
int current[NUM_AXES] = {90, 90, 90, 90};  // 현재 각도
int target[NUM_AXES]  = {90, 90, 90, 90};  // 목표 각도

char line[MAX_LINE];
int  lineLen = 0;
bool overflow = false;
bool moving = false;        // 목표 각도로 이동 중인지 (도착하면 "# reached" 출력)
unsigned long lastStep = 0;

// 현재 각도와 실제로 서보에 나가는 펄스 폭(us)을 한 줄로 출력
//   예) # reached Base=90deg/1472us Shoulder=...
void printAxes(const char* label) {
  Serial.print("# ");
  Serial.print(label);
  for (int i = 0; i < NUM_AXES; i++) {
    Serial.print(' ');
    Serial.print(names[i]);
    Serial.print('=');
    Serial.print(current[i]);
    Serial.print("deg/");
    Serial.print(servos[i].readMicroseconds());
    Serial.print("us");
  }
  Serial.println();
}

// "a90b45c120d60e" 파싱. 형식이 틀리거나 범위 밖이면 false
bool parseCommand(const char* s, int out[NUM_AXES]) {
  const char markers[NUM_AXES] = {'a', 'b', 'c', 'd'};
  const char* p = s;

  for (int i = 0; i < NUM_AXES; i++) {
    if (*p != markers[i]) return false;
    p++;
    if (*p < '0' || *p > '9') return false;   // 숫자가 최소 1자리 있어야 함

    long v = 0;
    int digits = 0;
    while (*p >= '0' && *p <= '9') {
      v = v * 10 + (*p - '0');
      p++;
      if (++digits > 3) return false;
    }
    if (v > 180) return false;
    out[i] = (int)v;
  }

  if (*p != 'e') return false;
  p++;
  return *p == '\0';
}

void handleLine(const char* s) {
  int angles[NUM_AXES];
  if (!parseCommand(s, angles)) {
    Serial.print("# invalid: ");
    Serial.println(s);
    return;
  }

  Serial.print("# target");
  for (int i = 0; i < NUM_AXES; i++) {
    target[i] = angles[i];
    Serial.print(' ');
    Serial.print(names[i]);
    Serial.print('=');
    Serial.print(angles[i]);
  }
  Serial.println();
  moving = true;
}

// 시리얼에서 한 글자씩 받아 '\n'이 오면 한 줄 처리
void readSerial() {
  while (Serial.available() > 0) {
    char c = (char)Serial.read();

    if (c == '\r') continue;          // 윈도우식 줄바꿈 무시

    if (c == '\n') {
      line[lineLen] = '\0';
      if (overflow) {
        Serial.println("# invalid: line too long");
      } else if (lineLen > 0) {
        handleLine(line);
      }
      lineLen = 0;
      overflow = false;
      continue;
    }

    if (lineLen < MAX_LINE - 1) {
      line[lineLen++] = c;
    } else {
      overflow = true;                // 줄바꿈 올 때까지 버림
    }
  }
}

// 목표 각도로 1도씩 천천히 이동 (갑자기 움직이면 로봇암이 흔들리고 서보에 무리가 감)
void updateServos() {
  unsigned long now = millis();
  if (now - lastStep < STEP_DELAY_MS) return;
  lastStep = now;

  bool allReached = true;
  for (int i = 0; i < NUM_AXES; i++) {
    if (current[i] < target[i]) current[i]++;
    else if (current[i] > target[i]) current[i]--;
    else continue;
    servos[i].write(current[i]);
    if (current[i] != target[i]) allReached = false;
  }

  if (moving && allReached) {
    moving = false;
    printAxes("reached");
  }
}

void setup() {
  Serial.begin(BAUD_RATE);

#if defined(ARDUINO_ARCH_ESP32)
  // ESP32: 서보용 PWM 타이머 할당
  ESP32PWM::allocateTimer(0);
  ESP32PWM::allocateTimer(1);
  ESP32PWM::allocateTimer(2);
  ESP32PWM::allocateTimer(3);
#endif

  for (int i = 0; i < NUM_AXES; i++) {
#if defined(ARDUINO_ARCH_ESP32)
    servos[i].setPeriodHertz(50);     // 일반 서보는 50Hz
#endif
    // Uno는 attach 전에 각도를 지정해 두면 처음 붙일 때 튀지 않음 (ESP32는 무시됨)
    servos[i].write(current[i]);
    servos[i].attach(pins[i], SERVO_MIN_US, SERVO_MAX_US);
    servos[i].write(current[i]);
  }

  Serial.println();                     // ESP32 부팅 메시지와 줄을 분리

  // 부팅 시 초기 자세: 축별 핀 번호, 각도, 서보에 나가는 펄스 폭
  for (int i = 0; i < NUM_AXES; i++) {
    Serial.print("# init ");
    Serial.print(names[i]);
    Serial.print(" pin=");
    Serial.print(pins[i]);
    Serial.print(" angle=");
    Serial.print(current[i]);
    Serial.print(" pulse=");
    Serial.print(servos[i].readMicroseconds());
    Serial.println("us");
  }

  Serial.print("# Robot arm ready on ");
  Serial.print(BOARD_NAME);
  Serial.println(" (all axes 90)");
}

void loop() {
  readSerial();
  updateServos();
}
