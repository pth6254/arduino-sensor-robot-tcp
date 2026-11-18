// 통합 스케치: DHT11 온도 센서 + 4축 로봇암을 보드 하나에서 동시에 처리
// PC 쪽 짝: integrated/integrated_server.cpp
// 지원 보드: Arduino Uno (AVR), ESP32 (보드에 따라 라이브러리/핀 설정이 자동 선택됨)
//
// ---------------------------------------------------------------------------
// 시리얼 프로토콜 (9600 baud)
// ---------------------------------------------------------------------------
//   PC -> 보드 : 로봇암 명령  "a<base>b<shoulder>c<upperarm>d<forearm>e\n"  예) a90b45c120d60e
//   보드 -> PC : 온도        줄 전체가 숫자인 한 줄                         예) 23.0
//               상태 메시지  '#'으로 시작하는 한 줄                         예) # reached Base=...
//
// ---------------------------------------------------------------------------
// 배선
// ---------------------------------------------------------------------------
//                       Arduino Uno     ESP32 DevKit
//   DHT11 DATA      ->  D2              GPIO4
//   DHT11 VCC       ->  5V              3V3   (ESP32는 3.3V 로직)
//   Base     신호선 ->  D3              GPIO13
//   Shoulder 신호선 ->  D5              GPIO14
//   Upperarm 신호선 ->  D6              GPIO27
//   Forearm  신호선 ->  D9              GPIO26
//   서보 전원은 외부 5V 권장, GND는 보드와 공통 연결
//
// 필요 라이브러리
//   - DHT sensor library (Adafruit), Adafruit Unified Sensor
//   - Uno: Servo (기본) / ESP32: ESP32Servo

#include <DHT.h>

// ---------------------------------------------------------------------------
// 보드별 설정
// ---------------------------------------------------------------------------
#if defined(ARDUINO_ARCH_AVR)          // Arduino Uno
  #include <Servo.h>
  #define BOARD_NAME    "Arduino Uno"
  #define DHT_PIN       2
  #define PIN_BASE      3
  #define PIN_SHOULDER  5
  #define PIN_UPPERARM  6
  #define PIN_FOREARM   9
#elif defined(ARDUINO_ARCH_ESP32)      // ESP32 DevKit (WROOM-32)
  #include <ESP32Servo.h>
  #define BOARD_NAME    "ESP32"
  // 부팅 스트래핑 핀(0, 2, 5, 12, 15), 입력 전용 핀(34~39), 플래시 핀(6~11)은 피함
  #define DHT_PIN       4
  #define PIN_BASE      13
  #define PIN_SHOULDER  14
  #define PIN_UPPERARM  27
  #define PIN_FOREARM   26
#else
  #error "Unsupported board: Arduino Uno 또는 ESP32만 지원"
#endif

#define BAUD_RATE     9600  // PC 서버와 반드시 같아야 함

// --- 온도 센서 ---
#define DHT_TYPE          DHT11
#define TEMP_INTERVAL_MS  2000   // DHT11은 1초에 1번보다 자주 읽으면 안 됨
#define TEMP_MAX_WAIT_MS  6000   // 로봇암이 계속 움직여도 이 시간이 지나면 읽음

// --- 로봇암 ---
// 서보 펄스 폭 범위 (0도 ~ 180도). 두 보드에서 같은 각도 = 같은 자세가 되도록 통일
#define SERVO_MIN_US  544
#define SERVO_MAX_US  2400

#define NUM_AXES      4
#define STEP_DELAY_MS 30    // 1도 움직일 때마다 대기 시간. 클수록 천천히 움직임
#define MAX_LINE      32

const int   pins[NUM_AXES]  = {PIN_BASE, PIN_SHOULDER, PIN_UPPERARM, PIN_FOREARM};
const char* names[NUM_AXES] = {"Base", "Shoulder", "Upperarm", "Forearm"};

DHT   dht(DHT_PIN, DHT_TYPE);
Servo servos[NUM_AXES];
int current[NUM_AXES] = {90, 90, 90, 90};  // 현재 각도
int target[NUM_AXES]  = {90, 90, 90, 90};  // 목표 각도

char line[MAX_LINE];
int  lineLen = 0;
bool overflow = false;
bool moving = false;        // 목표 각도로 이동 중인지 (도착하면 "# reached" 출력)
unsigned long lastStep = 0;
unsigned long lastTemp = 0;

// ---------------------------------------------------------------------------
// 로봇암
// ---------------------------------------------------------------------------
// 현재 각도와 실제로 서보에 나가는 펄스 폭(us)을 한 줄로 출력
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

// ---------------------------------------------------------------------------
// 온도 센서
// ---------------------------------------------------------------------------
// DHT11을 읽는 약 25ms 동안은 서보 펄스 타이밍이 흔들릴 수 있다.
// 그래서 로봇암이 움직이는 중에는 읽기를 미루고, 멈춰 있을 때 읽는다.
void updateTemperature() {
  unsigned long now = millis();
  unsigned long elapsed = now - lastTemp;
  if (elapsed < TEMP_INTERVAL_MS) return;
  if (moving && elapsed < TEMP_MAX_WAIT_MS) return;
  lastTemp = now;

  float t = dht.readTemperature();  // 섭씨

  // 읽기 실패 시 NaN. "nan"을 그대로 보내지 않고 '#' 메시지로 보낸다.
  if (isnan(t)) {
    Serial.println("# DHT11 read failed (check wiring)");
    return;
  }
  Serial.println(t, 1);  // 온도 줄: 숫자만, 예) 23.0
}

// ---------------------------------------------------------------------------
void setup() {
  Serial.begin(BAUD_RATE);
  dht.begin();

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
  Serial.print("# init DHT11 pin=");
  Serial.println(DHT_PIN);

  Serial.print("# Integrated (sensor + robot arm) ready on ");
  Serial.print(BOARD_NAME);
  Serial.println(" (all axes 90)");
}

void loop() {
  readSerial();
  updateServos();
  updateTemperature();
}
