// 실습 과제 1: DHT11 온도 센서 -> PC(sensor_server)로 시리얼 전송
// 지원 보드: Arduino Uno (AVR), ESP32 (보드에 따라 핀 설정이 자동 선택됨)
//
// ---------------------------------------------------------------------------
// 배선 (DHT11 모듈 3핀 기준)
// ---------------------------------------------------------------------------
//               Arduino Uno        ESP32 DevKit
//   VCC    ->   5V                 3V3   (ESP32는 3.3V 로직: 5V로 주면 DATA가 5V가 되어 위험)
//   DATA   ->   D2                 GPIO4
//   GND    ->   GND                GND
//   4핀 단품 센서면 DATA-VCC 사이에 10k 풀업 저항 필요 (모듈은 내장)
//
// 필요 라이브러리 (라이브러리 매니저에서 설치)
//   - DHT sensor library (Adafruit)
//   - Adafruit Unified Sensor
//
// 출력 형식: 온도(℃) 값을 한 줄에 하나씩 숫자로만 출력. 예) "23.0"
//   sensor_server는 "줄 전체가 숫자"인 경우만 온도로 인식한다.
//   상태 메시지는 '#'으로 시작하게 해서 온도로 오인되지 않게 한다.

#include <DHT.h>

// ---------------------------------------------------------------------------
// 보드별 설정
// ---------------------------------------------------------------------------
#if defined(ARDUINO_ARCH_AVR)          // Arduino Uno
  #define BOARD_NAME "Arduino Uno"
  #define DHT_PIN    2
#elif defined(ARDUINO_ARCH_ESP32)      // ESP32 DevKit (WROOM-32)
  #define BOARD_NAME "ESP32"
  #define DHT_PIN    4                 // 부팅 스트래핑 핀(0, 2, 5, 12, 15)은 피함
#else
  #error "Unsupported board: Arduino Uno 또는 ESP32만 지원"
#endif

#define DHT_TYPE     DHT11
#define BAUD_RATE    9600   // PC 서버(sensor_server)와 반드시 같아야 함
#define INTERVAL_MS  2000   // DHT11은 1초에 1번보다 자주 읽으면 안 됨 (여유 있게 2초)

DHT dht(DHT_PIN, DHT_TYPE);
unsigned long lastRead = 0;

void setup() {
  Serial.begin(BAUD_RATE);
  dht.begin();
  Serial.println();                     // ESP32 부팅 메시지와 줄을 분리
  Serial.print("# DHT11 sensor ready on ");
  Serial.println(BOARD_NAME);
}

void loop() {
  unsigned long now = millis();
  if (now - lastRead < INTERVAL_MS) return;
  lastRead = now;

  float t = dht.readTemperature();  // 섭씨

  // 읽기 실패 시 NaN이 반환됨. "nan"을 그대로 보내지 않고 '#' 메시지로 보낸다.
  if (isnan(t)) {
    Serial.println("# DHT11 read failed (check wiring)");
    return;
  }

  Serial.println(t, 1);  // 소수점 1자리, 예) 23.0
}
