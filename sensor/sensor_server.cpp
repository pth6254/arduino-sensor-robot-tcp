// 실습 과제 1: 아두이노 센서 데이터 모니터링 서버 (Non-blocking)
//
// 사용법:
//   ./sensor_server                 -> /dev/ttyUSB0, /dev/ttyACM0 순서로 보드를 찾아 온도 읽기 (9600 baud)
//                                      (Arduino Uno / ESP32 모두 지원, 못 찾으면 가상 데이터로 자동 전환)
//   ./sensor_server /dev/ttyACM0    -> 다른 시리얼 포트 지정
//   ./sensor_server dummy           -> 아두이노 없이 가상 온도 데이터 사용
//
// 아두이노 쪽은 온도 값을 한 줄씩 보내면 됩니다. 예: Serial.println(temp);

#include <iostream>
#include <string>
#include <vector>
#include <cstring>
#include <cerrno>
#include <cstdlib>
#include <cmath>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <fcntl.h>   // Non-blocking 설정
#include <termios.h> // 시리얼 포트 설정

#ifndef PORT
#define PORT 8080   // 다른 포트로 빌드하려면: g++ -DPORT=9090 ...
#endif
#define MAX_BUFFER 1024
// 인자 없이 실행하면 순서대로 시도할 시리얼 포트
//   ttyUSB*: Uno 호환보드(CH340), ESP32 DevKit(CP2102/CH340)
//   ttyACM*: Uno 정품(16U2), ESP32-S3/C3 네이티브 USB
static const char* SERIAL_CANDIDATES[] = {"/dev/ttyUSB0", "/dev/ttyACM0"};
#define TEMP_MIN -40.0f   // 정상 범위를 벗어난 값은 노이즈로 보고 버림
#define TEMP_MAX 125.0f

// ---------------------------------------------------------------------------
// 센서 데이터 획득
// ---------------------------------------------------------------------------
static int serial_fd = -1;          // -1 이면 가상 데이터 모드
static std::string serial_line;     // 아직 줄바꿈이 오지 않은 시리얼 데이터
static float last_temp = 20.0f;     // 가장 최근 온도 값
static bool has_temp = false;       // 아두이노에서 유효한 값을 한 번이라도 받았는지

// 시리얼 포트를 열고 9600 8N1, Non-blocking 으로 설정
int openSerial(const char* device) {
    int fd = open(device, O_RDONLY | O_NOCTTY | O_NONBLOCK);
    if (fd < 0) {
        perror("Serial open failed");
        return -1;
    }

    struct termios tty;
    if (tcgetattr(fd, &tty) != 0) {
        perror("tcgetattr failed");
        close(fd);
        return -1;
    }
    cfmakeraw(&tty);
    cfsetispeed(&tty, B9600);
    cfsetospeed(&tty, B9600);
    tty.c_cflag |= (CLOCAL | CREAD);
    if (tcsetattr(fd, TCSANOW, &tty) != 0) {
        perror("tcsetattr failed");
        close(fd);
        return -1;
    }
    return fd;
}

float getArduinoTemp() {
    if (serial_fd < 0) {
        // 가상 데이터 생성
        static float temp = 20.0f;
        temp += 0.1f;
        if (temp > 30.0f) temp = 20.0f;
        last_temp = temp;
        has_temp = true;
        return last_temp;
    }

    // 시리얼에서 읽을 수 있는 만큼 읽고, 완성된 줄 중 마지막 값을 사용
    char buf[256];
    ssize_t n;
    while ((n = read(serial_fd, buf, sizeof(buf))) > 0) {
        serial_line.append(buf, n);
    }

    size_t pos;
    while ((pos = serial_line.find('\n')) != std::string::npos) {
        std::string line = serial_line.substr(0, pos);
        serial_line.erase(0, pos + 1);
        while (!line.empty() && (line.back() == '\r' || line.back() == ' ')) line.pop_back();
        if (line.empty()) continue;

        // 줄 전체가 숫자일 때만 온도로 인정
        //  - '#' 상태 메시지, ESP32 부팅 시 섞여 들어오는 깨진 문자 등은 무시
        //  - "nan", "inf"는 strtof가 숫자로 받아들이므로 isfinite로 따로 거름
        char* end = nullptr;
        float v = std::strtof(line.c_str(), &end);
        if (end != line.c_str() && *end == '\0' && std::isfinite(v) &&
            v >= TEMP_MIN && v <= TEMP_MAX) {
            last_temp = v;
            has_temp = true;
        }
    }
    if (serial_line.size() > MAX_BUFFER) serial_line.clear();  // 쓰레기 데이터 방지

    return last_temp;  // 새 값이 없으면 직전 값 유지
}

// ---------------------------------------------------------------------------
// 소켓 유틸
// ---------------------------------------------------------------------------
void setNonBlocking(int sock) {
    int opts = fcntl(sock, F_GETFL);
    if (opts < 0) {
        perror("fcntl(F_GETFL)");
        return;
    }
    if (fcntl(sock, F_SETFL, opts | O_NONBLOCK) < 0) {
        perror("fcntl(F_SETFL)");
    }
}

int main(int argc, char* argv[]) {
    if (argc > 1 && std::string(argv[1]) == "dummy") {
        std::cout << "Dummy mode: using virtual temperature data." << std::endl;
    } else if (argc > 1) {
        serial_fd = openSerial(argv[1]);
        if (serial_fd >= 0) std::cout << "Reading temperature from " << argv[1] << std::endl;
        else std::cerr << "Cannot open " << argv[1] << ", falling back to dummy data." << std::endl;
    } else {
        for (const char* dev : SERIAL_CANDIDATES) {
            if (access(dev, F_OK) != 0) continue;  // 장치 파일이 없으면 건너뜀
            serial_fd = openSerial(dev);
            if (serial_fd >= 0) {
                std::cout << "Reading temperature from " << dev << std::endl;
                break;
            }
        }
        if (serial_fd < 0) {
            std::cerr << "No board found (/dev/ttyUSB0, /dev/ttyACM0), falling back to dummy data."
                      << std::endl;
        }
    }

    int server_fd, new_socket;
    struct sockaddr_in address;
    socklen_t addrlen = sizeof(address);

    // 1. 소켓 생성
    if ((server_fd = socket(AF_INET, SOCK_STREAM, 0)) < 0) {
        perror("Socket failed");
        return -1;
    }

    // 서버 재시작 시 "Address already in use" 방지
    int opt = 1;
    if (setsockopt(server_fd, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt)) < 0) {
        perror("setsockopt");
        return -1;
    }

    // 2. 주소 바인딩
    memset(&address, 0, sizeof(address));
    address.sin_family = AF_INET;
    address.sin_addr.s_addr = INADDR_ANY;
    address.sin_port = htons(PORT);

    if (bind(server_fd, (struct sockaddr*)&address, sizeof(address)) < 0) {
        perror("Bind failed");
        close(server_fd);
        return -1;
    }

    // 3. 리스닝 시작
    if (listen(server_fd, 5) < 0) {
        perror("Listen failed");
        close(server_fd);
        return -1;
    }

    // 4. 서버 소켓을 Non-blocking으로 설정
    setNonBlocking(server_fd);

    std::cout << "Sensor Server Started on port " << PORT << " (Non-blocking)..." << std::endl;

    std::vector<int> clients;

    while (true) {
        // A. 새로운 클라이언트 접속 처리 (대기 중인 접속을 모두 처리)
        while (true) {
            addrlen = sizeof(address);
            new_socket = accept(server_fd, (struct sockaddr*)&address, &addrlen);
            if (new_socket < 0) {
                if (errno != EWOULDBLOCK && errno != EAGAIN) perror("accept");
                break;  // 더 이상 대기 중인 접속 없음
            }
            std::cout << "New connection accepted: " << inet_ntoa(address.sin_addr)
                      << ":" << ntohs(address.sin_port)
                      << " (fd=" << new_socket << ")" << std::endl;
            setNonBlocking(new_socket);
            clients.push_back(new_socket);
        }

        // B. 센서 데이터 획득
        float currentTemp = getArduinoTemp();

        // 아두이노에서 첫 값이 오기 전에는 전송하지 않음 (가짜 초기값 20 전송 방지)
        if (!has_temp) {
            usleep(100000);
            continue;
        }

        // C. 데이터 패킷 생성 (ABCD 프로토콜: 'A' 'T' value 'D')
        char packet[4];
        packet[0] = 'A';                      // Start Byte
        packet[1] = 'T';                      // Type: Temperature
        packet[2] = (char)(int)currentTemp;   // 정수부만 전송 (실습용)
        packet[3] = 'D';                      // End Byte

        // D. 모든 클라이언트에게 데이터 전송
        for (auto it = clients.begin(); it != clients.end();) {
            // MSG_NOSIGNAL: 끊긴 소켓에 send 해도 SIGPIPE로 서버가 죽지 않도록
            ssize_t sendLen = send(*it, packet, sizeof(packet), MSG_NOSIGNAL);
            if (sendLen == -1 && errno != EWOULDBLOCK && errno != EAGAIN) {
                std::cout << "Client disconnected (fd=" << *it << ")" << std::endl;
                close(*it);
                it = clients.erase(it);
                continue;
            }
            ++it;
        }

        usleep(100000);  // 100ms
    }

    close(server_fd);
    return 0;
}
