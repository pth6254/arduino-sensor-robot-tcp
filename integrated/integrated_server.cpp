// 통합 서버: 온도 모니터링(과제 1) + 로봇암 제어(과제 2)를 보드 하나로 동시에 처리
// 보드 쪽 짝: arduino/integrated/integrated.ino
//
// 보드 하나에는 시리얼 포트가 하나뿐이라 sensor_server와 robot_server를 동시에 띄울 수 없다.
// 이 서버는 시리얼 포트를 혼자 쥐고, 두 TCP 포트를 함께 연다.
//
//   port 8080 (센서)  : 접속한 모든 클라이언트에게 100ms마다 온도 패킷 전송   'A' 'T' <온도> 'D'
//   port 8081 (로봇암): 클라이언트에서 받은 명령을 검증해서 보드로 전달       a90b45c120d60e\n
//
//   -> 기존 sensor_client, robot_client, GUI(gui/monitor_gui.py)가 그대로 접속할 수 있다.
//
// 사용법:
//   ./integrated_server                 -> /dev/ttyUSB0, /dev/ttyACM0 순서로 보드를 찾음 (9600 baud)
//                                          (못 찾으면 가상 모드로 자동 전환)
//   ./integrated_server /dev/ttyACM0    -> 다른 시리얼 포트 지정
//   ./integrated_server dummy           -> 보드 없이 가상 온도 + 터미널 출력만
//
// 모든 소켓은 논블로킹이고, poll()로 "읽을 것이 있는 곳"만 골라 처리한다.

#include <iostream>
#include <string>
#include <vector>
#include <chrono>
#include <cstring>
#include <cstdio>
#include <cstdlib>
#include <cerrno>
#include <cmath>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <fcntl.h>
#include <termios.h>
#include <poll.h>

#ifndef SENSOR_PORT
#define SENSOR_PORT 8080
#endif
#ifndef ROBOT_PORT
#define ROBOT_PORT 8081
#endif
#define MAX_BUFFER 1024
#define MAX_LINE 64             // 정상 명령은 최대 "a180b180c180d180e" = 17자
#define SEND_INTERVAL_MS 100    // 온도 패킷 전송 주기
#define BOOT_WAIT_MS 3000       // 포트를 열면 보드가 리셋됨 -> 부팅 메시지를 기다리는 최대 시간
#define TEMP_MIN -40.0f         // 정상 범위를 벗어난 값은 노이즈로 보고 버림
#define TEMP_MAX 125.0f

// 인자 없이 실행하면 순서대로 시도할 시리얼 포트
//   ttyUSB*: Uno 호환보드(CH340), ESP32 DevKit(CP2102/CH340)
//   ttyACM*: Uno 정품(16U2), ESP32-S3/C3 네이티브 USB
static const char* SERIAL_CANDIDATES[] = {"/dev/ttyUSB0", "/dev/ttyACM0"};

static bool dummy_mode = false;     // true면 보드 없이 가상 데이터 사용
static int serial_fd = -1;
static std::string serial_pending;  // 보드에서 받았지만 아직 '\n'이 오지 않은 데이터
static float last_temp = 20.0f;     // 가장 최근 온도 값
static bool has_temp = false;       // 유효한 온도를 한 번이라도 받았는지

struct ArmAngles {
    int base;
    int shoulder;
    int upperarm;
    int forearm;
};

struct RobotClient {
    int fd;
    std::string pending;  // 아직 '\n'이 오지 않은 데이터
};

// 출력용: 줄바꿈 같은 제어 문자를 \n, \r 형태로 보이게 바꿈
std::string escape(const std::string& s) {
    std::string out;
    for (unsigned char c : s) {
        if (c == '\n') out += "\\n";
        else if (c == '\r') out += "\\r";
        else if (c < 0x20 || c >= 0x7f) {
            char buf[8];
            std::snprintf(buf, sizeof(buf), "\\x%02X", c);
            out += buf;
        } else out += (char)c;
    }
    return out;
}

void setNonBlocking(int fd) {
    int opts = fcntl(fd, F_GETFL);
    if (opts < 0 || fcntl(fd, F_SETFL, opts | O_NONBLOCK) < 0) perror("fcntl");
}

// ---------------------------------------------------------------------------
// 보드 시리얼
// ---------------------------------------------------------------------------
int openSerial(const char* device) {
    int fd = open(device, O_RDWR | O_NOCTTY | O_NONBLOCK);
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

// 보드에서 온 한 줄 처리: '#'은 상태 메시지, 줄 전체가 숫자면 온도. "ready" 줄이면 true.
bool handleBoardLine(const std::string& line) {
    if (line[0] == '#') {
        std::cout << "[Board]             " << line << std::endl;
        return line.find("ready") != std::string::npos;
    }

    // 줄 전체가 숫자일 때만 온도로 인정 (ESP32 부팅 시 깨진 문자, "nan"/"inf" 등은 무시)
    char* end = nullptr;
    float v = std::strtof(line.c_str(), &end);
    if (end != line.c_str() && *end == '\0' && std::isfinite(v) &&
        v >= TEMP_MIN && v <= TEMP_MAX) {
        if (!has_temp || v != last_temp) {  // 값이 바뀔 때만 출력
            std::cout << "[Board]             temperature " << line << " C" << std::endl;
        }
        last_temp = v;
        has_temp = true;
    } else {
        std::cout << "[Board]             (ignored) \"" << escape(line) << "\"" << std::endl;
    }
    return false;
}

// 보드에서 읽을 수 있는 만큼 읽어서 줄 단위로 처리. 이번에 "ready" 줄을 봤으면 true.
bool readBoard() {
    char buf[256];
    bool sawReady = false;

    while (true) {
        ssize_t n = read(serial_fd, buf, sizeof(buf));
        if (n > 0) {
            serial_pending.append(buf, n);
            continue;
        }
        if (n < 0 && (errno == EAGAIN || errno == EWOULDBLOCK)) break;  // 더 읽을 것 없음
        if (n < 0 && errno == EINTR) continue;

        // n == 0 또는 그 외 에러: USB가 뽑힘
        std::cout << "[Board]             serial disconnected (USB unplugged?)."
                     " Temperature stops, robot commands are not sent." << std::endl;
        close(serial_fd);
        serial_fd = -1;
        has_temp = false;   // 오래된 온도를 계속 보내지 않도록
        break;
    }

    size_t pos;
    while ((pos = serial_pending.find('\n')) != std::string::npos) {
        std::string line = serial_pending.substr(0, pos);
        serial_pending.erase(0, pos + 1);
        while (!line.empty() && (line.back() == '\r' || line.back() == ' ')) line.pop_back();
        if (line.empty()) continue;
        if (handleBoardLine(line)) sawReady = true;
    }
    if (serial_pending.size() > MAX_BUFFER) serial_pending.clear();  // 쓰레기 데이터 방지
    return sawReady;
}

// 포트를 열면 보드가 리셋되므로, 부팅 메시지("ready")가 올 때까지 기다리며 출력
void waitForBoot() {
    std::cout << "Waiting for board boot (port open resets the board)..." << std::endl;
    int remaining = BOOT_WAIT_MS;
    while (remaining > 0 && serial_fd >= 0) {
        struct pollfd p = {serial_fd, POLLIN, 0};
        int r = poll(&p, 1, 100);
        remaining -= 100;
        if (r > 0 && readBoard()) return;
    }
    std::cout << "(no ready message from board within " << BOOT_WAIT_MS / 1000
              << "s - continuing anyway)" << std::endl;
}

// 가상 모드용 온도 생성 (20 ~ 30도를 천천히 반복)
void updateDummyTemp() {
    static float temp = 20.0f;
    temp += 0.02f;
    if (temp > 30.0f) temp = 20.0f;
    last_temp = temp;
    has_temp = true;
}

// ---------------------------------------------------------------------------
// 로봇암 명령
// ---------------------------------------------------------------------------
// "a90b45c120d60e" 형태의 한 줄을 파싱. 형식이 틀리거나 각도가 범위 밖이면 false.
bool parseCommand(const std::string& line, ArmAngles& out) {
    int consumed = 0;
    int n = std::sscanf(line.c_str(), "a%db%dc%dd%de%n",
                        &out.base, &out.shoulder, &out.upperarm, &out.forearm, &consumed);
    if (n != 4 || consumed == 0) return false;

    // 뒤에 남는 문자가 없어야 함 ('\r'은 허용: 윈도우식 줄바꿈)
    for (size_t i = consumed; i < line.size(); ++i) {
        if (line[i] != '\r') return false;
    }

    const int angles[4] = {out.base, out.shoulder, out.upperarm, out.forearm};
    for (int a : angles) {
        if (a < 0 || a > 180) return false;
    }
    return true;
}

void moveRobot(const ArmAngles& a) {
    std::cout << "[Parse]             Base=" << a.base
              << " Shoulder=" << a.shoulder
              << " Upperarm=" << a.upperarm
              << " Forearm=" << a.forearm << std::endl;

    // 보드에는 검증된 명령을 같은 프로토콜 형식으로 다시 만들어 전달
    char cmd[MAX_LINE];
    int len = std::snprintf(cmd, sizeof(cmd), "a%db%dc%dd%de\n",
                            a.base, a.shoulder, a.upperarm, a.forearm);

    if (serial_fd < 0) {
        std::cout << "[Server -> Serial]  (" << (dummy_mode ? "virtual robot" : "board disconnected")
                  << ", not sent) \"" << escape(cmd) << "\"" << std::endl;
        return;
    }
    if (write(serial_fd, cmd, len) != len) {
        perror("Serial write failed");
        return;
    }
    std::cout << "[Server -> Serial]  \"" << escape(cmd) << "\" (" << len << " bytes)" << std::endl;
}

// 로봇 클라이언트에서 받은 데이터 처리. 연결이 끊겼으면 false.
bool readRobotClient(RobotClient& c) {
    char buffer[MAX_BUFFER];
    ssize_t valread = read(c.fd, buffer, sizeof(buffer));
    if (valread == 0) return false;
    if (valread < 0) {
        return errno == EAGAIN || errno == EWOULDBLOCK || errno == EINTR;
    }

    std::cout << "[TCP -> Server]     \"" << escape(std::string(buffer, valread))
              << "\" (" << valread << " bytes)" << std::endl;
    c.pending.append(buffer, valread);

    // TCP 스트림이므로 '\n' 기준으로 명령을 잘라서 처리
    size_t pos;
    while ((pos = c.pending.find('\n')) != std::string::npos) {
        std::string line = c.pending.substr(0, pos);
        c.pending.erase(0, pos + 1);

        ArmAngles angles;
        if (parseCommand(line, angles)) {
            moveRobot(angles);
        } else {
            std::cout << "[Invalid Command]   \"" << escape(line) << "\"" << std::endl;
        }
    }

    // 줄바꿈 없이 계속 쓰레기 데이터가 들어오는 경우 방지
    if (c.pending.size() > MAX_LINE) {
        std::cout << "[Invalid Command]   line too long, discarded" << std::endl;
        c.pending.clear();
    }
    return true;
}

// ---------------------------------------------------------------------------
// 소켓
// ---------------------------------------------------------------------------
int makeListener(int port) {
    int fd = socket(AF_INET, SOCK_STREAM, 0);
    if (fd < 0) {
        perror("Socket failed");
        return -1;
    }

    int opt = 1;
    setsockopt(fd, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));

    struct sockaddr_in address;
    memset(&address, 0, sizeof(address));
    address.sin_family = AF_INET;
    address.sin_addr.s_addr = INADDR_ANY;
    address.sin_port = htons(port);

    if (bind(fd, (struct sockaddr*)&address, sizeof(address)) < 0) {
        std::cerr << "Bind failed on port " << port << ": " << strerror(errno) << std::endl;
        close(fd);
        return -1;
    }
    if (listen(fd, 5) < 0) {
        perror("Listen failed");
        close(fd);
        return -1;
    }
    setNonBlocking(fd);
    return fd;
}

// 대기 중인 접속을 모두 받아서 반환 (논블로킹)
std::vector<int> acceptAll(int listen_fd, const char* label) {
    std::vector<int> accepted;
    while (true) {
        struct sockaddr_in address;
        socklen_t addrlen = sizeof(address);
        int fd = accept(listen_fd, (struct sockaddr*)&address, &addrlen);
        if (fd < 0) {
            if (errno != EWOULDBLOCK && errno != EAGAIN && errno != EINTR) perror("accept");
            break;
        }
        std::cout << label << " connected: " << inet_ntoa(address.sin_addr)
                  << ":" << ntohs(address.sin_port) << " (fd=" << fd << ")" << std::endl;
        setNonBlocking(fd);
        accepted.push_back(fd);
    }
    return accepted;
}

int main(int argc, char* argv[]) {
    if (argc > 1 && std::string(argv[1]) == "dummy") {
        dummy_mode = true;
        std::cout << "Dummy mode: virtual temperature + virtual robot." << std::endl;
    } else if (argc > 1) {
        serial_fd = openSerial(argv[1]);
        if (serial_fd >= 0) std::cout << "Using board on " << argv[1] << std::endl;
    } else {
        for (const char* dev : SERIAL_CANDIDATES) {
            if (access(dev, F_OK) != 0) continue;  // 장치 파일이 없으면 건너뜀
            serial_fd = openSerial(dev);
            if (serial_fd >= 0) {
                std::cout << "Using board on " << dev << std::endl;
                break;
            }
        }
    }
    if (!dummy_mode && serial_fd < 0) {
        dummy_mode = true;
        std::cerr << "No board available, falling back to dummy mode"
                     " (virtual temperature + virtual robot)." << std::endl;
    }
    if (serial_fd >= 0) waitForBoot();

    int sensor_listen = makeListener(SENSOR_PORT);
    int robot_listen = makeListener(ROBOT_PORT);
    if (sensor_listen < 0 || robot_listen < 0) return -1;

    std::cout << "Integrated Server Started: sensor port " << SENSOR_PORT
              << ", robot port " << ROBOT_PORT << " (Non-blocking)..." << std::endl;

    std::vector<int> sensor_clients;
    std::vector<RobotClient> robot_clients;

    using Clock = std::chrono::steady_clock;
    auto last_send = Clock::now();

    while (true) {
        // --- 읽을 것이 생기거나, 다음 온도 전송 시각이 될 때까지 대기 ---
        std::vector<struct pollfd> fds;
        fds.push_back({sensor_listen, POLLIN, 0});
        fds.push_back({robot_listen, POLLIN, 0});
        int serialIdx = -1;
        if (serial_fd >= 0) {
            serialIdx = (int)fds.size();
            fds.push_back({serial_fd, POLLIN, 0});
        }
        size_t robotBase = fds.size();
        for (const auto& c : robot_clients) fds.push_back({c.fd, POLLIN, 0});

        auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(
                           Clock::now() - last_send).count();
        int timeout = elapsed >= SEND_INTERVAL_MS ? 0 : (int)(SEND_INTERVAL_MS - elapsed);

        if (poll(fds.data(), fds.size(), timeout) < 0) {
            if (errno == EINTR) continue;
            perror("poll");
            break;
        }

        // --- 로봇 클라이언트 명령 수신 (새 접속을 추가하기 전에 처리: fds 인덱스와 맞추기 위해) ---
        for (size_t i = robot_clients.size(); i-- > 0;) {
            if (!fds[robotBase + i].revents) continue;
            if (!readRobotClient(robot_clients[i])) {
                std::cout << "Robot client disconnected (fd=" << robot_clients[i].fd << ")" << std::endl;
                close(robot_clients[i].fd);
                robot_clients.erase(robot_clients.begin() + i);
            }
        }

        // --- 보드 응답 / 온도 수신 ---
        if (serialIdx >= 0 && fds[serialIdx].revents) readBoard();

        // --- 새 접속 ---
        if (fds[0].revents) {
            for (int fd : acceptAll(sensor_listen, "Sensor client")) sensor_clients.push_back(fd);
        }
        if (fds[1].revents) {
            for (int fd : acceptAll(robot_listen, "Robot client")) robot_clients.push_back({fd, ""});
        }

        // --- 100ms마다 온도 패킷 전송 ---
        auto now = Clock::now();
        if (std::chrono::duration_cast<std::chrono::milliseconds>(now - last_send).count()
                < SEND_INTERVAL_MS) continue;
        last_send = now;

        if (dummy_mode) updateDummyTemp();
        if (!has_temp) continue;  // 보드에서 첫 값이 오기 전에는 보내지 않음

        // ABCD 프로토콜: 'A' 'T' value 'D'
        char packet[4];
        packet[0] = 'A';                    // Start Byte
        packet[1] = 'T';                    // Type: Temperature
        packet[2] = (char)(int)last_temp;   // 정수부만 전송 (실습용)
        packet[3] = 'D';                    // End Byte

        for (auto it = sensor_clients.begin(); it != sensor_clients.end();) {
            // MSG_NOSIGNAL: 끊긴 소켓에 send 해도 SIGPIPE로 서버가 죽지 않도록
            ssize_t sent = send(*it, packet, sizeof(packet), MSG_NOSIGNAL);
            if (sent == -1 && errno != EWOULDBLOCK && errno != EAGAIN) {
                std::cout << "Sensor client disconnected (fd=" << *it << ")" << std::endl;
                close(*it);
                it = sensor_clients.erase(it);
                continue;
            }
            ++it;
        }
    }

    return 0;
}
