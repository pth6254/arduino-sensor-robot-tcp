// 실습 과제 2: 4축 로봇암 원격 제어 서버 (Robot Controller, Blocking)
//
// 사용법:
//   ./robot_server                 -> /dev/ttyUSB0, /dev/ttyACM0 순서로 보드를 찾아 명령 전송 (9600 baud)
//                                     (Arduino Uno / ESP32 모두 지원, 못 찾으면 가상 로봇으로 자동 전환)
//   ./robot_server /dev/ttyACM0    -> 다른 시리얼 포트 지정
//   ./robot_server dummy           -> 아두이노 없이 터미널 출력만 (가상 로봇)
//
// 프로토콜 (5장): 'a' + base + 'b' + shoulder + 'c' + upperarm + 'd' + forearm + 'e' + '\n'
//   예) "a90b45c120d60e\n"   (각도 범위 0~180)
//
// 데이터 흐름 출력 (서버 터미널):
//   [TCP -> Server]     클라이언트에서 받은 원본 바이트
//   [Parse]             해석한 4축 각도
//   [Server -> Serial]  보드로 보낸 바이트
//   [Board]             보드가 보낸 응답 (부팅 시 초기 각도, 목표 설정, 도착 각도/서보 펄스 폭)
//
// 소켓은 블로킹 모드 그대로 사용하고, poll()로 "TCP 소켓 / 시리얼 중 데이터가 온 쪽"을 골라 읽는다.

#include <iostream>
#include <string>
#include <cstring>
#include <cstdio>
#include <cerrno>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <fcntl.h>
#include <termios.h>
#include <poll.h>

#ifndef PORT
#define PORT 8081   // 센서 서버(8080)와 동시에 실행할 수 있도록 다른 포트 사용
#endif
#define MAX_BUFFER 1024
#define MAX_LINE 64           // 정상 명령은 최대 "a180b180c180d180e" = 17자
#define BOOT_WAIT_MS 3000     // 포트를 열면 보드가 리셋됨 -> 부팅 메시지를 기다리는 최대 시간

// 인자 없이 실행하면 순서대로 시도할 시리얼 포트
//   ttyUSB*: Uno 호환보드(CH340), ESP32 DevKit(CP2102/CH340)
//   ttyACM*: Uno 정품(16U2), ESP32-S3/C3 네이티브 USB
static const char* SERIAL_CANDIDATES[] = {"/dev/ttyUSB0", "/dev/ttyACM0"};

static int serial_fd = -1;          // -1 이면 가상 로봇 모드
static std::string serial_pending;  // 보드에서 받았지만 아직 '\n'이 오지 않은 데이터

struct ArmAngles {
    int base;
    int shoulder;
    int upperarm;
    int forearm;
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

// ---------------------------------------------------------------------------
// 보드 시리얼
// ---------------------------------------------------------------------------
int openSerial(const char* device) {
    // 읽기/쓰기 모두: 명령을 보내고, 보드의 응답도 받아서 출력
    int fd = open(device, O_RDWR | O_NOCTTY);
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

// 보드에서 온 데이터를 읽어서 한 줄씩 출력. 이번에 "ready" 줄을 봤으면 true.
bool readBoard() {
    char buf[256];
    ssize_t n = read(serial_fd, buf, sizeof(buf));
    if (n <= 0) {
        if (n < 0 && (errno == EAGAIN || errno == EINTR)) return false;
        std::cout << "[Board] serial disconnected (USB unplugged?) -> virtual robot mode" << std::endl;
        close(serial_fd);
        serial_fd = -1;
        return false;
    }
    serial_pending.append(buf, n);

    bool sawReady = false;
    size_t pos;
    while ((pos = serial_pending.find('\n')) != std::string::npos) {
        std::string line = serial_pending.substr(0, pos);
        serial_pending.erase(0, pos + 1);
        if (!line.empty() && line.back() == '\r') line.pop_back();
        if (line.empty()) continue;
        std::cout << "[Board]             " << line << std::endl;
        if (line.find("ready") != std::string::npos) sawReady = true;
    }
    if (serial_pending.size() > MAX_BUFFER) serial_pending.clear();
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

// ---------------------------------------------------------------------------
// 프로토콜 파싱
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

// ---------------------------------------------------------------------------
// 로봇 동작
// ---------------------------------------------------------------------------
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
        std::cout << "[Server -> Serial]  (virtual robot, not sent) \"" << escape(cmd) << "\"" << std::endl;
        return;
    }
    if (write(serial_fd, cmd, len) != len) {
        perror("Serial write failed");
        return;
    }
    std::cout << "[Server -> Serial]  \"" << escape(cmd) << "\" (" << len << " bytes)" << std::endl;
}

// 클라이언트에서 받은 데이터 처리. 연결이 끊겼으면 false.
bool readClient(int sock, std::string& pending) {
    char buffer[MAX_BUFFER];
    ssize_t valread = read(sock, buffer, sizeof(buffer));
    if (valread == 0) {
        std::cout << "Client disconnected." << std::endl;
        return false;
    }
    if (valread < 0) {
        if (errno == EINTR) return true;
        perror("read");
        return false;
    }

    std::cout << "[TCP -> Server]     \"" << escape(std::string(buffer, valread))
              << "\" (" << valread << " bytes)" << std::endl;
    pending.append(buffer, valread);

    // TCP 스트림이므로 '\n' 기준으로 명령을 잘라서 처리
    size_t pos;
    while ((pos = pending.find('\n')) != std::string::npos) {
        std::string line = pending.substr(0, pos);
        pending.erase(0, pos + 1);

        ArmAngles angles;
        if (parseCommand(line, angles)) {
            moveRobot(angles);
        } else {
            std::cout << "[Invalid Command]   \"" << escape(line) << "\"" << std::endl;
        }
    }

    // 줄바꿈 없이 계속 쓰레기 데이터가 들어오는 경우 방지
    if (pending.size() > MAX_LINE) {
        std::cout << "[Invalid Command]   line too long, discarded" << std::endl;
        pending.clear();
    }
    return true;
}

int main(int argc, char* argv[]) {
    if (argc > 1 && std::string(argv[1]) == "dummy") {
        std::cout << "Dummy mode: virtual robot (terminal output only)." << std::endl;
    } else if (argc > 1) {
        serial_fd = openSerial(argv[1]);
        if (serial_fd >= 0) std::cout << "Sending commands to board on " << argv[1] << std::endl;
        else std::cerr << "Cannot open " << argv[1]
                       << ", falling back to virtual robot (terminal output only)." << std::endl;
    } else {
        for (const char* dev : SERIAL_CANDIDATES) {
            if (access(dev, F_OK) != 0) continue;  // 장치 파일이 없으면 건너뜀
            serial_fd = openSerial(dev);
            if (serial_fd >= 0) {
                std::cout << "Sending commands to board on " << dev << std::endl;
                break;
            }
        }
        if (serial_fd < 0) {
            std::cerr << "No board found (/dev/ttyUSB0, /dev/ttyACM0), "
                         "falling back to virtual robot (terminal output only)." << std::endl;
        }
    }
    if (serial_fd >= 0) waitForBoot();

    // 1. 서버 소켓 생성 및 Listen (Blocking 모드 기본)
    int server_fd = socket(AF_INET, SOCK_STREAM, 0);
    if (server_fd < 0) {
        perror("Socket failed");
        return -1;
    }

    int opt = 1;
    if (setsockopt(server_fd, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt)) < 0) {
        perror("setsockopt");
        return -1;
    }

    struct sockaddr_in address;
    memset(&address, 0, sizeof(address));
    address.sin_family = AF_INET;
    address.sin_addr.s_addr = INADDR_ANY;
    address.sin_port = htons(PORT);

    if (bind(server_fd, (struct sockaddr*)&address, sizeof(address)) < 0) {
        perror("Bind failed");
        close(server_fd);
        return -1;
    }
    if (listen(server_fd, 1) < 0) {
        perror("Listen failed");
        close(server_fd);
        return -1;
    }

    std::cout << "Robot Server Waiting on port " << PORT << "..." << std::endl;

    // 2. 이벤트 루프: 클라이언트가 없으면 접속 대기, 있으면 명령 수신.
    //    동시에 보드 응답도 수신해서 출력. (컨트롤러는 한 번에 1명만 접속)
    int client = -1;
    std::string client_pending;

    while (true) {
        struct pollfd fds[2];
        int nfds = 0;
        fds[nfds++] = {client < 0 ? server_fd : client, POLLIN, 0};
        int serialIdx = -1;
        if (serial_fd >= 0) {
            serialIdx = nfds;
            fds[nfds++] = {serial_fd, POLLIN, 0};
        }

        if (poll(fds, nfds, -1) < 0) {
            if (errno == EINTR) continue;
            perror("poll");
            break;
        }

        if (fds[0].revents) {
            if (client < 0) {
                socklen_t addrlen = sizeof(address);
                client = accept(server_fd, (struct sockaddr*)&address, &addrlen);
                if (client < 0) {
                    if (errno != EINTR) perror("accept");
                } else {
                    std::cout << "Controller connected: " << inet_ntoa(address.sin_addr)
                              << ":" << ntohs(address.sin_port) << std::endl;
                    client_pending.clear();
                }
            } else if (!readClient(client, client_pending)) {
                close(client);
                client = -1;
                std::cout << "Robot Server Waiting..." << std::endl;
            }
        }

        if (serialIdx >= 0 && fds[serialIdx].revents) {
            readBoard();
        }
    }

    if (client >= 0) close(client);
    close(server_fd);
    if (serial_fd >= 0) close(serial_fd);
    return 0;
}
