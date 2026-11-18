// 실습 과제 2: 4축 로봇암 원격 제어 클라이언트 (조종기, 터미널 입력)
//
// 사용법:
//   ./robot_client                 -> 127.0.0.1:8081 접속
//   ./robot_client 192.168.0.10    -> 지정한 서버 IP로 접속
//
// 입력 명령:
//   90 45 120 60     -> 네 축 각도를 한 번에 지정 (base shoulder upperarm forearm)
//   base 120         -> 한 축만 변경 (base / shoulder / upperarm / forearm)
//   show             -> 현재 각도 출력
//   q                -> 종료

#include <iostream>
#include <sstream>
#include <string>
#include <cstring>
#include <cstdio>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <unistd.h>

#ifndef PORT
#define PORT 8081
#endif

struct ArmAngles {
    int base = 90;
    int shoulder = 90;
    int upperarm = 90;
    int forearm = 90;
};

bool inRange(int angle) { return angle >= 0 && angle <= 180; }

void printAngles(const ArmAngles& a) {
    std::cout << "  Base=" << a.base << " Shoulder=" << a.shoulder
              << " Upperarm=" << a.upperarm << " Forearm=" << a.forearm << std::endl;
}

// 5장 프로토콜 문자열 생성: a<base>b<shoulder>c<upperarm>d<forearm>e\n
std::string buildPacket(const ArmAngles& a) {
    char buf[64];
    std::snprintf(buf, sizeof(buf), "a%db%dc%dd%de\n",
                  a.base, a.shoulder, a.upperarm, a.forearm);
    return buf;
}

// 사용자 입력 한 줄을 해석해서 angles를 갱신. 전송할 필요가 있으면 true.
bool handleInput(const std::string& line, ArmAngles& angles) {
    std::istringstream iss(line);
    std::string first;
    if (!(iss >> first)) return false;  // 빈 줄

    if (first == "show") {
        printAngles(angles);
        return false;
    }

    // "base 120" 형태
    int* target = nullptr;
    if (first == "base") target = &angles.base;
    else if (first == "shoulder") target = &angles.shoulder;
    else if (first == "upperarm") target = &angles.upperarm;
    else if (first == "forearm") target = &angles.forearm;

    if (target) {
        int v;
        std::string extra;
        if (!(iss >> v) || (iss >> extra)) {
            std::cout << "  Usage: " << first << " <0~180>" << std::endl;
            return false;
        }
        if (!inRange(v)) {
            std::cout << "  Angle must be 0~180" << std::endl;
            return false;
        }
        *target = v;
        return true;
    }

    // "90 45 120 60" 형태
    std::istringstream nums(line);
    ArmAngles next;
    std::string extra;
    if (!(nums >> next.base >> next.shoulder >> next.upperarm >> next.forearm) || (nums >> extra)) {
        std::cout << "  Unknown command. e.g. \"90 45 120 60\", \"base 120\", \"show\", \"q\"" << std::endl;
        return false;
    }
    if (!inRange(next.base) || !inRange(next.shoulder) ||
        !inRange(next.upperarm) || !inRange(next.forearm)) {
        std::cout << "  Angle must be 0~180" << std::endl;
        return false;
    }
    angles = next;
    return true;
}

int main(int argc, char* argv[]) {
    const char* server_ip = (argc > 1) ? argv[1] : "127.0.0.1";

    // 1. 소켓 생성
    int sock = socket(AF_INET, SOCK_STREAM, 0);
    if (sock < 0) {
        perror("Socket creation error");
        return -1;
    }

    // 2. 서버 접속
    struct sockaddr_in serv_addr;
    memset(&serv_addr, 0, sizeof(serv_addr));
    serv_addr.sin_family = AF_INET;
    serv_addr.sin_port = htons(PORT);
    if (inet_pton(AF_INET, server_ip, &serv_addr.sin_addr) <= 0) {
        std::cerr << "Invalid address: " << server_ip << std::endl;
        return -1;
    }
    if (connect(sock, (struct sockaddr*)&serv_addr, sizeof(serv_addr)) < 0) {
        perror("Connection failed");
        return -1;
    }

    std::cout << "Connected to Robot Server (" << server_ip << ":" << PORT << ")." << std::endl;
    std::cout << "Commands: \"90 45 120 60\" | \"base 120\" | show | q" << std::endl;

    ArmAngles angles;
    std::string line;

    // 3. 사용자 입력 -> 패킷 생성 -> 전송
    while (true) {
        std::cout << "> " << std::flush;
        if (!std::getline(std::cin, line)) break;  // EOF (Ctrl+D)
        if (line == "q" || line == "quit") break;

        if (!handleInput(line, angles)) continue;

        std::string packet = buildPacket(angles);
        // MSG_NOSIGNAL: 서버가 끊겼을 때 SIGPIPE로 바로 죽지 않고 에러를 출력하도록
        if (send(sock, packet.data(), packet.size(), MSG_NOSIGNAL) < 0) {
            perror("Send failed (server disconnected?)");
            break;
        }
        std::cout << "  Sent: " << packet.substr(0, packet.size() - 1) << std::endl;
    }

    close(sock);
    return 0;
}
