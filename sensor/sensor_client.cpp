// 실습 과제 1: 아두이노 센서 데이터 모니터링 클라이언트
//
// 사용법:
//   ./sensor_client               -> 127.0.0.1:8080 접속
//   ./sensor_client 192.168.0.10  -> 지정한 서버 IP로 접속

#include <iostream>
#include <string>
#include <cstring>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <unistd.h>

#ifndef PORT
#define PORT 8080   // 다른 포트로 빌드하려면: g++ -DPORT=9090 ...
#endif
#define MAX_BUFFER 1024
#define PACKET_SIZE 4

int main(int argc, char* argv[]) {
    const char* server_ip = (argc > 1) ? argv[1] : "127.0.0.1";

    int sock = 0;
    struct sockaddr_in serv_addr;
    char buffer[MAX_BUFFER] = {0};

    // 1. 소켓 생성
    if ((sock = socket(AF_INET, SOCK_STREAM, 0)) < 0) {
        perror("Socket creation error");
        return -1;
    }

    // 2. 서버 접속
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

    std::cout << "Connected to Sensor Server (" << server_ip << ":" << PORT << ")." << std::endl;

    // TCP는 스트림이라 read() 한 번에 패킷이 여러 개 붙어서 오거나 잘려서 올 수 있음.
    // 받은 데이터를 누적해 두고 4바이트 단위로 잘라서 파싱한다.
    std::string pending;

    while (true) {
        // 3. 데이터 수신 (Blocking)
        ssize_t valread = read(sock, buffer, MAX_BUFFER);
        if (valread == 0) {
            std::cout << "Server disconnected." << std::endl;
            break;
        }
        if (valread < 0) {
            perror("read");
            break;
        }
        pending.append(buffer, valread);

        // 4. ABCD 프로토콜 파싱
        while (pending.size() >= PACKET_SIZE) {
            if (pending[0] != 'A' || pending[3] != 'D') {
                pending.erase(0, 1);  // 동기화가 깨졌으면 1바이트씩 버리며 재정렬
                continue;
            }
            if (pending[1] == 'T') {
                int tempVal = (signed char)pending[2];
                std::cout << "Received Temperature: " << tempVal << " C" << std::endl;
            }
            pending.erase(0, PACKET_SIZE);
        }
    }

    close(sock);
    return 0;
}
