#include <ros/ros.h>
#include <geometry_msgs/WrenchStamped.h>
#include <iostream>
#include <iomanip>
#include <cstring>
#include <arpa/inet.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <unistd.h>
#include <vector>

const char *IP_ADDR = "192.168.1.199";
const int SENSOR_UDP_PORT = 8890;
const int MAX_RETRY = 5;
const int RECV_SIZE = 50;
int s; // socket file descriptor

// Function to unpack a float from a byte array
float unpackFloat(const char *bytes) {
    uint32_t asInt = 0;
    std::memcpy(&asInt, bytes, sizeof(asInt));
    asInt = ntohl(asInt); // Convert from network byte order to host byte order
    float result;
    std::memcpy(&result, &asInt, sizeof(result));
    return result;
}

// Function to receive message with retry logic
char *recvMsg() {
    static char recvData[RECV_SIZE];
    struct sockaddr_in from;
    socklen_t fromLen = sizeof(from);

    int attempt = 0;
    while (attempt < MAX_RETRY) {
        ssize_t bytesReceived = recvfrom(s, recvData, sizeof(recvData), 0, 
                                         (struct sockaddr *)&from, &fromLen);

        if (bytesReceived == RECV_SIZE) {
            // std::cout << "[INFO] Received " << bytesReceived << " bytes from sensor.\n";
            return recvData;
        } else if (bytesReceived < 0) {
            perror("[ERROR] Failed to receive data");
        } else {
            std::cout << "[WARNING] Received " << bytesReceived 
                      << " bytes, expected " << RECV_SIZE << ". Retrying...\n";
        }

        attempt++;
        usleep(500000); // 0.5초 대기
    }

    std::cerr << "[ERROR] Failed to receive proper data after " 
              << MAX_RETRY << " attempts.\n";
    return nullptr;
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "udp_force_torque_publisher");
    ros::NodeHandle nh;
    ros::Publisher ft_pub = nh.advertise<geometry_msgs::WrenchStamped>("/sensor/force_torque", 10);

    // 1️⃣ 소켓 생성
    s = socket(AF_INET, SOCK_DGRAM, 0);
    if (s < 0) {
        std::cerr << "Failed to create socket" << std::endl;
        return 1;
    }

    // 2️⃣ 주소 설정
    struct sockaddr_in sensorAddr;
    sensorAddr.sin_family = AF_INET;
    sensorAddr.sin_port = htons(SENSOR_UDP_PORT);
    inet_pton(AF_INET, IP_ADDR, &sensorAddr.sin_addr);

    // 3️⃣ 타임아웃 설정 (2초)
    struct timeval tv;
    tv.tv_sec = 2;
    tv.tv_usec = 0;
    setsockopt(s, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));

    // 4️⃣ 데이터 전송
    std::string sendData = "000302";
    std::vector<unsigned char> sendDataBytes(sendData.size() / 2);
    for (size_t i = 0; i < sendData.size(); i += 2) {
        sendDataBytes[i / 2] = std::stoi(sendData.substr(i, 2), nullptr, 16);
    }

    int sentBytes = sendto(s, sendDataBytes.data(), sendDataBytes.size(), 0,
                           (struct sockaddr *)&sensorAddr, sizeof(sensorAddr));
    if (sentBytes < 0) {
        perror("[ERROR] Send failed");
        close(s);
        return 1;
    }

    std::cout << "[INFO] Sent " << sentBytes << " bytes to sensor.\n";

    // 5️⃣ 데이터 수신 및 ROS Publish
    // ros::Rate rate(1000); // 1000Hz Publish
    while (ros::ok()) {
        char *recvData = recvMsg();
        if (!recvData) {
            std::cerr << "[ERROR] No data received, exiting loop.\n";
            break;
        }

        // Force & Torque 데이터를 파싱
        geometry_msgs::WrenchStamped wrench_msg;
        wrench_msg.header.stamp = ros::Time::now();
        wrench_msg.header.frame_id = "sensor_frame";

        wrench_msg.wrench.force.x = unpackFloat(recvData);
        wrench_msg.wrench.force.y = unpackFloat(recvData + 4);
        wrench_msg.wrench.force.z = unpackFloat(recvData + 8);
        wrench_msg.wrench.torque.x = unpackFloat(recvData + 12);
        wrench_msg.wrench.torque.y = unpackFloat(recvData + 16);
        wrench_msg.wrench.torque.z = unpackFloat(recvData + 20);

        // ROS 토픽에 Publish
        ft_pub.publish(wrench_msg);

        std::cout << "[PUBLISH] Force: (" 
                  << wrench_msg.wrench.force.x << ", "
                  << wrench_msg.wrench.force.y << ", "
                  << wrench_msg.wrench.force.z << ") | Torque: ("
                  << wrench_msg.wrench.torque.x << ", "
                  << wrench_msg.wrench.torque.y << ", "
                  << wrench_msg.wrench.torque.z << ")\n";

        ros::spinOnce();
        // rate.sleep();
    }

    // 6️⃣ 소켓 종료
    close(s);
    std::cout << "[INFO] Socket closed.\n";

    return 0;
}
