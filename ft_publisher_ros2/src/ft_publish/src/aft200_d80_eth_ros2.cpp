#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/wrench_stamped.hpp>
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

class UDPForceTorquePublisher : public rclcpp::Node {
public:
    UDPForceTorquePublisher() : Node("udp_force_torque_publisher") {
        // ROS2 Publisher 생성
        publisher_ = this->create_publisher<geometry_msgs::msg::WrenchStamped>("/sensor/force_torque", 10);

        // 1️⃣ 소켓 생성
        s = socket(AF_INET, SOCK_DGRAM, 0);
        if (s < 0) {
            RCLCPP_ERROR(this->get_logger(), "Failed to create socket");
            rclcpp::shutdown();
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
            rclcpp::shutdown();
        }

        RCLCPP_INFO(this->get_logger(), "Sent %d bytes to sensor.", sentBytes);

        // 5️⃣ Timer 콜백 설정 (100Hz 주기)
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(1), // 100Hz
            std::bind(&UDPForceTorquePublisher::timer_callback, this)
        );
    }

    ~UDPForceTorquePublisher() {
        close(s);
        RCLCPP_INFO(this->get_logger(), "Socket closed.");
    }

private:
    void timer_callback() {
        char *recvData = recvMsg();
        if (!recvData) {
            RCLCPP_WARN(this->get_logger(), "No data received.");
            return;
        }

        auto wrench_msg = geometry_msgs::msg::WrenchStamped();
        wrench_msg.header.stamp = this->get_clock()->now();
        wrench_msg.header.frame_id = "sensor_frame";

        wrench_msg.wrench.force.x = unpackFloat(recvData);
        wrench_msg.wrench.force.y = unpackFloat(recvData + 4);
        wrench_msg.wrench.force.z = unpackFloat(recvData + 8);
        wrench_msg.wrench.torque.x = unpackFloat(recvData + 12);
        wrench_msg.wrench.torque.y = unpackFloat(recvData + 16);
        wrench_msg.wrench.torque.z = unpackFloat(recvData + 20);

        publisher_->publish(wrench_msg);
        
        // RCLCPP_INFO(this->get_logger(),
        //     "Force: [%.2f, %.2f, %.2f], Torque: [%.2f, %.2f, %.2f]",
        //     wrench_msg.wrench.force.x, wrench_msg.wrench.force.y, wrench_msg.wrench.force.z,
        //     wrench_msg.wrench.torque.x, wrench_msg.wrench.torque.y, wrench_msg.wrench.torque.z
        // );
    }

    float unpackFloat(const char *bytes) {
        uint32_t asInt = 0;
        std::memcpy(&asInt, bytes, sizeof(asInt));
        asInt = ntohl(asInt);
        float result;
        std::memcpy(&result, &asInt, sizeof(result));
        return result;
    }

    char *recvMsg() {
        static char recvData[RECV_SIZE];
        struct sockaddr_in from;
        socklen_t fromLen = sizeof(from);

        int attempt = 0;
        while (attempt < MAX_RETRY) {
            ssize_t bytesReceived = recvfrom(s, recvData, sizeof(recvData), 0, 
                                             (struct sockaddr *)&from, &fromLen);

            if (bytesReceived == RECV_SIZE) {
                return recvData;
            } else if (bytesReceived < 0) {
                perror("[ERROR] Failed to receive data");
            } else {
                RCLCPP_WARN(this->get_logger(),
                            "Received %ld bytes, expected %d. Retrying...",
                            bytesReceived, RECV_SIZE);
            }

            attempt++;
            usleep(500000); // 0.5초 대기
        }

        return nullptr;
    }

    rclcpp::Publisher<geometry_msgs::msg::WrenchStamped>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<UDPForceTorquePublisher>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
