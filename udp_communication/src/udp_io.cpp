#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <stdio.h>
#include <strings.h>
#include <sys/types.h>
#include <arpa/inet.h>
#include <sys/socket.h>
#include <netinet/in.h>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

#define PORT 9798
#define SERVER_IP "192.168.50.2"
#define MAXLINE 64

using namespace std::chrono_literals;

// data from stm
uint32_t epoch_stm = 0;
int16_t Encoder_motor[3] = {0, 0, 0};
float BMS_Voltage = 0;
float BMS_SOC = 0;
uint8_t Charging_Status = 0;
uint16_t Odom_Enc[2] = {0, 0};

class UdpIO : public rclcpp::Node
{
public:
  char buffer[100];
  char message[24];
  int sockfd;
  struct sockaddr_in servaddr;

  UdpIO()
      : Node("udp_io")
  {
    udp_client_init();
    timer_ = this->create_wall_timer(10ms, std::bind(&UdpIO::timer_callback, this));
  }

private:
  void timer_callback()
  {
    udp_client_routine();
  }

  void udp_client_init()
  {

    // clear servaddr
    bzero(&servaddr, sizeof(servaddr));
    servaddr.sin_addr.s_addr = inet_addr(SERVER_IP);
    servaddr.sin_port = htons(PORT);
    servaddr.sin_family = AF_INET;

    // create datagram socket
    sockfd = socket(AF_INET, SOCK_DGRAM, 0);
  }

  void udp_client_routine()
  {
    // connect to server
    if (connect(sockfd, (struct sockaddr *)&servaddr, sizeof(servaddr)) < 0)
    {
      printf("\n Error : Connect Failed \n");
      exit(0);
    }
    PC_Mode = 2;
    memcpy(message, &udp_epoch, 4);
    memcpy(message + 4, &PC_Mode, 1);
    PC_Vel[2] = -50;
    // PC_Vel[1] = 50;
    memcpy(message + 5, &PC_Vel, 6);

    udp_epoch++;

    // Send message to server none blocking mode
    sendto(sockfd, message, MAXLINE, 0, (struct sockaddr *)&servaddr, sizeof(servaddr));

    // receive the datagram in non blocking mode
    int len = sizeof(servaddr);
    int n = recvfrom(sockfd, buffer, sizeof(buffer), MSG_DONTWAIT, (struct sockaddr *)&servaddr, (socklen_t *)&len);
    if (n > 0)
    {
      buffer[n] = 0;
      // RCLCPP_INFO(this->get_logger(), "Server : %s", buffer);
      memcpy(&epoch_stm, buffer, 4);
      memcpy(&Encoder_motor, buffer + 4, 6);
      memcpy(&BMS_Voltage, buffer + 16, 4);
      memcpy(&BMS_SOC, buffer + 20, 4);
      memcpy(&Charging_Status, buffer + 24, 1);
      memcpy(&Odom_Enc, buffer + 27, 4);

      printf("%d %d %d %f %f %d %d %d\n", epoch_stm, Encoder_motor[0], Encoder_motor[1], BMS_Voltage, BMS_SOC, Charging_Status, Odom_Enc[0], Odom_Enc[1]);
    }
  }

  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;

  // RobotInterface Variables
  uint32_t udp_epoch = 0;
  uint8_t PC_Mode = 0;
  int16_t PC_Vel[3] = {0, 0, 0};
};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<UdpIO>());
  rclcpp::shutdown();

  // close the socket
  close(UdpIO().sockfd);
  return 0;
}
