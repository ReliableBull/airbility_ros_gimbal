#include <ros/ros.h>
#include <geometry_msgs/Vector3.h>
#include <stdint.h>
#include <unistd.h>
#include <fcntl.h>
#include <termio.h>
#include <cstring>
#include <cmath>
#include <iostream>
#include <iomanip>

#define BUF_SIZE 47

// 구조체 정의
typedef struct {
    uint16_t timestamp;
    int16_t rollIMUangle;
    int16_t pitchIMUangle;
    int16_t yawIMUangle; 
    int16_t rollIMUspeed;
    int16_t pitchIMUspeed;
    int16_t yawIMUspeed; 
    int16_t rollStatorRotorAngle;
    int16_t pitchStatorRotorAngle;
    int16_t yawStatorRotorAngle; 
} T_GimbalGetAnglesExtReq;

// 16비트 부호 있는 정수 변환
int16_t hexToSignedDecimal(uint16_t hexValue) {
    return static_cast<int16_t>(hexValue);
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "gimbal_control");
    ros::NodeHandle nh;

    ros::Publisher imu_pub = nh.advertise<geometry_msgs::Vector3>("encoder_angles", 1);

    int fd;
    char command[7] = {0x55, 0xaa, 0xdc, 0x04, 0x10, 0x00, 0x14};
    char buf[BUF_SIZE] = {0x00};

    // 시리얼 포트 설정
    fd = open("/dev/ttyUSB1", O_RDWR | O_NOCTTY);
    if (fd == -1) {
        ROS_ERROR("Failed to open serial port!");
        return -1;
    }

    struct termios newtio;
    memset(&newtio, 0, sizeof(newtio));
    newtio.c_cflag = B115200 | CS8 | CLOCAL | CREAD;
    newtio.c_iflag = IGNPAR;
    newtio.c_oflag = 0;
    newtio.c_lflag = 0;
    newtio.c_cc[VTIME] = 0;
    newtio.c_cc[VMIN] = BUF_SIZE;

    tcflush(fd, TCIFLUSH);
    tcsetattr(fd, TCSANOW, &newtio);

    ros::Rate loop_rate(30); // 10 Hz
    // write(fd, command, 7);
    while (1) {
        // write(fd, command, 7);
        
        int res = read(fd, buf, BUF_SIZE);
        // ROS_INFO("%d\n",res);
        if (buf[0] == 0x55 && buf[1] == 0xAA && buf[2] == 0xDC && buf[4] == 0x40) {
            uint16_t roll = ((buf[28] & 0x0F) << 8) + buf[29];
            uint16_t pitch = (buf[30] << 8) + buf[31];
            int16_t yaw = (buf[32] << 8) + buf[33];

            double roll_angle = (roll * 180.0) / 4095.0 - 90.0;
            double pitch_angle = (pitch * 360.0) / 65536.0;
            double yaw_angle = (hexToSignedDecimal(yaw) * 360.0) / 65536.0;

            // // ROS 메시지 생성 및 발행
            geometry_msgs::Vector3 imu_msg;
            if (pitch_angle >= 180)
                pitch_angle = pitch_angle - 360;

            imu_msg.x = roll_angle; // roll
            imu_msg.y = yaw_angle  ;// pitch
            imu_msg.z = pitch_angle ;// yaw

            imu_pub.publish(imu_msg);

            // ROS_INFO("Roll: %.2f, Pitch: %.2f, Yaw: %.2f", roll_angle, pitch_angle, yaw_angle);
            std::cout<<"roll : " <<roll_angle<<std::endl;
            std::cout<<"pitch : " <<yaw_angle<<std::endl;
            std::cout<<"yaw: " <<pitch_angle<<std::endl;
        }

        // ros::spinOnce();
        loop_rate.sleep();
    }

    close(fd);
    return 0;
}
