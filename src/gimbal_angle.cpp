#include <ros/ros.h>
#include <geometry_msgs/Vector3.h>
#include <std_msgs/Int32.h>
#include <stdint.h>
#include <unistd.h>
#include <fcntl.h>
#include <termio.h>
#include <cstring>
#include <cmath>
#include <iostream>
#include <iomanip>

#define BUF_SIZE 47

int missionMode = -1;

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

uint8_t funcChecksum(uint8_t* viewlink_data_buf)
{
	uint8_t len = viewlink_data_buf[3];
	uint8_t checksum = len;
	for(uint8_t i = 0 ; i< len-2; i++)
	{
		checksum = checksum ^ viewlink_data_buf[4+i];
	}
	return checksum;
}

 // Mission flag callBack
void missionCallback(const std_msgs::Int32::ConstPtr& msg)
{
    missionMode = msg->data;
    
    // std::cout<<missionMode<<std::endl;
    // ROS_INFO("Received Mission Mode: %d", msg->data);
}

// 16비트 부호 있는 정수 변환
int16_t hexToSignedDecimal(uint16_t hexValue) {
    return static_cast<int16_t>(hexValue);
}

int main(int argc, char** argv) {
    // std::cout<<"0111";
    ros::init(argc, argv, "gimbal_control");
    ros::NodeHandle nh;

    ros::Publisher imu_pub = nh.advertise<geometry_msgs::Vector3>("/encoder_angles", 1);
     
    // Mission flag subscriber
    ros::Subscriber sub = nh.subscribe("/missionTogimbal", 10, missionCallback);


    int fd;
    char command[7] = {0x55, 0xaa, 0xdc, 0x04, 0x10, 0x00, 0x14};
    uint8_t pitchDownCommand[20] = {0x55, 0xAA, 0xDC, 0x11, 0x30, 0x0B, 0x00, 0x00, 0x40, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x2A};
    char WhiteHotIR[20] = {0x55, 0xaa, 0xdc, 0x11, 0x30, 0x0F,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x03,0x82,0x00,0x00,0x00,0xAF};
    char BlackHotIR[20] = {0x55, 0xaa, 0xdc, 0x11, 0x30, 0x0F,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x03,0xC2,0x00,0x00,0x00,0xEF};
    char RedHotIR[20] = {0x55, 0xaa, 0xdc, 0x11, 0x30, 0x0F,0x1F,0xFE,0x0E,0x38,0x00,0x00,0x00,0x00,0x04,0x82,0x00,0x00,0x00,0x7F};
    char buf[BUF_SIZE] = {0x00};

	// std::cout<<"0";
    uint8_t checksum = funcChecksum(pitchDownCommand);
    
    // std::cout<<(uint8_t)checksum<<std::endl;
    pitchDownCommand[19] = checksum;

    // std::cout<<"1";
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

    ros::Rate loop_rate(100); // 10 Hz
    // write(fd, command, 7);
    // std::cout<<"2";
    while (1) {
        
        // Arrive destination
        if(missionMode == 3)
        {
            write(fd, pitchDownCommand, 20);
        
            // std::cout<<"OK"<<std::endl;
        }
        // if(missionMode == 5)
            // write(fd, WhiteHotIR, 20);
            
       	// std::cout<<"ASDFAS"<<std::endl; 
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

        ros::spinOnce();
         loop_rate.sleep();
    }

    close(fd);
    return 0;
}

// #include "ros/ros.h"
// #include <std_msgs/Int32.h>
 
// void msgCallback(const std_msgs::Int32::ConstPtr& msg)  //const 상수
// {
//     ROS_INFO("receive msg =");
// }
 
// int main(int argc, char **argv)
// {
//     ros::init(argc, argv, "yh_sub_1");
//     ros::NodeHandle nh;
    
//     ros::Subscriber sub = nh.subscribe("missionTogimbal",100,msgCallback);
//     ros::spin(); //어떤 값이 들어오기 전까지 대기 (다시 위로 올라감)
//     return 0;
// }
