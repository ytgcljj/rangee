// author: Li Chunjing, Echiev, Beijing
//2018-01-07
#include <vector>
// #include "math_utils.h"

#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Float64.h"
#include "std_msgs/Float64MultiArray.h"
#include <sensor_msgs/Imu.h>  
#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>

#include <stdio.h>
#include <string.h>
#include <fcntl.h>
#include <time.h>
#include <termios.h>
#include <unistd.h>

#include "sensor_msgs/Range.h"
#include "std_msgs/Header.h"
// #include <time.h>
#include <sstream>
// #include <tf/transform_broadcaster.h>



#define DEVICENAME                      "/dev/ttyUSB0"
#define BAUDRATE                        9600



#define MAKEWORD(a, b)  ((unsigned short)(((unsigned char)(((unsigned long)(a)) & 0xff)) | ((unsigned short)((unsigned char)(((unsigned long)(b)) & 0xff))) << 8))
#define MAKEDWORD(a, b) ((unsigned int)(((unsigned short)(((unsigned long)(a)) & 0xffff)) | ((unsigned int)((unsigned short)(((unsigned long)(b)) & 0xffff))) << 16))
#define LOWORD(l)       ((unsigned short)(((unsigned long)(l)) & 0xffff))
#define HIWORD(l)       ((unsigned short)((((unsigned long)(l)) >> 16) & 0xffff))
#define LOBYTE(w)       ((unsigned char)(((unsigned long)(w)) & 0xff))
#define HIBYTE(w)       ((unsigned char)((((unsigned long)(w)) >> 8) & 0xff))

// #define M_PI   3.141592653
#define M_G    9.81  
#define DEG_TO_RAD(a) (a/180.0*M_PI) 
#define RAD_TO_DEG(a) (a/M_PI*180.0)   

#define GPS_Buffer_Length 100
#define UTCTime_Length 11
#define latitude_Length 11
#define N_S_Length 2
#define longitude_Length 12
#define E_W_Length 2 
#define VEL_Length 11 
#define DIR_Length 11
#define UTCDate_Length 11  

#define NMI_PER_HOUR_TO_M_PER_S(a)    (a * 1852.3 / 3600.0)


ros::Publisher *serial_send_data = NULL;

class PortHandler
{
  private:
    int     socket_fd_;
    int     baudrate_;
    char    port_name_[30];

    double  packet_start_time_;
    double  packet_timeout_;
    double  tx_time_per_byte;

  public:
    static const int DEFAULT_BAUDRATE_ = 115200;

    bool   is_using_;

  public:
      PortHandler(const char *port_name);
     	~PortHandler() { }

     bool    openPort();
     void    closePort();
     void    clearPort();
     void    Port_init(const char *port_name, const int baudrate);

     bool    setBaudRate(const int baudrate);
     bool    setupPort(int cflag_baud);
     int     getCFlagBaud(int baudrate);

     int     readPort(uint8_t *packet, int length);
     int     writePort(uint8_t *packet, int length);
};


PortHandler::PortHandler(const char *port_name) 
  : socket_fd_(-1),
    baudrate_(DEFAULT_BAUDRATE_),
    packet_start_time_(0.0),
    packet_timeout_(0.0),
    tx_time_per_byte(0.0)
{
  is_using_ = false;
  strcpy(port_name_, port_name);
}

bool PortHandler::openPort()
{
  return setBaudRate(baudrate_);
}

void PortHandler::closePort()
{
  if(socket_fd_ != -1)
    close(socket_fd_);
  socket_fd_ = -1;
}

void PortHandler::clearPort()
{
  tcflush(socket_fd_, TCIOFLUSH);
}

void PortHandler::Port_init(const char *port_name, const int baudrate)
{
    // Open port
  if (openPort())
  {
    printf("Succeeded to open the port %s !\n",port_name);
  }
  else
  {
    printf("Failed to open the port!\n");
  }

// Set port baudrate
  if (setBaudRate(baudrate))
  {
    printf("Succeeded to change the baudrate %d !\n",baudrate);
  }
  else
  {
    printf("Failed to change the baudrate!\n");
    printf("Press any key to terminate...\n");
  }
}

bool PortHandler::setBaudRate(const int baudrate)
{
  int baud = getCFlagBaud(baudrate);

  closePort();

  if(baud <= 0)   // custom baudrate
  {
    printf("Please check the baudrate!\n");
    return false;
  }
  else
  {
    baudrate_ = baudrate;
    return setupPort(baud);
  }
}

bool PortHandler::setupPort(int cflag_baud)
{
  struct termios newtio;

 // socket_fd_ = open(port_name_, O_RDWR|O_NOCTTY|O_NONBLOCK);
  socket_fd_ = open(port_name_, O_RDWR|O_NOCTTY);  

  if(socket_fd_ < 0)
  {
    printf("[PortHandler::SetupPort] Error opening serial port!\n");
    return false;
  }

  bzero(&newtio, sizeof(newtio)); // clear struct for new port settings

  newtio.c_cflag = cflag_baud | CS8 | CLOCAL | CREAD;
  newtio.c_iflag = IGNPAR;
  newtio.c_oflag      = 0;
  newtio.c_lflag      = 0;
  newtio.c_cc[VTIME]  = 20;  //每个单位是0.1秒  20就是2秒
  newtio.c_cc[VMIN]   = 10;   //

  // clean the buffer and activate the settings for the port
  tcflush(socket_fd_, TCIFLUSH);
  tcsetattr(socket_fd_, TCSANOW, &newtio);

  tx_time_per_byte = (1000.0 / (double)baudrate_) * 10.0;
  return true;
}

int PortHandler::readPort(uint8_t *packet, int length)
{
  return read(socket_fd_, packet, length);
}

int PortHandler::writePort(uint8_t *packet, int length)
// int PortHandler::writePort(const char *packet, int length)
{
  return write(socket_fd_, packet, length);
}

int PortHandler::getCFlagBaud(int baudrate)
{
  switch(baudrate)
  {
    case 9600:
      return B9600;
    case 19200:
      return B19200;
    case 38400:
      return B38400;
    case 57600:
      return B57600;
    case 115200:
      return B115200;
    case 230400:
      return B230400;
    case 460800:
      return B460800;
    case 500000:
      return B500000;
    case 576000:
      return B576000;
    case 921600:
      return B921600;
    case 1000000:
      return B1000000;
    case 1152000:
      return B1152000;
    case 1250000:           //added by li chunjing 2017-03-04
      return B1152000;
    case 1500000:
      return B1500000;
    case 2000000:
      return B2000000;
    case 2500000:
      return B2500000;
    case 3000000:
      return B3000000;
    case 3500000:
      return B3500000;
    case 4000000:
      return B4000000;
    default:
      return -1;
  }
}

class File_RW
{
  private:
      int fd;        //文件描述符
      char buf[100];
      int ret;

  public:
    File_RW(){fd = -1; ret = -1;};
    ~File_RW(){};

    void file_open(char *port_name);
    void file_write(float data);
    void file_write(double data);
    void file_write(int32_t data);
    void file_write(char ch);
    void file_write(char *ch);
    void file_close();
};

void File_RW::file_open(char *port_name)
{
  fd = open(port_name, O_CREAT | O_TRUNC | O_RDWR, 0666);
 // fd = creat(port_name, O_RDWR);
  //fd = open(port_name, O_CREAT);
     if (-1 == fd)        
     {
        printf("文件打开错误\n");
     }
     else
     {
        printf("文件打开成功，fd = %d.\n", fd);
     }
}

void File_RW::file_write(float data)
{
  sprintf(buf, "%.6f", data);
  ret = write(fd, buf, strlen(buf));
}

void File_RW::file_write(double data)
{
  sprintf(buf, "%.6f", data);
  ret = write(fd, buf, strlen(buf));
}

void File_RW::file_write(int32_t data)
{
  sprintf(buf, "%d", data);
  ret = write(fd, buf, strlen(buf));
}

void File_RW::file_write(char ch)
{
  buf[0] = ch;
  ret = write(fd, buf, 1);
}

void File_RW::file_write(char *ch)
{
  uint32_t i = 0;
  while(ch[i] != '\0')
  {
    buf[i] = ch[i];
    i++;   
  }
   ret = write(fd, buf, i);

}

void File_RW::file_close()
{
  close(fd);
  printf("file closed succeeded，fd = %d.\n", fd);
}

void getNowTime(char *result)
{
  timespec time;
  tm nowTime;

  clock_gettime(CLOCK_REALTIME, &time);  //获取相对于1970到现在的秒数
  localtime_r(&time.tv_sec, &nowTime);
  // char current[1024];
  // ROS_INFO("%04d_%02d_%02d_%02d:%02d:%02d", nowTime.tm_year + 1900, nowTime.tm_mon+1, nowTime.tm_mday, 
  // nowTime.tm_hour, nowTime.tm_min, nowTime.tm_sec);

  sprintf(result, "/home/x260-16/WORK_SPACE_LCJ/test_gps/data/%04d_%02d_%02d_%02d_%02d_%02d.txt",nowTime.tm_year + 1900, nowTime.tm_mon+1, nowTime.tm_mday, 
  nowTime.tm_hour, nowTime.tm_min, nowTime.tm_sec); // 产生"123"
}

PortHandler portHandler(DEVICENAME);

uint8_t packet[100];

//file write:
File_RW file_rw;

int main(int argc, char **argv)  //订阅节点需要做四件事情：(1)初始化；(2)从主题订阅消息；(3)然后等待消息到达；(4)当消息到达时，chatterCallback()被回调
{
    ros::init(argc,argv,"serial_ultrasonic");
    ros::NodeHandle n;

    // ros::Publisher pubserial_send_data = n.advertise<std_msgs::Float64MultiArray>("/range_ultrasonic",1000);
    // serial_send_data = &pubserial_send_data;

    ros::Publisher ultrasound_pub = n.advertise<sensor_msgs::Range>("UltraSoundPublisher", 10);

    int32_t range_ultrasonic_mm = 0;
    double range_ultrasonic_m = 0;

    portHandler.Port_init(DEVICENAME, BAUDRATE);

    char file_name[100];
    getNowTime(file_name);
    file_rw.file_open(file_name);

    //指定循环的频率 
    ros::Rate loop_rate(10); 
    while(ros::ok()) 
    {
       packet[0] = 0xd2;
       packet[1] = 0x02;
       packet[2] = 0xb4;

       portHandler.writePort(packet,3);

       portHandler.readPort(packet,2);

       range_ultrasonic_mm = MAKEWORD(packet[1], packet[0]);
       range_ultrasonic_m = range_ultrasonic_mm / 1000.0;


       sensor_msgs::Range msg;
       std_msgs::Header header;
       header.stamp = ros::Time::now();
       header.frame_id = "/ultrasound";
       msg.header = header;
       msg.field_of_view = 1;
       msg.min_range = 0;
       msg.max_range = 5;
       msg.range = range_ultrasonic_m;//rand()%3;

        tf::TransformBroadcaster broadcaster;
          broadcaster.sendTransform(
          tf::StampedTransform(
          tf::Transform(tf::Quaternion(0, 0, 0, 1), tf::Vector3(0.1, 0.0, 0.2)),
          ros::Time::now(),"base_link", "ultrasound"));

        ultrasound_pub.publish(msg);

      //  std_msgs::Float64MultiArray msg;

      // //pulish range_ultrasonic data
      //  msg.data.push_back((double)(range_ultrasonic_m));

      //  serial_send_data->publish(msg);

       ROS_INFO("Rx[0]=%d  Rx[1]=%d  range_ultrasonic_m=%f",packet[0], packet[1], range_ultrasonic_m);

        //处理ROS的信息，比如订阅消息,并调用回调函数 
        ros::spinOnce(); 
        loop_rate.sleep(); 
    } 

    return 0;
}
