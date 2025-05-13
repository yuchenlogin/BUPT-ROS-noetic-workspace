#include "ros/ros.h"
#include <std_msgs/Float64MultiArray.h>
#include <signal.h>
#include"time.h"

#include <pthread.h>

#include "uartjy901.h"
pthread_t thread_recv_data;
clock_t  read_succ_t,failed_t;

int JY901type = -1;// 0->'/dev/usb_imu_906_internal'; 1->'/dev/CP210x_external'; 1->'/dev/CH340_external'; -1->null;

std::string find_jy901type(int &type) 
{

  if (access("/dev/usb_imu_906_internal", F_OK) == 0)
  {
    std::cout << "find /dev/usb_imu_906_internal!" << std::endl;
    type = 0;
    return "/dev/usb_imu_906_internal";
  }
  else if (access("/dev/CH340_external", F_OK) == 0)
  {
    std::cout << "find /dev/CH340_external!" << std::endl;
    type = 1;
    return "/dev/CH340_external";
  }
  else if (access("/dev/CP210x_external", F_OK) == 0)
  {
    std::cout << "find /dev/CP210x_external!" << std::endl;
    type = 1;
    return "/dev/CP210x_external";
  }else{
    ROS_WARN("not find jy901 device!");
    type = -1;
    usleep(500000);
    return "";
  }
}

void *recv_data_thread(void *ptr)
{
  int ret = 0;
  while (ros::ok() && ros::master::check())
  {  
    usleep(500000);// 500ms-2Hz check the port
    std::string device_name;
    device_name = find_jy901type(JY901type);
    if (device_name.size() == 0) continue;
    
    ret = jy901_init(device_name.c_str());
    if (ret == -1)
    {
      ROS_WARN("open jy901 failed!retry...");
      jy901_close();
      continue;
    }

    while (ros::ok() &&  ros::master::check())
    {
      ret = jy901_loop();
      if (ret != 0)
      {
        failed_t = clock();
        if ((double)(failed_t-read_succ_t)/CLOCKS_PER_SEC > 0.2)// if read fail 0.2s,break and reinit..
        {
          ROS_WARN("jy901 read error, reinitializing...");
          usleep(500000);// 500ms check the port
          break;
        }
      }else {read_succ_t = clock();}
    }

  }
  jy901_close();
  ROS_WARN("EXIT REC data thread");
}
void shutdown(int sig){
    ros::shutdown();  
    pthread_join(thread_recv_data, NULL);
    exit(0); 
}
void exchange_values(double * a, double * b)
{
  double temp;
  temp = *a;
  *a = *b;
  *b = temp;
}


int main(int argc, char **argv)
{
  ros::init(argc, argv, "jy901Module_node");
  ros::NodeHandle n;


  pthread_create(&thread_recv_data, NULL, recv_data_thread, NULL);

  ros::Publisher jy901Data_Pub = n.advertise<std_msgs::Float64MultiArray>("/jy901Module_node/jy901Data", 1);

  std_msgs::Float64MultiArray f64MArray;
  f64MArray.data.resize(9);

  signal(SIGINT, shutdown);

  ros::Rate loopRete(200);
  while (ros::ok() && ros::master::check())
  {
    for (uint16_t i = 0; i < 3; i++)
    {
      f64MArray.data[i] = jy901_getGyro(i);
      f64MArray.data[i + 3] = jy901_getAcc(i);
      f64MArray.data[i + 6] = jy901_getEuler(i);
    }
    if (JY901type==0)
    {
      //最新的内置jy906坐标, xy交换,z反转
      //gyro
      exchange_values(&f64MArray.data[0],&f64MArray.data[1]);
      f64MArray.data[2] = -f64MArray.data[2];
      //acc
      exchange_values(&f64MArray.data[3],&f64MArray.data[4]);
      f64MArray.data[5] = -f64MArray.data[5];
      //euler
      exchange_values(&f64MArray.data[6],&f64MArray.data[7]);
      f64MArray.data[8] = -f64MArray.data[8];

    }
    jy901Data_Pub.publish(f64MArray);

    loopRete.sleep();
  }
  pthread_join(thread_recv_data, NULL);
  jy901_close();

  return 0;
}
