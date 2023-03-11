#include <ros/ros.h>
#include <serial/serial.h>
#include <serial/v8stdint.h>
#include <std_msgs/String.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Float64.h>
#include <std_msgs/ByteMultiArray.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/Twist.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Int32.h>
#include <iostream>
#include "uwb.h"

using std::cout;
using std::endl;
using namespace std;

#define RX_IN_USB 1

int en_print=0;

typedef struct {
  uint8_t header[2];
  uint8_t id;
  uint8_t role;
  float pos_3d[3];
  float vel_3d[3];
  float dis_arr[8];
  float imu_gyro_3d[3];
  float imu_acc_3d[3];
  uint8_t reserved1[12];
  float angle_3d[3];
  float quaternion[4];
  uint8_t reserved2[4];
  uint32_t local_time;
  uint32_t system_time;
  uint8_t reserved3[1];
  float eop_3d[3];
  float voltage;
  uint8_t reserved4[5];
  uint8_t check_sum;
}TAG_FRAME0;

TAG_FRAME0 frame0;

#if !RX_IN_USB
serial::Serial m_serial;
#else
serial::Serial m_serial;
#endif


#define BUFF_SIZE 200
unsigned char data_buf[BUFF_SIZE] = {0};

bool checkcommand(const unsigned char *data_buf, int data_length)
{
	uint8_t sum=0;
	int i;
	

	if (!(*(data_buf) == 0x55 && *(data_buf + 1) == 0x01))
	{
		cout<<"Head Check Fail!!!"<<endl;
		return false; 
	}

	for(int32_t i=0;i<data_length-1;++i)
	{
	 sum += *(data_buf+i);
	}

	if (*(data_buf + data_length - 1) != sum)
	{
		cout<<"Sum Check Fail!"<<endl;
		return false;
	}
	return true;
}

uint32_t uint32FromDataf(unsigned char *data,int* anal_cnt)
{
	uint32_t i = 0x00;
	i |= (*(data+*anal_cnt+3) << 24);
	i |= (*(data+*anal_cnt+2) << 16);
	i |= (*(data+*anal_cnt+1) << 8);
	i |= (*(data+*anal_cnt+0));
	
	*anal_cnt +=4;
	return i;
}

uint16_t uint16FromDataf(unsigned char *data,int* anal_cnt)
{
	uint16_t i = 0x00;
	i |= (*(data+*anal_cnt+1) << 8);
	i |= (*(data+*anal_cnt+0));
	
	*anal_cnt +=2;
	return i;
}

float floatFromDataf(unsigned char *data,int* anal_cnt)
{
	int i = 0x00;
	float out=0;
	i |= (*(data+*anal_cnt+3) << 24);
	i |= (*(data+*anal_cnt+2) << 16);
	i |= (*(data+*anal_cnt+1) << 8);
	i |= (*(data+*anal_cnt+0));
	
	*anal_cnt +=4;
	
	 out=(*(float *)&i);
	return out;
}

float int16FromDataf(unsigned char *data,int* anal_cnt)
{
	int16_t temp = (int16_t)((*(data+*anal_cnt+0)) | (*(data+*anal_cnt+1))<< 8 ); 
	float result = temp;
	*anal_cnt +=2;
	return result;
}

int32_t int24FromDataf(unsigned char *data,int* anal_cnt)
{
	int32_t temp = (int32_t)((*(data+*anal_cnt+0)) << 8 | (*(data+*anal_cnt+1))<< 16 | (*(data+*anal_cnt+2))<< 24) / 256; 
	int32_t result = temp;
	*anal_cnt +=3;
	return result;
}

char charFromDataf(unsigned char *data,int* anal_cnt)
{
	char out=0;
	 out=(*(data+*anal_cnt));
	*anal_cnt +=1;
	return (out);
}

char reverseFromDataf(unsigned char *data,int* anal_cnt,int r_num)
{
	*anal_cnt +=r_num;
	return (0);
}

int intFromDataf(unsigned char *data,int* anal_cnt)
{
	int i = 0x00;
	i |= (*(data+*anal_cnt+3) << 24);
	i |= (*(data+*anal_cnt+2) << 16);
	i |= (*(data+*anal_cnt+1) << 8);
	i |= (*(data+*anal_cnt+0));
	*anal_cnt +=4;
	return (i);
}

void decode(unsigned char *data_buf, int data_length)
{
	int anal_cnt=2;//start num
	/*
	  uint8_t id;
	  uint8_t role;
	  float pos_3d[3];
	  float vel_3d[3];
	  float dis_arr[8];
	  float imu_gyro_3d[3];
	  float imu_acc_3d[3];
	  uint8_t reserved1[12];
	  float angle_3d[3];
	  float quaternion[4];
	  uint8_t reserved2[4];
	  uint32_t local_time;
	  uint32_t system_time;
	  uint8_t reserved3[1];
	  uint8_t eop_3d[3];
	  uint16_t voltage;
	  uint8_t reserved4[5];
	*/
	frame0.id=charFromDataf(data_buf,&anal_cnt);
	frame0.role=charFromDataf(data_buf,&anal_cnt);
	
	frame0.pos_3d[0]=int24FromDataf(data_buf,&anal_cnt)/1000.0f;
	frame0.pos_3d[1]=int24FromDataf(data_buf,&anal_cnt)/1000.0f;
	frame0.pos_3d[2]=int24FromDataf(data_buf,&anal_cnt)/1000.0f;
	
	frame0.vel_3d[0]=int24FromDataf(data_buf,&anal_cnt)/10000.0f;
	frame0.vel_3d[1]=int24FromDataf(data_buf,&anal_cnt)/10000.0f;
	frame0.vel_3d[2]=int24FromDataf(data_buf,&anal_cnt)/10000.0f;
		
	frame0.dis_arr[0]=int24FromDataf(data_buf,&anal_cnt)/1000.0f;	
	frame0.dis_arr[1]=int24FromDataf(data_buf,&anal_cnt)/1000.0f;
	frame0.dis_arr[2]=int24FromDataf(data_buf,&anal_cnt)/1000.0f;	
	frame0.dis_arr[3]=int24FromDataf(data_buf,&anal_cnt)/1000.0f;	
	frame0.dis_arr[4]=int24FromDataf(data_buf,&anal_cnt)/1000.0f;	
	frame0.dis_arr[5]=int24FromDataf(data_buf,&anal_cnt)/1000.0f;
	frame0.dis_arr[6]=int24FromDataf(data_buf,&anal_cnt)/1000.0f;	
	frame0.dis_arr[7]=int24FromDataf(data_buf,&anal_cnt)/1000.0f;
	
	frame0.imu_gyro_3d[0]=floatFromDataf(data_buf,&anal_cnt);	
	frame0.imu_gyro_3d[1]=floatFromDataf(data_buf,&anal_cnt);	
	frame0.imu_gyro_3d[2]=floatFromDataf(data_buf,&anal_cnt);
		
	frame0.imu_acc_3d[0]=floatFromDataf(data_buf,&anal_cnt);	
	frame0.imu_acc_3d[1]=floatFromDataf(data_buf,&anal_cnt);	
	frame0.imu_acc_3d[2]=floatFromDataf(data_buf,&anal_cnt);
		
	reverseFromDataf(data_buf,&anal_cnt,12);
	
	frame0.angle_3d[0]=int16FromDataf(data_buf,&anal_cnt)/100.0f;	
	frame0.angle_3d[1]=int16FromDataf(data_buf,&anal_cnt)/100.0f;	
	frame0.angle_3d[2]=int16FromDataf(data_buf,&anal_cnt)/100.0f;	
	
	frame0.quaternion[0]=floatFromDataf(data_buf,&anal_cnt);	
	frame0.quaternion[1]=floatFromDataf(data_buf,&anal_cnt);	
	frame0.quaternion[2]=floatFromDataf(data_buf,&anal_cnt);
	frame0.quaternion[3]=floatFromDataf(data_buf,&anal_cnt);
	
	reverseFromDataf(data_buf,&anal_cnt,4);
	
	frame0.local_time=uint32FromDataf(data_buf,&anal_cnt);
	frame0.system_time=uint32FromDataf(data_buf,&anal_cnt);
	
	reverseFromDataf(data_buf,&anal_cnt,1);
	
	frame0.eop_3d[0]=charFromDataf(data_buf,&anal_cnt)/100.0f;
	frame0.eop_3d[1]=charFromDataf(data_buf,&anal_cnt)/100.0f;
	frame0.eop_3d[2]=charFromDataf(data_buf,&anal_cnt)/100.0f;
	
	frame0.voltage=uint16FromDataf(data_buf,&anal_cnt)/1000.0f;
	
	static int cnt;
	if((frame0.id==2||1)&&en_print){cnt=0;
	//printf("%d %d %d %d %d %d\n",data_buf[0],data_buf[1],data_buf[2],data_buf[3],data_buf[4],data_buf[5]);
	printf("id=%d role=%d bat=%.2f time1=%d time2=%d\n", frame0.id,frame0.role ,frame0.voltage,
	frame0.local_time,frame0.system_time);
	printf("d1=%.3f d2=%.3f d3=%.3f d4=%.3f d5=%.3f d6=%.3f d7=%.3f d8=%.3f\n",
	frame0.dis_arr[0],frame0.dis_arr[1],frame0.dis_arr[2],frame0.dis_arr[3],frame0.dis_arr[4],
	frame0.dis_arr[5],frame0.dis_arr[6],frame0.dis_arr[7]);
	printf("acc0=%.2f acc1=%.2f acc2=%.2f\n", 
	frame0.imu_acc_3d[0],frame0.imu_acc_3d[1],frame0.imu_acc_3d[2]);
	printf("gyro0=%.2f gyro1=%.2f gyro2=%.2f\n", frame0.imu_gyro_3d[0],frame0.imu_gyro_3d[1],frame0.imu_gyro_3d[2]);
	printf("att0=%.2f att1=%.2f att2=%.2f\n", frame0.angle_3d[0],frame0.angle_3d[1],frame0.angle_3d[2]);
	printf("q0=%.2f q1=%.2f q2=%.2f q3=%2f\n", frame0.quaternion[0],frame0.quaternion[1],frame0.quaternion[2],frame0.quaternion[3]);
	printf("posx=%.2f posy=%.2f posz=%.2f\n", frame0.pos_3d[0],frame0.pos_3d[1],frame0.pos_3d[2]);
	printf("velx=%.2f vely=%.2f velz=%.2f\n", frame0.vel_3d[0],frame0.vel_3d[1],frame0.vel_3d[2]);
	printf("eop0=%.2f eop1=%.2f eop2=%.2f\n\n", frame0.eop_3d[0],frame0.eop_3d[1],frame0.eop_3d[2]);
} 
}


unsigned char RxBuffer1[200];
unsigned char RxState1 = 0;
int RxBufferNum1 = 0;
int RxBufferCnt1 = 0;
int _data_len1 = 0;
int _data_cnt1 = 0;
void rx_anal(unsigned char com_data)//Rx interupt
{
	if(RxState1==0&&com_data==0x55)
	{
		RxState1=1;
		RxBuffer1[0]=com_data;
	}
	else if(RxState1==1&&com_data==0x01)
	{
		RxState1=2;
		RxBuffer1[1]=com_data;
		_data_len1 = 128 - 2;
		_data_cnt1 = 0;
	}
	else if(RxState1==2&&_data_len1>0)
	{
		_data_len1--;
		RxBuffer1[2+_data_cnt1++]=com_data;
		if(_data_len1==0)
		  RxState1= 3;
	}
	else if(RxState1==3)
	{
		RxState1 = 0;
		RxBuffer1[2+_data_cnt1]=com_data;
		if (_data_cnt1>0)
		{
			//cout<<"_data_cnt1: "<<_data_cnt1<<endl;
			if (checkcommand(RxBuffer1, _data_cnt1+2))
			{
				decode(RxBuffer1, _data_cnt1+2);
			}
		}
	}
	else
		RxState1 = 0;
}


int main(int argc, char *argv[])
{
	int data_length = 0,fd=0,data_cnt=0;
	unsigned char rx_temp[200]={0};
	ros::init(argc, argv, "uwb_node");
	ros::NodeHandle nh;
	
	//
	ros::Publisher odom_pub = nh.advertise<nav_msgs::Odometry>("uwb_odom", 100); 

	ros::Time current_time, last_time;
	current_time = ros::Time::now();
	last_time = ros::Time::now();

	std::string port_name="/dev/ttyUSB0";
	int baudrate = 115200;
	ros::param::get("~port_name", port_name);
	ros::param::get("~baudrate", baudrate);
	ros::param::get("~en_print", en_print);

	ROS_INFO("serial port name:%s", port_name.c_str());
	ROS_INFO("serial baudrate:%d", baudrate);
	ROS_INFO("en_print:%d", en_print);
	try 
	    { 
	        m_serial.setPort(port_name); 
	        m_serial.setBaudrate(baudrate); 
	        serial::Timeout to = serial::Timeout::simpleTimeout(100); 
	        m_serial.setTimeout(to); 
	        m_serial.open(); 
	    } 
	    catch (serial::IOException& e) 
	    { 
	        ROS_ERROR_STREAM("Unable to open port "); 
	        return -1; 
    	     }	 

	ros::Rate loop_rate(100);
	while (ros::ok())
	{  	

		ros::spinOnce();               // check for incoming messages
    		current_time = ros::Time::now();

		data_length = m_serial.available();
		if (data_length>0)
		{
			//cout<<"data_length: "<<data_length<<endl;
			m_serial.read(rx_temp, data_length);
			if(data_length>0){//push into fifo
				for(int i=0;i<data_length;i++)
			            rx_anal(rx_temp[i]);
			}
		}


		//publish
		/*
		  uint8_t id;
		  uint8_t role;
		  float pos_3d[3];//
		  float vel_3d[3];//
		  float dis_arr[8];//
		  float imu_gyro_3d[3];//
		  float imu_acc_3d[3];
		  uint8_t reserved1[12];//
		  float angle_3d[3];
		  float quaternion[4];//
		  uint8_t reserved2[4];//
		  uint32_t local_time;//
		  uint32_t system_time;//
		  uint8_t reserved3[1];//
		  uint8_t eop_3d[3];
		  uint16_t voltage;
		  uint8_t reserved4[5];
		*/

		nav_msgs::Odometry odom;
    		odom.header.stamp = current_time;
    		odom.header.frame_id = "uwb_odom";
    		odom.pose.pose.position.x = frame0.pos_3d[0];
    		odom.pose.pose.position.y = frame0.pos_3d[1];
    		odom.pose.pose.position.z = frame0.pos_3d[2];
		odom.pose.pose.orientation.x = frame0.quaternion[0];
		odom.pose.pose.orientation.y = frame0.quaternion[1];
		odom.pose.pose.orientation.z = frame0.quaternion[2];
		odom.pose.pose.orientation.w = frame0.quaternion[3];
		odom.pose.covariance.elems[0]= frame0.dis_arr[0];
		odom.pose.covariance.elems[1]= frame0.dis_arr[1];
		odom.pose.covariance.elems[2]= frame0.dis_arr[2];
		odom.pose.covariance.elems[3]= frame0.dis_arr[3];
		odom.pose.covariance.elems[4]= frame0.dis_arr[4];
		odom.pose.covariance.elems[5]= frame0.dis_arr[5];
		odom.pose.covariance.elems[6]= frame0.dis_arr[6];
		odom.pose.covariance.elems[7]= frame0.dis_arr[7];
		odom.pose.covariance.elems[8]= frame0.voltage;
		//set the velocity
		odom.child_frame_id = "base_link";
		odom.twist.twist.linear.x = frame0.vel_3d[0];
		odom.twist.twist.linear.y = frame0.vel_3d[1];
		odom.twist.twist.linear.z = frame0.vel_3d[2];
		odom.twist.twist.angular.x = frame0.imu_gyro_3d[0];
		odom.twist.twist.angular.y = frame0.imu_gyro_3d[1];
		odom.twist.twist.angular.z = frame0.imu_gyro_3d[2];
		odom.twist.covariance.elems[0]= frame0.angle_3d[0];
		odom.twist.covariance.elems[1]= frame0.angle_3d[1];
		odom.twist.covariance.elems[2]= frame0.angle_3d[2];
		odom.twist.covariance.elems[3]= frame0.imu_acc_3d[0];
		odom.twist.covariance.elems[4]= frame0.imu_acc_3d[1];
		odom.twist.covariance.elems[5]= frame0.imu_acc_3d[2];
		odom.twist.covariance.elems[6]= frame0.eop_3d[0];
		odom.twist.covariance.elems[7]= frame0.eop_3d[1];
		odom.twist.covariance.elems[8]= frame0.eop_3d[2];

		odom_pub.publish(odom);
		ros::spinOnce(); 
		loop_rate.sleep();
	}
	cout << "serial quit" << endl;
	return 0;
}
