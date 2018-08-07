/*订阅 cmd_vel 
*将角速度和线速度转换成轮速和舵机角度
*上报里程计数据
*通过car_if 发送控制命令，读取imu数据和车轮行程
*根据车轮行程,舵机角度，以及IMU 数据计算odom数据
*/
#include "ros/ros.h"
#include <geometry_msgs/Twist.h>

//#include "boost/thread/mutex.hpp"
#include "nodelet/nodelet.h"

#include "message_filters/subscriber.h"
//#include <pointcloud_to_laserscan/pointcloud_to_laserscan_nodelet.h>
#include <pluginlib/class_list_macros.h>

#include <tf/transform_broadcaster.h>

#include "car_if.h"

#include <nav_msgs/Path.h>
#include <std_msgs/String.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/PoseStamped.h>


 #define KEY_TOPIC   "/cmd_vel"
 
#define MAX_ANGLE_VALUE 24*PI_VALUE/180
#define PI_VALUE 3.1415926

float linear_speed=0.0;
float angular_speed=0.0;
float angular_aw=0;

#define MAX_FILTER_COUNT 10
float angular_list[MAX_FILTER_COUNT];

float G_current_speed=0.0;
float G_current_ln_acc =0.0;
float G_current_angle=0.0;

//float angular_aw=0.0;

float g_position_x=5.0;
float g_position_y=10.0;
float g_angular =0.0;
char calculate_dir = 0;
Car_interface *car_if; 

unsigned int G_walks=0;

// 小车前后轮的距离15.7cm
float dc = 15.7;
//小车轮子到中心轴的距离8.0cm
float dw = 8.0;
#define MAX_TRY_TIMES 10


#define FAKE_HW_DATA 
#define FAKE_IMU
#ifdef FAKE_HW_DATA
float fake_walk_acc = 3;
#endif


#ifdef FAKE_IMU
float fake_angle_acc =0;
#endif

float read_imu_data(void)
{
	int err = -1;
	int try_times=0;
	float rotation=0;
#ifdef FAKE_IMU
	return fake_angle_acc;
#else
	while(err){
		if(car_if)
	        err=car_if->get_rotation_data(&rotation);
		
		try_times++;
		if(try_times > MAX_TRY_TIMES)
			break;
	}
	if(err < 0){
		ROS_ERROR("get_rotation_data error! %f",rotation);
		return g_angular;
	}
	ROS_ERROR("read imu data is :%f \n",rotation);
	return (2*PI_VALUE - (rotation*PI_VALUE/180));	
#endif
}


float angular_filter(float angular)
{
	float ret_anglular=0;
	for(int i=0;i<(MAX_FILTER_COUNT-1);i++){
		angular_list[i] = angular_list[i+1];
		ret_anglular+=angular_list[i];
	} 
	angular_list[MAX_FILTER_COUNT-1] = angular;
	ret_anglular+= angular;
	return (ret_anglular/MAX_FILTER_COUNT);
}

unsigned int  read_walk_counter(void)
{
	int err = -1;
	unsigned int walks;
	int try_times=0;
#ifdef FAKE_HW_DATA
	return G_walks+fake_walk_acc;
	//return 0;
#else
	while(err){
		if(car_if)
	    err=car_if->get_walk_count_data(&walks);
		try_times++;
		if(try_times > MAX_TRY_TIMES)
			break;
	}
	
	if((err < 0)||(walks < 0)){
	ROS_ERROR("get_odom_data error! %d",walks);
	return G_walks;
	}
	return walks;
#endif
}
//每一步的距离 2.2 cm
#define WALK_COUNT_UNIT  1.1 
int calculate_odom()
{
	unsigned int walks=0;
	unsigned int walks_temp=0;
	unsigned char dir  = calculate_dir;
	unsigned int angular_temp=0;
	int err = -1;
	int try_times=0;
	unsigned int v_temp=0;
	unsigned char buf[2];

	float angle = PI_VALUE/2 - angular_aw;
	float req_r=0;
	float R0_t=0;
	float L0 = 0; // cm
	float L = 0.0;
	float L_n=0;


#ifdef FAKE_HW_DATA
	walks_temp = (G_walks+fake_walk_acc);
	if(walks_temp >= 255)
		walks_temp = walks_temp -255;
#ifdef FAKE_IMU
	angular_temp = fake_angle_acc;
#endif
#else
	while(err){
		if(car_if)
	    	err=car_if->simple_read(buf);

		try_times++;
		if(try_times > MAX_TRY_TIMES)
			break;
	}
	v_temp = buf[1];
	angular_temp = v_temp* 2;
	walks_temp = buf[0];
#endif


	if(walks_temp >= G_walks)
	walks = walks_temp - G_walks;
	else
	walks = 0xff - G_walks + walks_temp; 


	G_walks = walks_temp;

	if((walks < 0)||(walks > 30)){
		ROS_ERROR("WALKS ERROR! %d \n",walks);
		walks =0;
		return 0;
	}

	L0 = walks*WALK_COUNT_UNIT; // cm


//角度为正，速度为正
  
  //已知角度，求轨迹半径:
  //(tan(@)*dc) * (tan(@)*dc) = r*r + dc*dc/4
  // (tan(@)*dc) * (tan(@)*dc) - dc*dc/4 = r*r;
  
   req_r = sqrt(tan(angle)*dc*tan(angle)*dc + dc*dc/4);

   ROS_ERROR("calculate_odom req_r %u walks %u  G_walks %u angle %f L0 %f \n",req_r,walks,G_walks,angle,L0);

  //求R0
  //R0 = tan(angle)*dc – dw
   R0_t = tan(angle)*dc - dw;

   if(R0_t >10000)
   	R0_t = 0;

  //求车心所经过的距离L
  //L/L0 = req_r/R0
  if(R0_t==0)
  	L = L0;
  else
  L = (req_r*L0)/R0_t;

  //求小车经过的角度
  //周长C = 2PIr
  //走过的角度 n = ( L/C ) * 2PI
	if(R0_t==0)
		L_n = 0;
	else
	L_n = (L/(2*req_r));  //

  float  x =  sin(L_n)*req_r;
  float  y = req_r - cos(L_n)*req_r;
  float raw_x = 0.0;
  float raw_y = 0.0;

  if(R0_t==0)
 	 x=L;

//坐标系旋转 一个导航角后再将增量累加上去。
//	ROS_ERROR("calculate_odom R0_t %f Ln = %f x= %f y=%f \n",R0_t,L_n,x,y);
	raw_x = x*cos(g_angular) + y*sin(g_angular);
// g_position_x +=cos(g_angular)*d_temp/100;  
	raw_y = x*sin(g_angular)- y*cos(g_angular) ;
//	g_position_y +=sin(g_angular)*d_temp/100;
	
    if((angular_aw >= 0)&&(dir == 0)) {
		g_position_x+=raw_x/100;
		g_position_y+=raw_y/100;
#ifdef FAKE_IMU
		fake_angle_acc += L_n;
#else
	G_current_ln_acc = L_n;
#endif
	} else if((angular_aw >= 0)&&(dir == 1))  {
		g_position_x-=raw_x/100;
		g_position_y-=raw_y/100;
#ifdef FAKE_IMU
		fake_angle_acc -= L_n;
#else
	G_current_ln_acc = -L_n;
#endif

	}else if((angular_aw < 0)&&(dir == 0))  {
		g_position_x-=raw_x/100;
		g_position_y-=raw_y/100;
#ifdef FAKE_IMU
		fake_angle_acc +=L_n;
#else
	G_current_ln_acc = L_n;
#endif

	}else if((angular_aw < 0)&&(dir == 1))  {
		g_position_x+=raw_x/100;
		g_position_y+=raw_y/100;
#ifdef FAKE_IMU
		fake_angle_acc -=L_n;
#else
		G_current_ln_acc = -L_n;
#endif
	}

	g_angular = (2*PI_VALUE - (angular_temp*PI_VALUE/180));
	g_angular = angular_filter(g_angular);
  //ROS_ERROR("calculate_odom raw_x=%f raw_y=%f g_position_x %f g_position_y %f g_angular %f \n",raw_x,raw_y,g_position_x,g_position_y,g_angular);
  return 0;
}
//1.57 0 4.71
static void update_odom(void)
{

    calculate_odom();
    ROS_ERROR("minicar report angular %f \n",g_angular);
	
}

//c = 2pir
//v_l = 2pir/t
//v_a = 2pi/t
//v_l = r*v_a
//r = v_l/v_a


//底层的速度单位为cm/s
void set_linear_speed(float l_speed)
{
	int err = -1;
	int try_times = 0;
	int speed_out = (int)(l_speed*100);
	unsigned char speed_temp;
	if(G_current_speed == l_speed)
	{
		ROS_ERROR("speed no change!\n");
		return;
	}	
	
	ROS_ERROR("set_linear_speed %d cm/s !\n",speed_out);

	if(speed_out < 0){
		calculate_dir = 1;
		speed_out = -speed_out;
	}
	else
		calculate_dir = 0;

#ifdef FAKE_HW_DATA
	fake_walk_acc = speed_out / 2.2;
#else
	speed_temp = speed_out;
	ROS_ERROR("set_linear_speed %d  %d cm/s !\n",speed_out,speed_temp);

	while(err){
		if(car_if)
			err=car_if->set_speed((char)(speed_temp));
		try_times++;
		ROS_ERROR("set speed %d\n",speed_temp);
		if(try_times > MAX_TRY_TIMES)
		break;
	}
	if(err)
	ROS_ERROR("set_speed error!");
	
	err =-1;
	while(err){
		if(car_if)
			err=car_if->set_direction(calculate_dir);
		try_times++;
		if(try_times > MAX_TRY_TIMES)
			break;
	}

	if(err)
	ROS_ERROR("set_dir error!");
	else
	G_current_speed = l_speed;
#endif	
	
}

void set_wheel_angle(float angle)
{
	int err = -1;
	int try_times=0;
	int angle_out = (int)(angle*180/PI_VALUE);

	if(G_current_angle == angle)
	{
		ROS_ERROR("angle not change !\n");
	}
	angle_out = 45 - angle_out ;

	while(err){
	if(car_if)
	err=car_if->set_wheel_angle((char)(angle_out));
	try_times++;
	if(try_times > MAX_TRY_TIMES)
	break;
	}

	if(err)
	ROS_ERROR("set_wheel_angle error!");
	else
	G_current_angle = angle;
}


void chatterCallback(const geometry_msgs::Twist& twistMsg)
{
	float r = 0.0;
	float aw = 0.0;
	int angle_out = 0;
	linear_speed=twistMsg.linear.x;
	angular_speed=twistMsg.angular.z;
//角速度为0时:
	if(angular_speed == 0)
	{
		angular_aw = 0;
	} else if((angular_speed != 0)&&(linear_speed == 0)){ 
		//线速度为0时无法转弯，需要强制给一个速度
		linear_speed = 0.5;
		if(angular_speed>0)
			angular_aw =  (MAX_ANGLE_VALUE*PI_VALUE/180);
		else
			angular_aw = -(MAX_ANGLE_VALUE*PI_VALUE/180);
		ROS_ERROR("force set linear speed 0.5 %f \n",angular_aw);
	} else {
		//线速度角速度都不为0
		r = linear_speed/angular_speed;
	
		angular_aw  = PI_VALUE/2 - atan(sqrt(r*r + dc*dc/4)/dc);

		if(angular_speed < 0)
			angular_aw = -angular_aw;

		
		angle_out = (int)(angular_aw*180/PI_VALUE);
		
		if(angle_out > MAX_ANGLE_VALUE){
			ROS_ERROR("set_angle error angle %d !\n",angle_out);
			angle_out = MAX_ANGLE_VALUE;
		} else if (angle_out < -MAX_ANGLE_VALUE){
			angle_out = -MAX_ANGLE_VALUE;
		}
		angular_aw = (angle_out*PI_VALUE/180);
	}

	ROS_ERROR("chatterCallback %f %f %f %f \n",linear_speed,angular_speed,r,angular_aw);
	set_linear_speed(linear_speed);
	set_wheel_angle(angular_aw);
}



int main(int argc, char **argv)
{
	ros::init(argc, argv, "minicar");
	ros::NodeHandle n;
	ros::Publisher path_pub = n.advertise<nav_msgs::Path>("trajectory",1, true);
    ros::Time current_time;
	
	geometry_msgs::TransformStamped odom_trans;
	tf::TransformBroadcaster broadcaster;
	
	odom_trans.header.frame_id = "odom";
	odom_trans.child_frame_id = "base_link";


	nav_msgs::Path path;
	path.header.stamp=current_time;
	path.header.frame_id="odom";

#ifdef FAKE_HW_DATA
#else
	car_if = new Car_interface("/dev/ttyUSB0");
#endif		
  ros::Subscriber sub = n.subscribe(KEY_TOPIC, 1000, chatterCallback);
  ROS_INFO("minicar start \n");
  ros::Rate loop_rate(1);
 float angle_temp=0;
  while (ros::ok())
  {
   	update_odom();
	current_time = ros::Time::now();
	odom_trans.header.stamp = current_time;
	odom_trans.transform.translation.x = g_position_x;
	odom_trans.transform.translation.y = g_position_y;
	odom_trans.transform.translation.z = 0.0;
	angle_temp = g_angular;
	odom_trans.transform.rotation = tf::createQuaternionMsgFromYaw(angle_temp);
	broadcaster.sendTransform(odom_trans);


	geometry_msgs::PoseStamped this_pose_stamped;
	this_pose_stamped.pose.position.x = g_position_x;
	this_pose_stamped.pose.position.y = g_position_y;

	geometry_msgs::Quaternion goal_quat = tf::createQuaternionMsgFromYaw(angle_temp);
	this_pose_stamped.pose.orientation.x = goal_quat.x;
	this_pose_stamped.pose.orientation.y = goal_quat.y;
	this_pose_stamped.pose.orientation.z = goal_quat.z;
	this_pose_stamped.pose.orientation.w = goal_quat.w;

	this_pose_stamped.header.stamp=current_time;
	path_pub.publish(path);
	this_pose_stamped.header.frame_id="odom";
	path.poses.push_back(this_pose_stamped);
	
  	ros::spinOnce();
	loop_rate.sleep();
  }
  set_linear_speed(0);
    return 0;
}

