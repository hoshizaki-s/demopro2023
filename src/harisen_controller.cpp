#include <ros/ros.h>
#include <std_msgs/Int32.h>
#include "std_msgs/String.h"
#include "dynamixel_sdk_examples/GetPosition.h"
#include "dynamixel_sdk_examples/SetPosition.h"
#include "dynamixel_sdk/dynamixel_sdk.h"
#include <sound_play/SoundRequest.h>


#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Quaternion.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_broadcaster.h>

#include <map>
#include <math.h>

#define ARM_CLOSE 1000
#define ARM_OPEN 0
#define MAX_VEL 0.15
#define MAX_OMEGA 0.05

#define WAKE_WORDS "/home/hoshizaki-s/workspace/src/demopro2023/start.wav"
#define ATTACK_WORDS "/home/hoshizaki-s/workspace/src/demopro2023/attack.wav"
#define MOVE_WORDS "/home/hoshizaki-s/workspace/src/demopro2023/move.wav"
#define END_WORDS "/home/hoshizaki-s/workspace/src/demopro2023/end.wav"

using namespace dynamixel;

bool flag1_ = false;
//bool flag2_ = false;
bool end_flag_ = false;
ros::Publisher set_position_pub_;
ros::Subscriber first_flag_sub_;
ros::Subscriber second_flag_sub_; 

ros::Publisher twist_pub_, sound_pub_;


// オドメトリから得られる現在の位置と姿勢
double robot_x, robot_y;
double roll, pitch, yaw;
geometry_msgs::Quaternion robot_r;

geometry_msgs::Twist twist; // 指令する速度、角速度
geometry_msgs::PoseStamped goal; // 目標地点

// オドメトリのコールバック
void odom_callback(const nav_msgs::Odometry::ConstPtr &msg)
{
	robot_x = msg->pose.pose.position.x;
	robot_y = msg->pose.pose.position.y;
	robot_r = msg->pose.pose.orientation;
}

// クォータニオンをオイラーに変換                                               
void geometry_quat_to_rpy(double &roll, double &pitch, double &yaw, geometry_msgs::Quaternion geometry_quat)
{
    tf::Quaternion quat;
    quaternionMsgToTF(geometry_quat, quat);
    tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);
}

//　goalで指定した位置に近いかの判定を行う
int near_position(geometry_msgs::PoseStamped fgoal)
{
	double difx = robot_x - fgoal.pose.position.x;
	double dify = robot_y - fgoal.pose.position.y;
	return (sqrt(difx * difx + dify * dify) < 0.2);
}


void go_position(geometry_msgs::PoseStamped fgoal)
{
    double k_v = 0.8; // 速度の係数	0.1
    double k_w = 0.5; // 角速度の係数
	
	// 指令する速度と角速度
	double v = 0.0;
	double w = 0.0;

	//　角速度の計算
	double theta = atan2(fgoal.pose.position.y - robot_y, fgoal.pose.position.x - robot_x);
	while (theta <= -M_PI || M_PI <= theta)
	{
		if (theta <= -M_PI)
			theta = theta + 2 * M_PI;
		else
			theta = theta - 2 * M_PI;
	}

	// 現在のロボットのroll, pitch, yawを計算
	geometry_quat_to_rpy(roll, pitch, yaw, robot_r);

	while (yaw <= -M_PI || M_PI <= yaw)
	{
		if (yaw <= -M_PI)
			yaw = yaw + 2 * M_PI;
		else
			yaw = yaw - 2 * M_PI;
	}

	theta = theta - yaw; //thetaに目標とする点に向くために必要となる角度を格納 角度の差を取る処理は関数化したほうが良い。

	while (theta <= -M_PI || M_PI <= theta)
	{
		if (theta <= -M_PI)
			theta = theta + 2 * M_PI;
		else
			theta = theta - 2 * M_PI;
	}

	w = k_w * theta;

    v = k_v * (fgoal.pose.position.x - robot_x);

	// 速度の計算(追従する点が自分より前か後ろかで計算を変更)	クリッピングも関数化したほうが良い。
	if ((theta <= (M_PI / 2)) && (theta >= (-M_PI / 2)))
		w = k_w * theta;
	else
		w = -k_w * theta;
	
    if(abs(v) > MAX_VEL){
        v = MAX_VEL*v/abs(v);
    }
    if(abs(w) > MAX_OMEGA){
        w = MAX_VEL*w/abs(w);
    }

	// publishする値の格納
	twist.linear.x = v;
	twist.linear.y = 0.0;
	twist.linear.z = 0.0;
	twist.angular.x = 0.0;
	twist.angular.y = 0.0;
	twist.angular.z = w;

    twist_pub_.publish(twist);  
	//std::cout << "v: " << v << ", w: " << w << std::endl;

}

void flag1_callback(const std_msgs::Int32 flag){
    std::cout << "call back" << std::endl;
    flag1_ = (1==flag.data);
    std::cout << flag.data << std::endl;
}
// void flag2_callback(const std_msgs::Int32 flag){
//     flag2_ = (1==flag.data);
// }

int main(int argc, char ** argv){
    ros::init(argc, argv, "harisen_controller");
    ros::NodeHandle nh_;
    set_position_pub_ = nh_.advertise<dynamixel_sdk_examples::SetPosition>("/set_position", 10);
    first_flag_sub_ = nh_.subscribe("/first_flag", 10, flag1_callback);
    //second_flag_sub_ = nh_.subscribe("/second_flag", 10, flag2_callback);

    ros::Subscriber odom_sub = nh_.subscribe("ypspur_ros/odom", 100, odom_callback);
	twist_pub_ = nh_.advertise<geometry_msgs::Twist>("ypspur_ros/cmd_vel", 1000);
    sound_pub_ = nh_.advertise<sound_play::SoundRequest>("/robotsound", 10);
    
    sound_play::SoundRequest sound_request;
    sound_request.sound = -2;
    sound_request.command = 1;
    sound_request.volume = 1.0;
    sound_request.arg = WAKE_WORDS;

    // odometryの値の初期化
	robot_x = 0.0;
	robot_y = 0.0;
	robot_r.x = 0.0;
	robot_r.y = 0.0;
	robot_r.z = 0.0;
	robot_r.w = 1.0;

    dynamixel_sdk_examples::SetPosition msg;
    msg.id = 1;
    msg.position = ARM_CLOSE;
    set_position_pub_.publish(msg);
    sleep(1);
    set_position_pub_.publish(msg);

    goal.pose.position.x = 0.0;
    goal.pose.position.y = 0.0;
    

    while(ros::ok()){
        ros::spinOnce();
        go_position(goal);
        if (near_position(goal)){
            twist.linear.x = 0.0;
            twist.angular.z = 0.0;
            twist_pub_.publish(twist);
            sleep(10);   
            break;
        } 
    }
    
    sound_pub_.publish(sound_request);
    goal.pose.position.x = 1.0;
    goal.pose.position.y = 0.0;

    while(ros::ok()){
        if(end_flag_)break;
        ros::spinOnce();
        if(flag1_ ){
        //&& flag2_){
            sound_request.arg = MOVE_WORDS;
            sound_pub_.publish(sound_request);
            sleep(2);

            while(ros::ok()){
                ros::spinOnce();
                go_position(goal);
                if (near_position(goal)){
                    twist.linear.x = 0.0;
                    twist.angular.z = 0.0;
                    twist_pub_.publish(twist);   
                    break;
                } 
            }

            sound_request.arg = ATTACK_WORDS;
            sound_pub_.publish(sound_request);
            dynamixel_sdk_examples::SetPosition msg;
            msg.id = 1;
            msg.position = ARM_OPEN;
            set_position_pub_.publish(msg);
            sleep(2);
            msg.id = 1;
            msg.position = ARM_CLOSE;
            set_position_pub_.publish(msg);
            end_flag_ = true;
            break;
        }
    }

    sound_request.arg = END_WORDS;
    sound_pub_.publish(sound_request);
    return 0;

}