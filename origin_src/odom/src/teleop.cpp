#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/Joy.h>


class Teleop
{
  public:
    Teleop();

  private:
    void joyCallback(const sensor_msgs::Joy::ConstPtr& joy);

    ros::NodeHandle nh_;

    int linear_, angular_,transverse_;    // 手柄上的轴号（分别表示用哪个轴控制前后移动、旋转以及横向运动）
    double l_scale_, a_scale_, t_scale_;  // 速度比例系数
    
    ros::Publisher vel_pub_;
    ros::Subscriber joy_sub_;
};


Teleop::Teleop():linear_(1),angular_(0),transverse_(2)
{
  // param()函数从参数服务器取参数值给变量。如果无法获取，则将默认值赋给变量
  nh_.param("axis_linear", linear_, linear_);
  nh_.param("axis_angular", angular_, angular_);
    nh_.param("axis_transverse", transverse_, transverse_);
  nh_.param("scale_angular", a_scale_, a_scale_);
  nh_.param("scale_linear", l_scale_, l_scale_);
    nh_.param("scale_transverse", t_scale_, t_scale_);

  vel_pub_ = nh_.advertise<geometry_msgs::Twist>("cmd_vel", 1);
  joy_sub_ = nh_.subscribe<sensor_msgs::Joy>("joy", 10, &Teleop::joyCallback, this);
}


void Teleop::joyCallback(const sensor_msgs::Joy::ConstPtr& joy)
{
  geometry_msgs::Twist twist;
  
  // 发布的机器人速度等于joy数据乘以速度比例系数
  // 比如手柄X轴向前推到最大时为1.0,速度比例系数为0.4,则机器人最大前进速度为0.4m/s
  twist.linear.x = l_scale_*joy->axes[linear_];
    twist.linear.y = t_scale_*joy->axes[transverse_];
  twist.angular.z = a_scale_*joy->axes[angular_];

  vel_pub_.publish(twist);  // 发布机器人速度信息
}


int main(int argc, char** argv)
{
  ros::init(argc, argv, "teleop_joy");
  Teleop teleop_turtle;

  ros::spin();
}
