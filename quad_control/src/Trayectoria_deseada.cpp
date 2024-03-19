#include <ros/ros.h>
#include <geometry_msgs/Vector3.h>
#include <cmath>

double s=0;
int main(int argc, char** argv) {
    ros::init(argc, argv, "desired_position_publisher");
    ros::NodeHandle nh;

    ros::Publisher desired_vlin_pub = nh.advertise<geometry_msgs::Vector3>("desired_vlin", 100);
    ros::Publisher desired_vang_pub = nh.advertise<geometry_msgs::Vector3>("desired_vang", 100);

    ros::Rate loop_rate(100);  // Frecuencia de publicación
  

    while (ros::ok()) {
        geometry_msgs::Vector3 desired_vlin;
        geometry_msgs::Vector3 desired_vang;
        double t = s/100;    

        if (t <= 5) {
            desired_vlin.x = 0;
            desired_vlin.y = 0;
            desired_vlin.z = -0.5;
            
            desired_vang.x = 0;
            desired_vang.y = 0;
            desired_vang.z = 0;
            

        } else if (t <= 65) {
            double x_des = 0.5 * sin(0.1 * (t - 5));
            double y_des = 0.5 * cos(0.1 * (t - 5));
            desired_vlin.x = x_des;
            desired_vlin.y = y_des;
            desired_vlin.z = 0;
            
            desired_vang.x = 0;
            desired_vang.y = 0;
            desired_vang.z = 0.1;
            
            
        }
        

       
        desired_vang_pub.publish(desired_vang);
        desired_vlin_pub.publish(desired_vlin);
        ros::spinOnce();
        loop_rate.sleep();
        s++;

    }

    return 0;
}
