#include <ros/ros.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/Vector3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h> 

// Constantes del controlador PD
const double Kp_angular = 1.0;
const double Kd_angular = 0.1;
// variables globales
double jxx=0.0411; 
double jyy=0.0478;
double jzz=0.0599;
float ang_dx=0;
float ang_dy=0;
float ang_dz=0;
double pos_xn = 0;
double pos_yn = 0;
double pos_zn = 0;
//Vectores
geometry_msgs::Vector3 ang_posd;
geometry_msgs::Vector3 ang_pos;
geometry_msgs::Vector3 quad_attitude;
geometry_msgs::Vector3 torques;
geometry_msgs::Vector3 errory;
//Callbacks
void ang_callback(const geometry_msgs::Vector3::ConstPtr& az)
{
    ang_dx=az->x;
    ang_dy=az->y;
    ang_dz=az->z;
}
void att_callback(const geometry_msgs::Vector3::ConstPtr& ag)
{
    pos_xn = ag->x;
	pos_yn = ag->y;
	pos_zn = ag->z;
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "angular_position_controller");
    ros::NodeHandle nh;
    //Roll pitch desados viejos
    float rollvd = 0;
    float pitchvd = 0;
    float yawvd = 0; 
    //Roll pitch yaw viejos
    float rollv = 0;
    float pitchv = 0;
    float yawv = 0;  
    //dvelocidad angulo deseado
    float rolld_punto = 0; 
    float pitchd_punto = 0;
    float yawd_punto = 0;  
    //velocidad angular 
    float roll_punto = 0;
    float pitch_punto = 0;
    float yaw_punto = 0;  
    //torques 
    float troll = 0;
    float tpitch = 0;
    float tyaw = 0;
    // K
    int kp = .1;
    int kd = 1;
    // Subscripciones a los tópicos de orientación actual y deseada
    ros::Subscriber att_sub = nh.subscribe("quad_attitude", 100, att_callback);
    ros::Subscriber sub = nh.subscribe<geometry_msgs::Vector3>("ang_des", 100, &ang_callback);

    // Publicación de comandos de velocidad angular
    ros::Publisher tor_pub = nh.advertise<geometry_msgs::Vector3>("quad_attitude", 100);
    ros::Publisher t_pub = nh.advertise<geometry_msgs::Vector3>("torques", 100);
    ros::Publisher ey_pub = nh.advertise<geometry_msgs::Vector3>("errory", 100);

    ros::Rate loop_rate(100);

    while (ros::ok()) {
    //Pocision angular deseada
    float rolld = ang_dx;
    float pitchd = ang_dy;
    float yawd = ang_dz;  
    //Velocidad angular deseada
    rolld_punto = (rolld-rollvd)/.01; 
    pitchd_punto = (pitchd-pitchvd)/.01;
    yawd_punto = (yawd-yawvd)/.01; 
    //Pocision angular vieja deseada actualizada 
    rollvd = rollvd + rolld; 
    pitchvd = pitchvd + pitchd;
    yawvd = yawvd + yawd; 
    //Pocison angular
    float roll = ang_pos.x;
    float pitch = ang_pos.y;
    float yaw = ang_pos.z;
    //velocidad angular 
    roll_punto = (roll-rollv)/.01;
    pitch_punto = (pitch-pitchv)/.01;
    yaw_punto = (yaw-yawv)/.01;  
    //Pocision angular vieja  actualizada 
    rollv = rollv + roll; 
    pitchv = pitchv + pitch;
    yawv = yawv + yaw; 
    //errores 
    float eroll = roll - rolld;
    float erollp  = roll_punto - rolld_punto;
    float epitch = pitch- pitchd;
    float epitchp = pitch_punto - pitchd_punto;
    float eyaw = yaw - yawd;
    float eyawp = yaw_punto - yawd_punto;
    //Contol 
    float uroll = -kp*eroll -kd*erollp;
    float uyaw  = -kp*epitch -kd*epitchp;
    float upitch = -kp*yaw -kd*eyawp;
 
    //torques 
    troll = (jxx*(((jzz-jyy)/jxx)*(pitch_punto*yaw_punto)+uroll)); 
    tpitch = (jyy*(((jxx-jzz)/jyy)*(roll_punto*yaw_punto)+upitch)); 
    tyaw = (jzz*(((jyy-jxx)/jzz)*(pitch_punto*roll_punto)+uyaw)); 

    torques.x = rolld;
    torques.y = pitchd;
    torques.z = yawd;
    t_pub.publish(torques);

    errory.z = eyaw;
    ey_pub.publish(errory);



    quad_attitude.x = rolld;
    quad_attitude.y = pitchd;
    quad_attitude.z = yawd;
    tor_pub.publish(quad_attitude);
    

    ros::spinOnce();
    loop_rate.sleep();
    }

    return 0;
}
