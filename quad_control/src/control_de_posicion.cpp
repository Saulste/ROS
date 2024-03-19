#include <ros/ros.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Pose.h>
#include <cmath>
#include <eigen3/Eigen/Dense>
#include "gazebo_msgs/ModelState.h"

// Constantes del controlador PD
const double kp = 0.1;
const double kd = 1; 
float pos_x;
float pos_y;
float pos_z;

float   pos_wx = 0;
float	pos_wy = 0;
float	pos_wz = 0;

float pos_xa;
float pos_ya;
float pos_za;
// Variables globales para almacenar la posición actual y deseada
geometry_msgs::Vector3 current_pos;
geometry_msgs::Vector3 desired_vel;
geometry_msgs::Vector3 desired_vang; 
geometry_msgs::Vector3 ang_pos;
geometry_msgs::Vector3 error_l;
// Callback para la posición actual del drone
void pos_callback(const geometry_msgs::Vector3::ConstPtr& s) 
{
    pos_xa = s->x;
	pos_ya = s->y;
	pos_za = s->z;
}

// Callback para la posición deseada
void desired_vlin_callback(const geometry_msgs::Vector3::ConstPtr& p) 
{
    pos_x = p->x;
	pos_y = p->y;
	pos_z = p->z;
} 
// Callback para la Velocidad angular de yaw DESEADA
void ang_callback(const geometry_msgs::Vector3::ConstPtr& as)
{
    pos_wx = as->x;
	pos_wy = as->y;
	pos_wz = as->z;
} 
// Callback para la pocision angular 
void att_callback(const geometry_msgs::Vector3::ConstPtr& msg)
{
    ang_pos = *msg;
}


int main(int argc, char** argv) {
    ros::init(argc, argv, "linear_position_controller");
    ros::NodeHandle nh; 
    //Valores viejos 
    float xv = 0; 
    float yv = 0; 
    float zv = 0;  
    
    //Velocidades 
    float x_punto = 0;
    float y_punto = 0;
    float z_punto = 0;    

    //Pocision deseada 
    float xd = 0;
    float yd = 0;
    float zd = 0;
    //errores 
    float ex = 0; 
    float ey = 0;
    float ez = 0;
    float ex_punto = 0;
    float ey_punto = 0; 
    float ez_punto = 0;
    //U's
    float Uvx = 0;
    float Uvy = 0; 
    float Uvz = 0; 
    //Thrust
    float th = 0; 
    //Masa 
    int m = 2;  
    //Velocidad  Angular Deseada 
    float rolld = 0;  
    float pitchd = 0;

    //Velocidad angular deseada de yaw 
    float yawd =0; 
    //Velocidad angular 
    float roll_punto = 0; 
    float pitch_punto = 0;
    float yaw_punto = 0; 
    
    //Publishers pos angular deseado
    geometry_msgs::Vector3 quad_position;
    geometry_msgs::Vector3 ang_des;
    geometry_msgs::Vector3 th_;
    ros::Publisher vlin_pub = nh.advertise<geometry_msgs::Vector3>("quad_position", 100);
    ros::Publisher ang_des_pub = nh.advertise<geometry_msgs::Vector3>("ang_des", 100);
    ros::Publisher el_pub = nh.advertise<geometry_msgs::Vector3>("errore_l", 100);
    ros::Publisher th_pub = nh.advertise<geometry_msgs::Vector3>("th_", 100);
   
    // Subscripciones a los tópicos de posición actual , deseada en velocidad lineal/angular y pocision angular
        
    ros::Subscriber current_pos_sub = nh.subscribe("quad_position",100, &pos_callback);
    ros::Subscriber desired_vel_sub = nh.subscribe("desired_vlin", 100, &desired_vlin_callback);
    ros::Subscriber pang_vel_sub = nh.subscribe("desired_vang", 100, &ang_callback);
   


    ros::Rate loop_rate(100);

    while (ros::ok()) 
    { 

        yawd =yawd+(pos_wz*.01); 
        //Velocidades deseadas
        float xd_punto = pos_x;
        float yd_punto = pos_y;
        float zd_punto = pos_z;
        //pocision actuales de current_pos 
        float xa = pos_xa; 
        float ya = pos_ya;
        float za = pos_za; 
        // pocision vieja actualizada
        xv = xv+xa;  
        yv = yv+ya;
        zv = zv+za; 
        //velocidades
        x_punto = (xa-xv)/.01;
        y_punto = (ya-yv)/.01;
        z_punto = (za-zv)/.01;   

        // Integrando velocidad deseada para pocision deseada
        xd = xd+(xd_punto*(.01));
        yd = yd+(yd_punto*(.01));
        zd = zd+(zd_punto*(.01));
        //errores lineales
        ex = xa-xd; 
        ey = ya-yd; 
        ez = za-zd; 
        //errores puntos
        ex_punto = x_punto-xd_punto;
        ey_punto = y_punto-yd_punto; 
        ez_punto = z_punto-zd_punto;
        //Control  Pd   
        //Uvx = (-2*ex)-(10*ex_punto);
        //Uvy = (-3*ey)-(10*ey_punto); 
        //Uvz = (-2*ez)-(10*ez_punto);  

        //Pocision Angular 
        float roll = ang_pos.x;
        float pitch = ang_pos.y;
        float yaw = ang_pos.z; 
        //Thrust calculo


        th = (m/(cos(roll)*cos(pitch)))*(-9.8+Uvz); 
        float Uvx = (th / m) * ((sin(roll) * sin(yaw)) + (cos(roll) * cos(yaw) * sin(pitch)));
        float Uvy = (th / m) * ((cos(roll) * sin(yaw) * sin(pitch)) - (cos(yaw) * sin(roll)));
        //Posicion angular deseada
        rolld = asin((m/th)*((sin(yawd)*(Uvx))-(cos(yawd)*Uvy))); 
        pitchd = asin(((m/th)*Uvx-(sin(yawd)*sin(rolld)))/(cos(yawd)*cos(rolld)));
        th_.z=th;
        th_pub.publish(th_);

        double xf = th * sin(roll);
        double yf = th * sin(pitch);

        quad_position.x = xd;
        quad_position.y = yd;
        quad_position.z = zd;
        vlin_pub.publish(quad_position);

        error_l.x = ex;
        error_l.y = ey;
        error_l.z = ez;
        el_pub.publish(error_l);

        ang_des.x = rolld;
        ang_des.y = pitchd;
        ang_des.z = yawd;
        ang_des_pub.publish(ang_des);
        
        ros::spinOnce();     
        loop_rate.sleep();


    }

    return 0;
}
