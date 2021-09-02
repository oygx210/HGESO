#include <eigen3/Eigen/Dense>
#include <iostream>
#include <ros/ros.h>
#include <math.h>
#include "geometry_msgs/Vector3.h"
#include "std_msgs/Float64.h"

float x_des;
float y_des;
float z_des;
float psi_des;

float x_vel_des;
float y_vel_des;
float z_vel_des;

float roll_vel_des;
float pitch_vel_des;
float yaw_vel_des;

int main(int argc, char **argv)
{
	ros::init(argc, argv, "HGESO_references");
	ros::NodeHandle nh;
	ros::Rate loop_rate(100);
	
	geometry_msgs::Vector3 positionRefVector;
	geometry_msgs::Vector3 velocityRefVector;
	geometry_msgs::Vector3 att_velRefVector;
	std_msgs::Float64 psiReference;
	
	ros::Publisher posRefpub = nh.advertise<geometry_msgs::Vector3>("pos_ref",100);
	ros::Publisher velRefpub = nh.advertise<geometry_msgs::Vector3>("vel_ref",100);
	ros::Publisher att_velRefpub = nh.advertise<geometry_msgs::Vector3>("att_vel_des",100);
	ros::Publisher psiRefpub = nh.advertise<std_msgs::Float64>("psi_ref",100);	
	
	while(ros::ok())
	{
		x_des = 0;
		y_des = 0;
		z_des = -2;
		psi_des = 0;
		
		x_vel_des = 0;
		y_vel_des = 0;
		z_vel_des = 0;
		
		roll_vel_des = 0;
		pitch_vel_des = 0;
		yaw_vel_des = 0;
		
		positionRefVector.x = x_des;
		positionRefVector.y = y_des;
		positionRefVector.z = z_des;

		psiReference.data = psi_des;
		
		velocityRefVector.x = x_vel_des;
		velocityRefVector.y = y_vel_des;
		velocityRefVector.z = z_vel_des;
		
		att_velRefVector.x = roll_vel_des;
		att_velRefVector.y = pitch_vel_des; 
		att_velRefVector.z = yaw_vel_des;
		
		posRefpub.publish(positionRefVector);
		psiRefpub.publish(psiReference);
		velRefpub.publish(velocityRefVector);
		att_velRefpub.publish(att_velRefVector);
		
		ros::spinOnce();
		loop_rate.sleep();				
	}
	
	return 0;
}
