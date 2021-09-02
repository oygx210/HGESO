#include <eigen3/Eigen/Dense>
#include <iostream>
#include <ros/ros.h>
#include "geometry_msgs/Vector3.h"
#include <math.h>
#include <eigen3/Eigen/Dense>



float step = 0.01;	//Integration step time
float g = 9.81;			//gravity
float m = 1.8;			//quadrotor's mass
float U1;						//Thrust
Eigen::Vector3f tau;			//Torques
float Jxx = 0.0411;	//Moments of inertia X
float Jyy = 0.0478;	//Moments of inertia Y
float Jzz = 0.0599;	//Moments of inertia Z

Eigen::Vector3f quadPos;		//Quadrotor's position output
Eigen::Vector3f quadAtt;		//Quadrotor's attitude output

void ForceInputCallback(const geometry_msgs::Vector3::ConstPtr& thr)
{
	U1 = thr->x;
}

void TorqueInputsCallback(const geometry_msgs::Vector3::ConstPtr& tor)
{
	tau(0) = tor->x;
	tau(1) = tor->y;
	tau(2) = tor->z;
}

void linPosCallback(const geometry_msgs::Vector3::ConstPtr& pos)
{
	quadPos(0) = pos->x;
	quadPos(1) = pos->y;
	quadPos(2) = pos->z;
}

void attCallback (const geometry_msgs::Vector3::ConstPtr& att)
{
	quadAtt(0) = att->x;	//roll
	quadAtt(1) = att->y;	//pitch
	quadAtt(2) = att->z;	//yaw
}

int main(int argc, char **argv)
{

	//ROS initialization
	ros::init(argc, argv, "observer");
	ros::NodeHandle nh;
	ros::Rate loop_rate(100);
	
	ros::Subscriber thrust_sub = nh.subscribe("Thrust_and_des_att",100,&ForceInputCallback);
	ros::Subscriber torques_sub = nh.subscribe("Torques",100,&TorqueInputsCallback);
	ros::Subscriber position_sub = nh.subscribe("uav_position",100,&linPosCallback);
	ros::Subscriber attitude_sub = nh.subscribe("uav_attitude",100,&attCallback);
	
	ros::Publisher posEst_pub = nh.advertise<geometry_msgs::Vector3>("uav_estimated_position",100);
	ros::Publisher attEst_pub = nh.advertise<geometry_msgs::Vector3>("uav_estimated_attitude",100);
	ros::Publisher linVelEst_pub = nh.advertise<geometry_msgs::Vector3>("uav_estimated_vel_IF",100);	
	ros::Publisher angVelEst_pub = nh.advertise<geometry_msgs::Vector3>("uav_estimated_angular_vel",100);
	ros::Publisher distEst_pub = nh.advertise<geometry_msgs::Vector3>("Disturbance_estimation_xyz",100);
	
	geometry_msgs::Vector3 posEst_Vector;
	geometry_msgs::Vector3 attEst_Vector;
	geometry_msgs::Vector3 linVelEst_Vector;
	geometry_msgs::Vector3 angVelEst_Vector;
	geometry_msgs::Vector3 distEst_Vector;
	
	//HGESO outputs
	float x1_est = 0;				//x_est
	float x2_est = 0;				//vel_x_est
	float x3_est = 0;				//dist_x_est
	float x1_est_dot = 0;
	float x2_est_dot = 0;
	float x3_est_dot = 0;
		
	float y1_est = 0;				//y_est
	float y2_est = 0;				//vel_y_est
	float y3_est = 0;				//dist_y_est
	float y1_est_dot = 0;
	float y2_est_dot = 0;
	float y3_est_dot = 0;	
	
	float z1_est = 0;				//z_est
	float z2_est = 0;				//vel_z_est
	float z3_est = 0;				//dist_z_est
	float z1_est_dot = 0;
	float z2_est_dot = 0;
	float z3_est_dot = 0;	
	
	float phi1_est = 0;			//phi_est
	float phi2_est = 0;			//vel_phi_est
	float phi3_est = 0;			//dist_phi_est
	float phi1_est_dot = 0;
	float phi2_est_dot = 0;
	float phi3_est_dot = 0;	
	
	float theta1_est = 0;		//theta_est
	float theta2_est = 0;		//vel_theta_est
	float theta3_est = 0;		//dist_theta_est
	float theta1_est_dot = 0;
	float theta2_est_dot = 0;
	float theta3_est_dot = 0;	
	
	float psi1_est = 0;			//psi_est
	float psi2_est = 0;			//vel_psi_est
	float psi3_est = 0;			//dist_psi_est
	float psi1_est_dot = 0;
	float psi2_est_dot = 0;
	float psi3_est_dot = 0;	
	
	
	//HGESO gains
	Eigen::Vector3f gamma;
	gamma << 3,3,1;
	float epsilon = 0.03;
	
	while(ros::ok())
	{
		//---------------Attitude estimation-----------------------
		//----------------------Roll-------------------------------
		phi3_est_dot = (gamma(2)/powf(epsilon,3)) * (quadAtt(0) - phi1_est);
		phi3_est = phi3_est + (phi3_est_dot * step); 
		
		phi2_est_dot = phi3_est + tau(0)/Jxx + ((Jyy-Jzz)/Jxx) * theta2_est * psi2_est + (gamma(1)/powf(epsilon,2)) * (quadAtt(0) - phi1_est);
		phi2_est = phi2_est + (phi2_est_dot * step);
		
		phi1_est_dot = phi2_est + (gamma(0)/epsilon) * (quadAtt(0) - phi1_est);
		phi1_est = phi1_est + (phi1_est_dot * step);
		//----------------------------------------------------------
		//----------------------Pitch-------------------------------
		theta3_est_dot = (gamma(2)/powf(epsilon,3)) * (quadAtt(1) - theta1_est);
		theta3_est = theta3_est + (theta3_est_dot * step); 
		
		theta2_est_dot = theta3_est + tau(1)/Jyy + ((Jzz-Jxx)/Jyy) * phi2_est * psi2_est + (gamma(1)/powf(epsilon,2)) * (quadAtt(1) - theta1_est);
		theta2_est = theta2_est + (theta2_est_dot * step);
		
		theta1_est_dot = theta2_est + (gamma(0)/epsilon) * (quadAtt(1) - theta1_est);
		theta1_est = theta1_est + (theta1_est_dot * step);
		//----------------------------------------------------------
		//-----------------------Yaw--------------------------------
		psi3_est_dot = (gamma(2)/powf(epsilon,3)) * (quadAtt(2) - psi1_est);
		psi3_est = psi3_est + (psi3_est_dot * step);
		
		psi2_est_dot = psi3_est + tau(2)/Jzz + ((Jxx-Jyy)/Jzz) * phi2_est * theta2_est + (gamma(1)/powf(epsilon,2)) * (quadAtt(2) - psi1_est);
		psi2_est = psi2_est + (psi2_est_dot * step);
		
		psi1_est_dot = psi2_est + (gamma(0)/epsilon) * (quadAtt(2) - psi1_est);
		psi1_est = psi1_est + (psi1_est_dot * step);
		
		//------------------Position estimation--------------------------
		//---------------------------X-----------------------------------
		x3_est_dot = (gamma(2)/powf(epsilon,3)) * (quadPos(0) - x1_est);
		x3_est = x3_est + (x3_est_dot * step);
		
		x2_est_dot = x3_est + (U1/m) * (sin(phi1_est)*sin(psi1_est) + cos(phi1_est)*cos(psi1_est)*sin(theta1_est)) + (gamma(1)/powf(epsilon,2)) * (quadPos(0) - x1_est);
		x2_est = x2_est + (x2_est_dot * step);
		
		x1_est_dot = x2_est + (gamma(0)/epsilon) * (quadPos(0) - x1_est); 
		x1_est = x1_est + (x1_est_dot * step);
		//---------------------------------------------------------------
		//--------------------------Y------------------------------------
		y3_est_dot = (gamma(2)/powf(epsilon,3)) * (quadPos(1) - y1_est);
		y3_est = y3_est + (y3_est_dot * step);
		
		y2_est_dot = y3_est + (U1/m) * (cos(phi1_est)*sin(psi1_est)*sin(theta1_est) - cos(psi1_est)*sin(phi1_est)) + (gamma(1)/powf(epsilon,2)) * (quadPos(1) - y1_est);
		y2_est = y2_est + (y2_est_dot * step);
		
		y1_est_dot = y2_est + (gamma(0)/epsilon) * (quadPos(1) - y1_est); 
		y1_est = y1_est + (y1_est_dot * step);
		//---------------------------------------------------------------
		//--------------------------Z------------------------------------
		z3_est_dot = (gamma(2)/powf(epsilon,3)) * (quadPos(2) - z1_est);
		z3_est = z3_est + (z3_est_dot * step);
		
		z2_est_dot = z3_est + (U1/m) * (cos(phi1_est)*cos(theta1_est)) + g + (gamma(1)/powf(epsilon,2)) * (quadPos(2) - z1_est);
		z2_est = z2_est + (z2_est_dot * step);
		
		z1_est_dot = z2_est + (gamma(0)/epsilon) * (quadPos(2) - z1_est);
		z1_est = z1_est + (z1_est_dot * step);
		//---------------------------------------------------------------
		
		//ROS Publishing
		posEst_Vector.x = x1_est;
		posEst_Vector.y = y1_est;
		posEst_Vector.z = z1_est;
		
		attEst_Vector.x = phi1_est;
		attEst_Vector.y = theta1_est;
		attEst_Vector.z = psi1_est;
		
		linVelEst_Vector.x = x2_est;
		linVelEst_Vector.y = y2_est;
		linVelEst_Vector.z = z2_est;
		
		angVelEst_Vector.x = phi2_est;
		angVelEst_Vector.y = theta2_est;
		angVelEst_Vector.z = psi2_est;
		
		distEst_Vector.x = x3_est;
		distEst_Vector.y = y3_est;
		distEst_Vector.z = z3_est;
		
		posEst_pub.publish(posEst_Vector);
		attEst_pub.publish(attEst_Vector);
		linVelEst_pub.publish(linVelEst_Vector);
		angVelEst_pub.publish(angVelEst_Vector);
		distEst_pub.publish(distEst_Vector);
				
		
		ros::spinOnce();
		loop_rate.sleep();
				
	}
	return 0;	
}
