#include <eigen3/Eigen/Dense>
#include <iostream>
#include <ros/ros.h>

#include "geometry_msgs/Pose.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/Vector3.h"

#include "std_msgs/Float64.h"
#include "std_msgs/UInt8.h"

#include <math.h>
#include <eigen3/Eigen/Dense>

#include <tf2/LinearMath/Quaternion.h>

//Fixed-step time;
float step = 0.01;

//Attitude
Eigen::Vector3f uav_att;	//	phi, theta, psi
Eigen::Vector3f uav_att_vel;	//	phi_dot, theta_dot, psi_dot

//Desired values
Eigen::Vector3f att_des;  //phi_des, theta_des, psi_des
Eigen::Vector3f att_vel_des;  //phi_dot_des, theta_dot_des, psi_dot_des

//Error signals
Eigen::Vector3f error_att; //error_phi, error_theta, error_psi
Eigen::Vector3f error_att_vel;	// error_phi_dot error_theta_dot error_psi_dot 

//Sliding surfaces
Eigen::Vector3f sigma_att;	//sigma_phi sigma_theta sigma_psi

//Adaptive sliding mode control parameters
Eigen::Vector3f lambda_att; //lambda_phi lambda_theta lambda_psi
Eigen::Vector3f K1_att; //K1_phi, K1_theta, K1_psi
Eigen::Vector3f K1_att_dot;//K1_phi_dot K1_theta_dot K1_psi_dot
Eigen::Vector3f K2_att; //K2_phi, K2_theta, K2_psi
Eigen::Vector3f k_att;	//kphi ktheta kpsi
Eigen::Vector3f kmin_att; //kmin_phi kmin_theta, kmin_psi
Eigen::Vector3f mu_att; //mu_phi, mu_theta, mu_psi

//Outputs
Eigen::Vector3f tau; //tau_phi, //tau_theta //tau_psi
float bandera;

float sign(float value)
{
	int result;
	
	if(value > 0)
	{
		result = 1;
	}
	
	else if(value < 0)
	{
		result = -1;
	}
	
	else if(value == 0)
	{
		result = 0;
	}
	
	return result;
}


void desRP_callback(const geometry_msgs::Vector3::ConstPtr& vec)
{
	att_des(0) = vec->y;
	att_des(1) = vec->z;
}

void desYaw_callback(const std_msgs::Float64ConstPtr& psiRef)
{
	att_des(2) = psiRef->data;
}

void att_velRef_callback(const geometry_msgs::Vector3::ConstPtr& attVelRef)
{
	att_vel_des(0) = attVelRef->x;
	att_vel_des(1) = attVelRef->y;
	att_vel_des(2) = attVelRef->z;
}

void attitude_callback(const geometry_msgs::Vector3::ConstPtr& att)
{
	uav_att(0) = att->x;
	uav_att(1) = att->y;
	uav_att(2) = att->z;
}

void attitude_vel_callback(const geometry_msgs::Vector3::ConstPtr& attVel)
{
	uav_att_vel(0) = attVel->x;
	uav_att_vel(1) = attVel->y;
	uav_att_vel(2) = attVel->z;
}

void Flagcallback(const std_msgs::Float64::ConstPtr& Fl)
{
	bandera = Fl->data;
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "asmc_attitude");
	ros::NodeHandle nh;
	ros::Rate loop_rate(100);
	
	ros::Subscriber desired_rp = nh.subscribe("Thrust_and_des_att", 100, &desRP_callback);
	ros::Subscriber desired_yaw = nh.subscribe("psi_ref",100, &desYaw_callback);
	ros::Subscriber desired_att_vel = nh.subscribe("att_vel_des",100, &att_velRef_callback);	
	ros::Subscriber attitude = nh.subscribe("uav_estimated_attitude",100, &attitude_callback);
	ros::Subscriber attitude_velocity = nh.subscribe("uav_estimated_angular_vel",100, &attitude_vel_callback);
	ros::Subscriber flag_sub = nh.subscribe("Flag_topic",100, &Flagcallback);
	
	geometry_msgs::Vector3 torques;
	ros::Publisher pub_torques = nh.advertise<geometry_msgs::Vector3>("Torques",100);
	
	lambda_att << 5,5,4; //lambda_phi lambda_theta lambda_psi
	K1_att << 0,0,0; //K1_phi, K1_theta, K1_psi
	K1_att_dot << 0,0,0; //K1_phi_dot K1_theta_dot K1_psi_dot
	K2_att << 1,1,1; //K2_phi, K2_theta, K2_psi
	k_att << 0.25,0.25,0.25;	//kphi ktheta kpsi
	kmin_att << 0.01,0.01,0.01; //kmin_phi kmin_theta, kmin_psi
	mu_att << 0.1,0.1,0.1; //mu_phi, mu_theta, mu_psi

	
	
	while(ros::ok())
	{
		if(bandera==0)
		{
			// Error signals
			for(int i = 0; i <= 2; i++)
			{
				error_att(i) = uav_att(i) - att_des(i);	//Error
				error_att_vel(i) = uav_att_vel(i) - att_vel_des(i);	//Error dot
				
				sigma_att(i) = lambda_att(i) * error_att(i) + error_att_vel(i); //Sliding surface
			
				//Adaptive gain
			
				if(K1_att(i) > kmin_att(i))	
				{
					K1_att_dot(i) = k_att(i) * sign(abs(sigma_att(i))-mu_att(i));
				}
				else
				{
					K1_att_dot(i) = kmin_att(i);
				}
				
				K1_att(i) = K1_att(i) + step * K1_att_dot(i); //New value of K1
			}
		
			tau(0) = -K1_att(0) * sqrt(abs(sigma_att(0))) * sign(sigma_att(0)) - K2_att(0) * sigma_att(0);
			tau(1) = -K1_att(1) * sqrt(abs(sigma_att(1))) * sign(sigma_att(1)) - K2_att(1) * sigma_att(1);
			tau(2) = -K1_att(2) * sqrt(abs(sigma_att(2))) * sign(sigma_att(2)) - K2_att(2) * sigma_att(2);
		
			torques.x = tau(0);
			torques.y = tau(1);
			torques.z = tau(2);
		
		
			pub_torques.publish(torques);
			ros::spinOnce();
			loop_rate.sleep();
		}
 }
	
	return 0;
} 
