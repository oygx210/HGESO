#include <eigen3/Eigen/Dense>
#include <iostream>
#include <ros/ros.h>
#include <math.h>
#include "geometry_msgs/Vector3.h"
#include "std_msgs/Float64.h"
#include <math.h>

//Fixed-step time;
float step = 0.01;

//Position 
Eigen::Vector3f uav_pos_IF; //	x,y,z
Eigen::Vector3f uav_vel_IF; //	x_dot, y_dot, z_dot

//Attitude
Eigen::Vector3f uav_att; //phi, theta, psi

//Desired values
Eigen::Vector3f pos_des;	//x_des, y_des, z_des
Eigen::Vector3f vel_des;	//x_dot_des, y_dot_des, z_dot_des
float psi_des;	

//Error signals
Eigen::Vector3f error_pos; //error_x, error_y, error_z
Eigen::Vector3f error_vel; //error_x_dot, error_y_dot, error_z_dot

//Sliding surfaces
Eigen::Vector3f sigma_pos;	//sigma_x, sigma_y, sigma_z

//Adaptive sliding mode control parameters
Eigen::Vector3f lambda_pos;	//lambda_x lambda_y lambda_z
Eigen::Vector3f K1_pos; //K1x, K1y, K1z
Eigen::Vector3f K1_pos_dot; //K1x_dot K1y_dot K1z_dot
Eigen::Vector3f K2_pos; //K2x, K2y, K2z
Eigen::Vector3f k_pos; //kx, ky, kz
Eigen::Vector3f kmin_pos; //kmin_x, kmin_y, kmin_z
Eigen::Vector3f mu_pos; //mu_x, mu_y, mu_z

//Outputs
float ux;
float uy;
float uz;
float U1;

float phi_des;
float theta_des;
float phi_des_arg;
float theta_des_arg;

//UAV parameters
float m = 1.8;
float g = 9.81;
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

void pos_ref_callback(const geometry_msgs::Vector3::ConstPtr& posRef)
{
	pos_des(0) = posRef->x;
	pos_des(1) = posRef->y;
	pos_des(2) = posRef->z;
}

void vel_ref_callback(const geometry_msgs::Vector3::ConstPtr& velRef)
{
	vel_des(0) = velRef->x;
	vel_des(1) = velRef->y;
	vel_des(2) = velRef->z;
}

void psi_ref_callback(const std_msgs::Float64ConstPtr& psiRef)
{
	psi_des = psiRef->data;
}

void position_callback(const geometry_msgs::Vector3::ConstPtr& pos)
{
	uav_pos_IF(0) = pos->x;
	uav_pos_IF(1) = pos->y;
	uav_pos_IF(2) = pos->z;
}

void velocity_callback(const geometry_msgs::Vector3::ConstPtr& vel)
{
	uav_vel_IF(0) = vel->x;
	uav_vel_IF(1) = vel->y;
	uav_vel_IF(2) = vel->z;
}

void attitude_callback(const geometry_msgs::Vector3::ConstPtr& att)
{
	uav_att(0) = att->x;
	uav_att(1) = att->y;
	uav_att(2) = att->z;
}

void Flagcallback(const std_msgs::Float64::ConstPtr& Fl)
{
	bandera = Fl->data;
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "asmc_position");
	ros::NodeHandle nh;
	ros::Rate loop_rate(30);
	
	ros::Subscriber position_des = nh.subscribe("pos_ref", 100, &pos_ref_callback);
	ros::Subscriber velocity_des = nh.subscribe("vel_ref", 100, &vel_ref_callback);
	ros::Subscriber yaw_des = nh.subscribe("psi_ref", 100, &psi_ref_callback);
	ros::Subscriber position = nh.subscribe("uav_estimated_position",100, &position_callback);
	ros::Subscriber velocity = nh.subscribe("uav_estimated_vel_IF",100, &velocity_callback);
	ros::Subscriber attitude = nh.subscribe("uav_estimated_attitude",100, &attitude_callback);
	ros::Subscriber flag_sub = nh.subscribe("Flag_topic",100, &Flagcallback);
	
	geometry_msgs::Vector3 vector;
	ros::Publisher pub = nh.advertise<geometry_msgs::Vector3>("Thrust_and_des_att",100);
	
	lambda_pos << 1,1,1;	//lambda_x lambda_y lambda_z
	K1_pos << 0,0,0; //K1x, K1y, K1z
	K1_pos_dot << 0,0,0; //K1x_dot K1y_dot K1z_dot
	K2_pos << 2,2,2; //K2x, K2y, K2z
	k_pos << 0.3,0.3,0.3; //kx, ky, kz
	kmin_pos << 0.01,0.01,0.01; //kmin_x, kmin_y, kmin_z
	mu_pos << 0.1,0.1,0.1; //mu_x, mu_y, mu_z
	
	while(ros::ok())
	{
		if(bandera == 0)
		{
			// Error signals
			for(int i = 0; i <= 2; i++)
			{
				error_pos(i) = uav_pos_IF(i) - pos_des(i);	//Error
				error_vel(i) = uav_vel_IF(i) - vel_des(i);	//Error dot
				
				sigma_pos(i) = lambda_pos(i) * error_pos(i) + error_vel(i); //Sliding surface
			
				//Adaptive gain
			
				if(K1_pos(i) > kmin_pos(i))	
				{
					K1_pos_dot(i) = k_pos(i) * sign(abs(sigma_pos(i))-mu_pos(i));
				}
				else
				{
					K1_pos_dot(i) = kmin_pos(i);
				}
				
				K1_pos(i) = K1_pos(i) + step * K1_pos_dot(i); //New value of K1
			}
			
			//Virtual controller
			ux = -K1_pos(0) * sqrt(abs(sigma_pos(0))) * sign(sigma_pos(0)) - K2_pos(0) * sigma_pos(0);
			uy = -K1_pos(1) * sqrt(abs(sigma_pos(1))) * sign(sigma_pos(1)) - K2_pos(1) * sigma_pos(1);
			uz = -K1_pos(2) * sqrt(abs(sigma_pos(2))) * sign(sigma_pos(2)) - K2_pos(2) * sigma_pos(2);
		
			//Thrust	
			U1 = (m / cos(uav_att(0)) * cos(uav_att(1))) * (uz - g) - lambda_pos(2) * error_pos(2);	
		
			if(U1 < -30)
			{
				U1 = -30;
			}	
		
			else if(U1 > 0)
			{
				U1 = 0;
			}
			
			//Desired roll and pitch
		
			phi_des_arg = (m / U1) * (sin(psi_des) * ux - cos(psi_des) * uy);
			
			if(phi_des_arg > 1)
			{
				phi_des_arg = 1;
			}
			else if(phi_des_arg < -1)
			{
				phi_des_arg = -1;
			}
		
			phi_des = asin(phi_des_arg); 
		
			theta_des_arg = ((m/U1)*ux -sin(psi_des)*sin(phi_des)) / (cos(psi_des) * cos(phi_des));
			
			if(theta_des_arg > 1)
			{
				theta_des_arg = 1;
			}
			else if(theta_des_arg < -1)
			{
				theta_des_arg = -1;
			}
			
			theta_des = asin(theta_des_arg);
		
		
			//Data to publish
			vector.x = U1;
			vector.y = phi_des;
			vector.z = theta_des;
			
			std::cout << "ux " << ux << std::endl; 
			std::cout << "uy " << uy << std::endl;
			std::cout << "uz " << uz << std::endl;
			std::cout << "U1 " << U1 << std::endl;
			
			pub.publish(vector);
			ros::spinOnce();
			loop_rate.sleep();
		
		}
 	}
	return 0;
} 
