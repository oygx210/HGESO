#include <eigen3/Eigen/Dense>
#include <iostream>
#include <ros/ros.h>
#include "geometry_msgs/Vector3.h"
#include "std_msgs/Float64.h"
#include <math.h>

float step = 0.01;
float g = 9.81;
float m = 1.8;

Eigen::Vector3f linear_position(0,0,0);
Eigen::Vector3f attitude_position(0,0,0);

Eigen::Vector3f linear_velocity_BF(0,0,0);
Eigen::Vector3f linear_velocity_IF(0,0,0);
Eigen::Vector3f attitude_velocity(0,0,0);

Eigen::Vector3f linear_acceleration_BF(0,0,0);
Eigen::Vector3f attitude_acceleration(0,0,0);

Eigen::Vector3f tau;
float U1;
Eigen::Vector3f force(0,0,0);

Eigen::Vector3f e3(0,0,1);

Eigen::Matrix3f J;

Eigen::Matrix3f RotationMatrix(Eigen::Vector3f attitude)
{
	float cos_phi = cos(attitude(0));
	float sin_phi = sin(attitude(0));
	
	float cos_theta = cos(attitude(1));
	float sin_theta = sin(attitude(1));
	
	
	float cos_psi = cos(attitude(2));
	float sin_psi = sin(attitude(2));
	
	
	Eigen::Matrix3f R;
	
	R << cos_psi * cos_theta, cos_psi * sin_phi * sin_theta - cos_phi * sin_psi, sin_psi * sin_phi + cos_psi * cos_phi * sin_theta,
			 cos_theta * sin_psi, cos_psi * cos_phi + sin_psi * sin_phi * sin_theta, cos_phi * sin_psi * sin_theta - cos_psi * sin_phi,
			 -sin_theta, cos_theta * sin_phi, cos_phi * cos_theta;
			 
	return R;
	
}

Eigen::Matrix3f skewOmega(Eigen::Vector3f attitude_velocity)
{
	Eigen::Matrix3f Skew;
	
	Skew << 0, -attitude_velocity(2), attitude_velocity(1),
					attitude_velocity(2), 0, -attitude_velocity(0),
					-attitude_velocity(1), attitude_velocity(0), 0;
					
	return Skew;

}

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

int main(int argc, char **argv)
{
	ros::init(argc, argv, "quad_dynamics");
	ros::NodeHandle nh;
	ros::Rate loop_rate(100);
	
	ros::Subscriber thrust = nh.subscribe("Thrust_and_des_att",100,&ForceInputCallback);
	ros::Subscriber torques = nh.subscribe("Torques",100,&TorqueInputsCallback);
	
	geometry_msgs::Vector3 positionVector;
	geometry_msgs::Vector3 attitudeVector;
	geometry_msgs::Vector3 velocityVector;
	geometry_msgs::Vector3 att_velVector;
	geometry_msgs::Vector3 BFvelocityVector;
	
	ros::Publisher positionPub = nh.advertise<geometry_msgs::Vector3>("uav_position",100);
	ros::Publisher attitudePub = nh.advertise<geometry_msgs::Vector3>("uav_attitude",100);
	ros::Publisher velocityPub = nh.advertise<geometry_msgs::Vector3>("uav_velocity",100);
	ros::Publisher att_velPub = nh.advertise<geometry_msgs::Vector3>("uav_attitude_vel",100);
	ros::Publisher BFvelocityPub = nh.advertise<geometry_msgs::Vector3>("uav_vel_BF",100);
	
	J << 0.0411, 0, 0,
		 0, 0.0478, 0,
		 0, 0, 0.0599;
	
	while(ros::ok())
	{
		//Angular dynamics
		attitude_acceleration = J.inverse() * (tau - skewOmega(attitude_velocity) * J * attitude_velocity);
	
		for(int i = 0; i <= 2; i++)
		{
			attitude_velocity(i) = attitude_velocity(i) + step * attitude_acceleration(i);
			attitude_position(i) = attitude_position(i) + step * attitude_velocity(i);
		}
		
		//Linear dynamics
		force = U1 * e3 + RotationMatrix(attitude_position).inverse() * m * g * e3;
		
		linear_acceleration_BF = force/m - skewOmega(attitude_velocity) * linear_velocity_BF;
		 
		for(int i = 0; i <= 2; i++)
		{
			linear_velocity_BF(i) = linear_velocity_BF(i) + step * linear_acceleration_BF(i);
		}
		
		linear_velocity_IF = RotationMatrix(attitude_position) * linear_velocity_BF;
		
		for(int i = 0; i <= 2; i++)
		{
			linear_position(i) = linear_position(i) + step * linear_velocity_IF(i);
		}
		
		positionVector.x = linear_position(0);
		positionVector.y = linear_position(1);
		positionVector.z = linear_position(2);
		
		attitudeVector.x = attitude_position(0);
		attitudeVector.y = attitude_position(1);
		attitudeVector.z = attitude_position(2);
		
		velocityVector.x = linear_velocity_IF(0);
		velocityVector.y = linear_velocity_IF(1);
		velocityVector.z = linear_velocity_IF(2);
		
		att_velVector.x = attitude_velocity(0);
		att_velVector.y = attitude_velocity(1);
		att_velVector.z = attitude_velocity(2);
		
		BFvelocityVector.x = linear_velocity_BF(0);
		BFvelocityVector.y = linear_velocity_BF(1);
		BFvelocityVector.z = linear_velocity_BF(2);
		
		positionPub.publish(positionVector);
		attitudePub.publish(attitudeVector);
		velocityPub.publish(velocityVector);
		att_velPub.publish(att_velVector);
		BFvelocityPub.publish(BFvelocityVector);
		
		//std::cout << "U1 : " << U1 << std::endl;
		//std::cout << "force: " << force << std::endl; 
		
		ros::spinOnce();
		loop_rate.sleep();
				
	}
	return 0;	
}
