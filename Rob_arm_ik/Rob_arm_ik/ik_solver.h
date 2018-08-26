#ifndef _IK_SOLVER_H
#define _IK_SOLVER_H

#include<cstdio>
#include<cmath>
#include <iostream>
#include <cstring>

#define pi 3.14159265
#define degTorad pi/180
#define radTodeg 180/pi

class rob_arm_ik_solver
{
private:
	float theta_1[2],theta_2[2],theta_3[4],theta_4[8],theta_5[8];//save single joint result
	int theta_1_count,theta_2_count,theta_3_count,theta_4_count,theta_5_count;//save number of each joint solution
	float joint[8][5];//save all last result 
		
public:
	rob_arm_ik_solver();
	~rob_arm_ik_solver();
	/************************************************** 
	 * function: 
			   --solving inverse kinematics of rob left arm 
	 * params: 
	           --P[3]:End-effector's space coordinates (homogeneous coordinate)
			   --N[3]:x-axis direction column matrix (moving coordinate system's orientation cosine in  static coordinate system)
			   --O[3]:y-axis direction column matrix (moving coordinate system's orientation cosine in  static coordinate system)
			   --A[3]:z-axis direction column matrix (moving coordinate system's orientation cosine in  static coordinate system)
			   --(*p)[5]:computed result will save in this array
	 * return:  
	           --quantity of result
	 *************************************************/
	//algorithm 1: Inverse Transformation method
	int ITM_IK_Solver_Left_Arm(float P[3],float N[3],float O[3],float A[3], float (*p)[5]);

	/************************************************** 
	 * function: 
			   --solving inverse kinematics of rob left arm 
	 * params: 
	           --P[3]:End-effector's space coordinates (homogeneous coordinate)
			   --N[3]:x-axis direction column matrix (moving coordinate system's orientation cosine in  static coordinate system)
			   --O[3]:y-axis direction column matrix (moving coordinate system's orientation cosine in  static coordinate system)
			   --A[3]:z-axis direction column matrix (moving coordinate system's orientation cosine in  static coordinate system)
			   --(*p)[5]:computed result will save in this array
	 * return:  
	           --quantity of result
	 *************************************************/
	//algorithm 2: Pieper method
	int PM_IK_Solver_Left_Arm(float P[3],float N[3],float O[3],float A[3], float (*p)[5]);

};


#endif