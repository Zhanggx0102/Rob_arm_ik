/*///*****************************************************************************************************************************************************
Author:		Shawn
Date:		2017-11-2
Abstract:	This used to test the new Ipal arm IK solve algorithm
			algorithm 1: Inverse Transformation method
			algorithm 2: Pieper method

*///******************************************************************************************************************************************************

#include <time.h>
#include <windows.h>
#include <iostream>

#include"ik_solver.h"

#define pi 3.14159265

//******************************************************************************************************************************************************
void Direct_Kinematics(float s1, float s2, float s3, float s4, float s5, float (*p)[4], float (*n)[4], float (*o)[4], float (*a)[4]);


//******************************************************************************************************************************************************
int main(int argc, char *argv[])
{
	float s1,s2,s3,s4,s5;//角度
	/*
	s1: [-180, 0]
	s2: [0, 90]
	s3: [0, 180]
	s4: [-90, 0]
	s5: [-80, 80]
	*/

	float Joint[5];//保存最终计算结果
	int solver_number;
	
	///*
	float P[4];
    float N[4];
    float O[4];
    float A[4];

	s1 =0*degTorad; 
	s2 =0*degTorad;
	s3 =90*degTorad; 
	s4 =0*degTorad;
	s5 =0*degTorad;

	Direct_Kinematics(s1, s2, s3, s4, s5, &P, &N, &O, &A);
	

	printf("    float P[4] = {%f , %f , %f , 1};\n",P[0],P[1],P[2] );
	printf("    float N[4] = {%f , %f , %f , 0};\n",N[0],N[1],N[2] );
	printf("    float O[4] = {%f , %f , %f , 0};\n",O[0],O[1],O[2] );
	printf("    float A[4] = {%f , %f , %f , 0};\n",A[0],A[1],A[2] );

	//rob_arm_ik_solver Letf_Arm_IK;
	//solver_number = Letf_Arm_IK.PM_IK_Solver_Left_Arm(P,N,O,A,&Joint);



	//*/
	
	
	/*
	//
    //float P[4] = {108.109085, 204.186157, 77.943832, 1};
    //float N[4] = {0.918181, -0.395192, -0.027681 , 0};
    //float O[4] = {-0.078564, -0.113159, -0.990466 , 0};
    //float A[4] = {0.388292, 0.911602, -0.134948 , 0};

    float P[4] = {112.159523 , 102.006897 , 42.072346 , 1};
    float N[4] = {0.901013 , -0.431327 , -0.046177 , 0};
    float O[4] = {-0.005968 , 0.094114 , -0.995543 , 0};
    float A[4] = {0.433750 , 0.897274 , 0.082224 , 0};



	// RM[3] = {0.973539, -0.126354, 0.190411, -0.226109, -0.411868, 0.882746, -0.033114, -0.902441, -0.429539} 
	// TM[3] = {173.388092, 146.133530, 76.248863}  

	//float P[4] = {173.388092, 146.133530, 76.248863 , 1};
    //float N[4] = {0.973539 , -0.226109 , -0.033114 , 0};
    //float O[4] = {-0.126354 , -0.411868 , -0.902441 , 0};
    //float A[4] = {0.190411 , 0.882746 , -0.429539 , 0};

	printf("    float P[4] = {%f , %f , %f , 1};\n",P[0],P[1],P[2] );
	printf("    float N[4] = {%f , %f , %f , 0};\n",N[0],N[1],N[2] );
	printf("    float O[4] = {%f , %f , %f , 0};\n",O[0],O[1],O[2] );
	printf("    float A[4] = {%f , %f , %f , 0};\n",A[0],A[1],A[2] );

	rob_arm_ik_solver Letf_Arm_IK;
	//solver_number = Letf_Arm_IK.ITM_IK_Solver_Left_Arm(P,N,O,A,&Joint);
	solver_number = Letf_Arm_IK.PM_IK_Solver_Left_Arm(P,N,O,A,&Joint);

	//-136.945310 82.387396 72.358662 -49.547273 -16.579318
	s1 = Joint[0];
	s2 = Joint[1];
	s3 = Joint[2];
	s4 = Joint[3];
	s5 = Joint[4];

	Direct_Kinematics(s1, s2, s3, s4, s5, &P, &N, &O, &A);
	printf("    float P[4] = {%f , %f , %f , 1};\n",P[0],P[1],P[2] );
	printf("    float N[4] = {%f , %f , %f , 0};\n",N[0],N[1],N[2] );
	printf("    float O[4] = {%f , %f , %f , 0};\n",O[0],O[1],O[2] );
	printf("    float A[4] = {%f , %f , %f , 0};\n",A[0],A[1],A[2] );
	//*/

	system("pause");
	return 0;
	
}

//******************************************************************************************************************************************************
void Direct_Kinematics(float s1, float s2, float s3, float s4, float s5, float (*p)[4], float (*n)[4], float (*o)[4], float (*a)[4])
{
	float P[4];
    float N[4];
    float O[4];
    float A[4];

	N[0] = - cos(s5)*(cos(s4)*(sin(s1)*sin(s3) - cos(s1)*cos(s2)*cos(s3)) + cos(s1)*sin(s2)*sin(s4)) - sin(s5)*(cos(s3)*sin(s1) + cos(s1)*cos(s2)*sin(s3));
	N[1] = cos(s5)*(cos(s4)*(cos(s1)*sin(s3) + cos(s2)*cos(s3)*sin(s1)) - sin(s1)*sin(s2)*sin(s4)) + sin(s5)*(cos(s1)*cos(s3) - cos(s2)*sin(s1)*sin(s3));
	N[2] = cos(s5)*(cos(s2)*sin(s4) + cos(s3)*cos(s4)*sin(s2)) - sin(s2)*sin(s3)*sin(s5);
	N[3] = 0;
	O[0] = sin(s5)*(cos(s4)*(sin(s1)*sin(s3) - cos(s1)*cos(s2)*cos(s3)) + cos(s1)*sin(s2)*sin(s4)) - cos(s5)*(cos(s3)*sin(s1) + cos(s1)*cos(s2)*sin(s3));
	O[1] = cos(s5)*(cos(s1)*cos(s3) - cos(s2)*sin(s1)*sin(s3)) - sin(s5)*(cos(s4)*(cos(s1)*sin(s3) + cos(s2)*cos(s3)*sin(s1)) - sin(s1)*sin(s2)*sin(s4));
	O[2] =  - sin(s5)*(cos(s2)*sin(s4) + cos(s3)*cos(s4)*sin(s2)) - cos(s5)*sin(s2)*sin(s3);
	O[3] = 0;
	A[0] = sin(s4)*(sin(s1)*sin(s3) - cos(s1)*cos(s2)*cos(s3)) - cos(s1)*cos(s4)*sin(s2);
	A[1] = - sin(s4)*(cos(s1)*sin(s3) + cos(s2)*cos(s3)*sin(s1)) - cos(s4)*sin(s1)*sin(s2);
	A[2] = cos(s2)*cos(s4) - cos(s3)*sin(s2)*sin(s4);
	A[3] = 0;
	P[0] = 40*cos(s1)*cos(s2) - (764*cos(s1)*sin(s2))/5;
	P[1] = 40*cos(s2)*sin(s1) - (764*sin(s1)*sin(s2))/5;
	P[2] = (764*cos(s2))/5 + 40*sin(s2);
	P[3] = 1;

	 for(int i=0;i<4;i++)
    {
        (*n)[i] = N[i];
        (*o)[i] = O[i];
        (*a)[i] = A[i];
        (*p)[i] = P[i];//trensform m to mm
    }


}
