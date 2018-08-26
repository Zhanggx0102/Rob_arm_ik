#include"ik_solver.h"

rob_arm_ik_solver::rob_arm_ik_solver()
{

}

rob_arm_ik_solver::~rob_arm_ik_solver()
{

}


/************************************************** 
* function: 
		--used for  PM_IK_Solver_Left_Arm function to get the rotate matrix
* params: 
	    --P[3]:End-effector's space coordinates (homogeneous coordinate)
		--N[3]:x-axis direction column matrix (moving coordinate system's orientation cosine in  static coordinate system)
		--O[3]:y-axis direction column matrix (moving coordinate system's orientation cosine in  static coordinate system)
		--theta_1
		--theta_2
* return:  
	    --rotate matrix ((*r)[9])
*************************************************/
int PM_IK_Rotate_Matrix(float N[3],float O[3], float A[3], float s1,float s2, float (*J)[3]);

/************************************************** 
* function: 
		--used for  PM_IK_Solver_Left_Arm function to get the rotate matrix
* params: 
	    --P[3]:End-effector's space coordinates (homogeneous coordinate)
		--N[3]:x-axis direction column matrix (moving coordinate system's orientation cosine in  static coordinate system)
		--O[3]:y-axis direction column matrix (moving coordinate system's orientation cosine in  static coordinate system)
		--theta_1
		--theta_2
* return:  
	    --rotate matrix ((*r)[9])
*************************************************/
int Direct_Kinematics_Verification ( float J[5], float P[3]);

int rob_arm_ik_solver::ITM_IK_Solver_Left_Arm(float P[3],float N[3],float O[3],float A[3], float (*p)[5])
	{
		//**初始化预存值***************************************************************************************************************
		for(int i=0;i<2;i++)
		{
			theta_1[i] = 999;
		}
		for(int i=0;i<2;i++)
		{
			theta_2[i] = 999;
		}
		for(int i=0;i<4;i++)
		{
			theta_3[i] = 999;
		}
		for(int i=0;i<8;i++)
		{
			theta_4[i] = 999;
		}
		for(int i=0;i<8;i++)
		{
			theta_5[i] = 999;
		}
		//**变量设置************************************************************************************************************************************************
		int solution_count = 0;
		int solution_row = 999;
		int solver1_Flag = 0;//joint1解得个数
		int solver2_Flag = 0;//joint2解得个数
		int solver3_Flag = 0;//joint3解得个数
		int solver4_Flag = 0;//joint4解得个数
		int solver5_Flag = 0;//joint5解得个数

		//**解joint1**************************************************************************************************************************************************
		// Py*cos(s1) - Px*sin(s1) = 0 

		if(P[0] == 0 && P[1] == 0)
		{
			//can't solve out
			printf("theta_1 no solution! \n");
		}
		else if(P[0] != 0 && P[1] == 0)
		{
			//sin(s1)=0
			theta_1[0] = 0;
			theta_1[1] = pi;
			solver1_Flag =2;

			goto Joint1_solution;
		}
		else if(P[0] == 0 && P[1] != 0)
		{
			//cos(s1)=0
			theta_1[0] = pi/2;
			solver1_Flag = 1;

			goto Joint1_solution;
		}
		else
		{
			//tan(s1) = (Py/Px)
			float temp;
			temp = atan2(P[1],P[0]);
			int k;
			for(k=-10;k<11;k++)
			{
				float temp2 = temp +k*pi;
				//printf("k: %d\n",k);
				//printf("temp2: %f\n",temp2*radTodeg);
					
				if((-180*degTorad<=temp2)&&(temp2<=0))//rang of angle: [-180,0]
				{
					solver1_Flag ++;
					if(solver1_Flag ==1)//保存第一个解
					{
						theta_1[0] = temp2;
					}
					if(solver1_Flag ==2)//保存第二个解
					{
						theta_1[1] = temp2;
					}
				}
			}
		}

		Joint1_solution:

		printf("Joint1[2]: %f %f \n",theta_1[0],theta_1[1]);
		theta_1_count = solver1_Flag;
		printf("theta_1_count: %d\n",theta_1_count);

		//**解joint2*******************************************************************************************************************************************************************************
		//152.8*cos(s2) + 40*sin(s2) = Pz
		if(theta_1_count==0)//joint1 no solution
		{
			printf("Joint1 Joint2 Error!\n");
			solver2_Flag =0;
		}
		else
		{
			float a = 40;
			float b = 152.8;
			float fai;

			fai = atan2(b,a);
			//printf("fai =  %f \n",fai);
			//printf("fai(radTodeg) =  %f \n",fai*radTodeg);
			
			float Rho = sqrt(a*a+b*b);
			//printf("Rho =  %f \n",Rho);

			float temp1, temp2, temp3, temp4;

			temp1 = atan2(P[2]/Rho,sqrt(1-(P[2]/Rho)*(P[2]/Rho))) - fai;
			temp2 = atan2(P[2]/Rho,-sqrt(1-(P[2]/Rho)*(P[2]/Rho))) - fai;
			//printf("temp1 = %f\n",temp1*radTodeg);
			//printf("temp2 = %f\n",temp2*radTodeg);

			int k;
			for(k=-10;k<11;k++)
			{
				temp3 = temp1 +k*pi;
				temp4 = temp2 +k*pi;
				//printf("k: %d\n",k);
				//printf("temp3: %f\n",temp3*radTodeg);
				//printf("temp4: %f\n",temp4*radTodeg);
					
				if((0*degTorad<=temp3)&&(temp3<=91*degTorad))//rang of angle: [0,90]
				{
					solver2_Flag ++;
					theta_2[0] = temp3;

				}
				if((0*degTorad<=temp4)&&(temp4<=91*degTorad))//rang of angle: [0,90]
				{
					solver2_Flag ++;
					theta_2[1] = temp4;

				}
			}

		}

		printf("Joint2[2]: %f %f\n",theta_2[0],theta_2[1]);
		theta_2_count = solver2_Flag;
		printf("theta_2_count: %d\n",theta_2_count);

		//**解joint4********************************************************************************************************************************************************************************
		// Az*cos(s2) - Ax*cos(s1)*sin(s2) - Ay*sin(s1)*sin(s2) = cos(s4)
		if((theta_1_count == 0)||(theta_2_count == 0))//joint1或joint2无解
		{
			printf("Joint4 Error!\n");
		}
		else//joint1 joint2有解
		{
			
			float expression[4];
			float temp1[8],temp2[8];
			for(int i=0;i<8;i++)
			{
				temp1[i] = 999;
				temp2[i] = 999;
			}
			for(int i = 0;i<4;i++)
			{
				if(theta_2[i]!=999)
				{
					if(i<2)
					{
										 //Az*cos(s2) - Ax*cos(s1)*sin(s2) - Ay*sin(s1)*sin(s2)
						expression[i] = A[2]*cos(theta_2[i])-A[0]*cos(theta_1[0])*sin(theta_2[i])-A[1]*sin(theta_1[0])*sin(theta_2[i]);
						temp1[2*i] = acos(expression[i]);
						temp1[2*i+1] = -acos(expression[i]);
					}
					else
					{					//Az*cos(s2) - Ax*cos(s1)*sin(s2) - Ay*sin(s1)*sin(s2)
						expression[i] = A[2]*cos(theta_2[i-2])-A[0]*cos(theta_1[1])*sin(theta_2[i-2])-A[1]*sin(theta_1[1])*sin(theta_2[i-2]);
						temp1[2*i] = acos(expression[i]);
						temp1[2*i+1] = -acos(expression[i]);
					}
				}
			}
			//printf("temp1[8]: %f %f %f %f  %f %f %f %f\n",temp1[0],temp1[1],temp1[2],temp1[3],temp1[4],temp1[5],temp1[6],temp1[7] );
			int k;
			for(k=-10;k<11;k++)
			{
				for(int i=0;i<8;i++)
				{
					if(temp1[i] != 999)
					{
						temp2[i] = temp1[i]+2*k*pi;
						//printf("temp2[%d] = %f\n",i,temp2[i]*radTodeg);
						if(((-91*degTorad)<=temp2[i])&&(temp2[i]<=0))//rang of angle: [-90,0]
						{
							theta_4[i] = temp2[i];
						}
					}
				}
			}
		}
		//判断解得个数
		for(int m=0;m<8;m++)
		{
			if(theta_4[m] != 999)
			{
				solver4_Flag++;
			}
		}
		printf("Joint4[8]: %f %f %f %f  %f %f %f %f\n",theta_4[0],theta_4[1],theta_4[2],theta_4[3],theta_4[4],theta_4[5],theta_4[6],theta_4[7] );
		theta_4_count = solver4_Flag;
		printf("theta_4_count: %d\n",theta_4_count);

		//**解joint3******************************************************************************************************************************
		//
		if((theta_1_count==0)||(theta_2_count==0)||(theta_4_count==0))//joint1,joint2,joint4有一个无解
		{
			printf("Joint3 Error!\n");
		}
		else
		{
			float expression1[4];
			float expression2[4];

			float temp1[4];
			float temp2[4];
			//init temp1 temp2
			for(int i=0;i<4;i++)
			{
				temp1[i] = 999;
				temp2[i] = 999;
				expression1[i] = 999;
				expression2[i] = 999;
			}
			//figure out expression1 expression2
			for(int i=0;i<4;i++)
			{
				if(i<2 && theta_1[0]!=999 && theta_2[i]!=999)
				{
									 //Ax*sin(s1) - Ay*cos(s1)
					expression1[i] = -(A[0]*sin(theta_1[0]) - A[1]*cos(theta_1[0]));
								    //Az*sin(s2) + Ax*cos(s1)*cos(s2) + Ay*cos(s2)*sin(s1)
					expression2[i] = A[2]*sin(theta_2[i]) + A[0]*cos(theta_1[0])*cos(theta_2[i]) + A[1]*cos(theta_2[i])*sin(theta_1[0]);
				}
				else if(2<=i && theta_1[1]!=999 && theta_2[i-2]!=999)
				{
					expression1[i] = -(A[0]*sin(theta_1[1]) - A[1]*cos(theta_1[1]));
					expression2[i] = A[2]*sin(theta_2[i-2]) + A[0]*cos(theta_1[1])*cos(theta_2[i-2]) + A[1]*cos(theta_2[i-2])*sin(theta_1[1]);
				}	
			}
			//handle singularity
			for (int i=0;i<8;i++)
			{
				if(sin(theta_4[i]) == 0.0)//奇异形位
				{
					theta_4[i] = theta_4[i] + 1*degTorad;//将theta_4[i]强行置1
				}
			}
			//handle joint3
			for(int i=0;i<4;i++)
			{
				if(expression2[i] == 0)
				{
					temp1[i] = pi/2;
				}
				else if(expression1[i]!=999  && expression2[i]!=999)
				{
					//printf("Joint3 expression1[%d] = %f\n",i,expression1[i]);
					//printf("Joint3 expression2[%d] = %f\n",i,expression2[i]);
					temp1[i] = atan2(expression1[i],expression2[i]);
				}
			}

			int k;
			for(k=-10;k<11;k++)
			{
				
				for(int i=0;i<4;i++)
				{
					temp2[i] = temp1[i] +k*pi;
					//printf("Joint3 temp2[%d] = %f\n",k,temp2[i]*radTodeg);
					if((0*degTorad<=temp2[i])&&(temp2[i]<=180*degTorad))//rang of angle: [0,180]
					{
						solver3_Flag++;
						theta_3[i] = temp2[i];
					}
				}
			}
		}

		printf("theta_3[4]: %f %f %f %f \n",theta_3[0],theta_3[1],theta_3[2],theta_3[3]);
		theta_3_count = solver3_Flag;
		printf("theta_3_count: %d\n",theta_3_count);

		//**解joint5************************************************************************************************************************************************************************
		//cos(s5) = Oy*(cos(s1)*cos(s3) - cos(s2)*sin(s1)*sin(s3)) - Ox*(cos(s3)*sin(s1) + cos(s1)*cos(s2)*sin(s3)) - Oz*sin(s2)*sin(s3)

		if((theta_1_count==0)||(theta_2_count==0)||(theta_3_count==0))//joint1,joint2,joint3有一个无解
		{
			printf("Joint5 Error!\n");
		}
		else
		{
			float expression1[4];

			float temp1[8];
			float temp2[8];
			//init temp1 temp2
			for(int i=0;i<4;i++)
			{
				temp1[i] = 999;
				temp2[i] = 999;
				expression1[i] = 999;
			}
			//figure out expression1 expression2
			for(int i=0;i<4;i++)
			{
				if(i<2 && theta_1[0]!=999 && theta_2[i]!=999 && theta_3[i]!=999)
				{
								 //Oy*(cos(s1)*cos(s3) - cos(s2)*sin(s1)*sin(s3)) - Ox*(cos(s3)*sin(s1) + cos(s1)*cos(s2)*sin(s3)) - Oz*sin(s2)*sin(s3)
					expression1[i] = O[1]*(cos(theta_1[0])*cos(theta_3[i]) - cos(theta_2[i])*sin(theta_1[0])*sin(theta_3[i])) - O[0]*(cos(theta_3[i])*sin(theta_1[0]) + cos(theta_1[0])*cos(theta_2[i])*sin(theta_3[i])) - O[2]*sin(theta_2[i])*sin(theta_3[i]);
					//printf("Joint5 expression1[%d] = %f\n",i,expression1[i]);
				}
				else if(2<=i && theta_1[1]!=999 && theta_2[i-2]!=999 && theta_3[i]!=999)
				{
					expression1[i] = O[1]*(cos(theta_1[1])*cos(theta_3[i]) - cos(theta_2[i-2])*sin(theta_1[1])*sin(theta_3[i])) - O[0]*(cos(theta_3[i])*sin(theta_1[1]) + cos(theta_1[1])*cos(theta_2[i-2])*sin(theta_3[i])) - O[2]*sin(theta_2[i-2])*sin(theta_3[i]);
					//printf("Joint5 expression1[%d] = %f\n",i,expression1[i]);
				}	
			}
			//handle singularity
			for (int i=0;i<8;i++)
			{
				if(sin(theta_4[i]) == 0.0)//奇异形位
				{
					theta_4[i] = theta_4[i] + 1*degTorad;//将theta_4[i]强行置1
				}
			}
			//handle joint5
			for(int i=0;i<4;i++)
			{
				if(expression1[i]!=999)
				{
					temp1[i*2] = acos(expression1[i]);
					temp1[i*2+1] = -acos(expression1[i]);
					//printf("Joint5 temp1[%d] = %f\n",i,temp1[i*2]);
					//printf("Joint5 temp1[%d] = %f\n",i,temp1[i*2+1]);
				}
			}

			int k;
			for(k=-10;k<11;k++)
			{
				for(int i=0;i<8;i++)
				{
					if(temp1[i] != 999)
					{
						temp2[i] = temp1[i]+2*k*pi;
						if(((-91*degTorad)<=temp2[i])&&(temp2[i]<=91*degTorad))//rang of angle: [-90,90]
						{
							theta_5[i] = temp2[i];
						}
					}
				}
			}

		}
		//判断解得个数
		for(int m=0;m<8;m++)
		{
			if(theta_5[m] != 999)
			{
				solver5_Flag++;
			}
		}

		printf("theta_5[4]: %f %f %f %f %f %f %f %f\n",theta_5[0],theta_5[1],theta_5[2],theta_5[3],theta_5[4],theta_5[5],theta_5[6],theta_5[7]);
		theta_5_count = solver5_Flag;
		printf("theta_5_count: %d\n",theta_5_count);

		//**解汇总************************************************************************************************************************************************************************
		//如果仅仅最后一个关节无解
		for(int i=0;i<8;i++)
		{
			//1到4列
			if(i<2)
			{
				joint[i][0] = theta_1[0];
				joint[i][1] = theta_2[0];
				joint[i][2] = theta_3[0];
				joint[i][3] = theta_4[i];
				joint[i][4] = theta_5[i];
			}
			else if(2<=i&&i<4)
			{
				joint[i][0] = theta_1[0];
				joint[i][1] = theta_2[1];
				joint[i][2] = theta_3[1];
				joint[i][3] = theta_4[i];
				joint[i][4] = theta_5[i];
			}
			else if(4<=i&&i<6)
			{
				joint[i][0] = theta_1[1];
				joint[i][1] = theta_2[0];
				joint[i][2] = theta_3[2];
				joint[i][3] = theta_4[i];
				joint[i][4] = theta_5[i];
			}
			else if(6<=i&&i<8)
			{
				joint[i][0] = theta_1[1];
				joint[i][1] = theta_2[1];
				joint[i][2] = theta_3[3];
				joint[i][3] = theta_4[i];
				joint[i][4] = theta_5[i];
			}
		}

		for(int j=0;j<8;j++)
		{
			printf("all solution:  %f %f %f %f %f\n",joint[j][0]*radTodeg,joint[j][1]*radTodeg,joint[j][2]*radTodeg,joint[j][3]*radTodeg,joint[j][4]*radTodeg);
		}

		for(int m=0;m<8;m++)
		{
			int solution_flag = 0;
			for(int n=0;n<5;n++)
			{
				if(joint[m][n] == 999)
				{
					break;
				}
				else
				{
					solution_flag ++;
				}
			}
			if(solution_flag == 5)
			{
				solution_flag = 0;
				//solution_count ++;
				solution_row = m;//找到最后一组有效解，并标记
			}
			
		}
		printf("solution_row:%d\n",solution_row);
		if(solution_row !=999)
		{
			printf("solution:  %f %f %f %f %f\n",joint[solution_row][0]*radTodeg,joint[solution_row][1]*radTodeg,joint[solution_row][2]*radTodeg,joint[solution_row][3]*radTodeg,joint[solution_row][4]*radTodeg);
			
			//Add verification function here
			float Verification_Value[5];
			for(int i=0;i<5;i++)
			{
				Verification_Value[i] = joint[solution_row][i];
			}
			int Verification_Flag = Direct_Kinematics_Verification(Verification_Value,P);
			if(Verification_Flag != 0)
			{
				solution_count ++;
				for(int q=0;q<5;q++)
				{
					(*p)[q] = joint[solution_row][q];
				}
			}
			else
			{
				printf("Out of work space!*************************** \n");
			}
		}
		else
		{
			printf("solution not found! \n");
		}

		printf("solution_count:%d\n",solution_count);

		//返回有效解得组数（共多少组有效解）
		return solution_count;
}


int rob_arm_ik_solver::PM_IK_Solver_Left_Arm(float P[3],float N[3],float O[3],float A[3], float (*p)[5])
{
	//**初始化预存值***************************************************************************************************************
		for(int i=0;i<2;i++)
		{
			theta_1[i] = 999;
		}
		for(int i=0;i<2;i++)
		{
			theta_2[i] = 999;
		}
		for(int i=0;i<4;i++)
		{
			theta_3[i] = 999;
		}
		for(int i=0;i<8;i++)
		{
			theta_4[i] = 999;
		}
		for(int i=0;i<8;i++)
		{
			theta_5[i] = 999;
		}
		//**变量设置************************************************************************************************************************************************
		int solution_count = 0;
		int solution_row = 999;
		int solver1_Flag = 0;//joint1解得个数
		int solver2_Flag = 0;//joint2解得个数
		int solver3_Flag = 0;//joint3解得个数
		int solver4_Flag = 0;//joint4解得个数
		int solver5_Flag = 0;//joint5解得个数

		//**解joint1**************************************************************************************************************************************************
		// Py*cos(s1) - Px*sin(s1) = 0 

		if(P[0] == 0 && P[1] == 0)
		{
			//can't solve out
			printf("theta_1 no solution! \n");
		}
		else if(P[0] != 0 && P[1] == 0)
		{
			//sin(s1)=0
			theta_1[0] = 0;
			theta_1[1] = pi;
			solver1_Flag =2;

			goto Joint1_solution;
		}
		else if(P[0] == 0 && P[1] != 0)
		{
			//cos(s1)=0
			theta_1[0] = pi/2;
			solver1_Flag = 1;

			goto Joint1_solution;
		}
		else
		{
			//tan(s1) = (Py/Px)
			float temp;
			temp = atan2(P[1],P[0]);
			int k;
			for(k=-10;k<11;k++)
			{
				float temp2 = temp +k*pi;
				//printf("k: %d\n",k);
				//printf("temp2: %f\n",temp2*radTodeg);
					
				if((-180*degTorad<=temp2)&&(temp2<=0))//rang of angle: [-180,0]
				{
					solver1_Flag ++;
					if(solver1_Flag ==1)//保存第一个解
					{
						theta_1[0] = temp2;
					}
					if(solver1_Flag ==2)//保存第二个解
					{
						theta_1[1] = temp2;
					}
				}
			}
		}

		Joint1_solution:

		printf("Joint1[2]: %f %f \n",theta_1[0],theta_1[1]);
		theta_1_count = solver1_Flag;
		printf("theta_1_count: %d\n",theta_1_count);

		//**解joint2*******************************************************************************************************************************************************************************
		//152.8*cos(s2) + 40*sin(s2) = Pz
		if(theta_1_count==0)//joint1 no solution
		{
			printf("Joint1 Joint2 Error!\n");
			solver2_Flag =0;
		}
		else
		{
			float a = 40;
			float b = 152.8;
			float fai;

			fai = atan2(b,a);
			//printf("fai =  %f \n",fai);
			//printf("fai(radTodeg) =  %f \n",fai*radTodeg);
			
			float Rho = sqrt(a*a+b*b);
			//printf("Rho =  %f \n",Rho);

			float temp1, temp2, temp3, temp4;

			temp1 = atan2(P[2]/Rho,sqrt(1-(P[2]/Rho)*(P[2]/Rho))) - fai;
			temp2 = atan2(P[2]/Rho,-sqrt(1-(P[2]/Rho)*(P[2]/Rho))) - fai;
			//printf("temp1 = %f\n",temp1*radTodeg);
			//printf("temp2 = %f\n",temp2*radTodeg);

			int k;
			for(k=-10;k<11;k++)
			{
				temp3 = temp1 +k*pi;
				temp4 = temp2 +k*pi;
				//printf("k: %d\n",k);
				//printf("temp3: %f\n",temp3*radTodeg);
				//printf("temp4: %f\n",temp4*radTodeg);
					
				if((0*degTorad<=temp3)&&(temp3<=91*degTorad))//rang of angle: [0,90]
				{
					solver2_Flag ++;
					theta_2[0] = temp3;

				}
				if((0*degTorad<=temp4)&&(temp4<=91*degTorad))//rang of angle: [0,90]
				{
					solver2_Flag ++;
					theta_2[1] = temp4;
				}
			}
		}

		printf("Joint2[2]: %f %f\n",theta_2[0],theta_2[1]);
		theta_2_count = solver2_Flag;
		printf("theta_2_count: %d\n",theta_2_count);

		//**joint3 joint4 joint5*******************************************************************************************************************************************************************************

		if((theta_1_count==0) || (theta_2_count==0))
		{
			printf("Joint1 or Joint2 No Solution!\n");
		}
		else
		{
			for (int i=0;i<4;i++)
			{
				switch (i)
				{
					case 0:
						float Joint0[3];
						if(theta_1[0] != 999 && theta_2[0] != 999)
						{
							if(PM_IK_Rotate_Matrix(N, O, A, theta_1[0], theta_2[0], &Joint0) == 1)
							{
								printf("Joint0[3]: %f %f %f\n",Joint0[0]*radTodeg,Joint0[1]*radTodeg,Joint0[2]*radTodeg);
								//save the result
								theta_3[0] = Joint0[0];
								theta_4[0] = Joint0[1];
								theta_5[0] = Joint0[2];

							}
							else
							{
								printf("Z-Y-Z Euler Angles solve error!\n");
							}
							break;
						}
						else
						{
							break;
						}

					case 1:
						float Joint1[3];
						if(theta_1[0] != 999 && theta_2[1] != 999)
						{
							if(PM_IK_Rotate_Matrix(N, O, A, theta_1[0], theta_2[1], &Joint1) == 1)
							{
								printf("Joint1[3]: %f %f %f\n",Joint1[0]*radTodeg,Joint1[1]*radTodeg,Joint1[2]*radTodeg);
								//save the result
								theta_3[1] = Joint1[0];
								theta_4[1] = Joint1[1];
								theta_5[1] = Joint1[2];
							}
							else
							{
								printf("Z-Y-Z Euler Angles solve error!\n");
							}
							break;
						}
						else
						{
							break;
						}
					case 2:
						float Joint2[3];
						if(theta_1[1] != 999 && theta_2[0] != 999)
						{
							if(PM_IK_Rotate_Matrix(N, O, A, theta_1[1], theta_2[0], &Joint2) == 1)
							{
								printf("Joint2[3]: %f %f %f\n",Joint2[0]*radTodeg,Joint2[1]*radTodeg,Joint2[2]*radTodeg);
								//save the result
								theta_3[2] = Joint2[0];
								theta_4[2] = Joint2[1];
								theta_5[2] = Joint2[2];
							}
							else
							{
								printf("Z-Y-Z Euler Angles solve error!\n");
							}
							break;
						}
						else
						{
							break;
						}
					case 3:
						float Joint3[3];
						if(theta_1[1] != 999 && theta_2[1] != 999)
						{
							if(PM_IK_Rotate_Matrix(N, O, A, theta_1[1], theta_2[1], &Joint3) == 1)
							{
								printf("Joint3[3]: %f %f %f\n",Joint3[0]*radTodeg,Joint3[1]*radTodeg,Joint3[2]*radTodeg);
								//save the result
								theta_3[3] = Joint3[0];
								theta_4[3] = Joint3[1];
								theta_5[3] = Joint3[2];
							}
							else
							{
								printf("Z-Y-Z Euler Angles solve error!\n");
							}
							break;
						}
						else
						{
							break;
						}

					default :
						break;
				}
			}
		}

		//**gather all the solution************************************************************************************************************************************************************************

		joint[0][0] = theta_1[0];
		joint[0][1] = theta_2[0];
		joint[0][2] = theta_3[0];
		joint[0][3] = theta_4[0];
		joint[0][4] = theta_5[0];

		joint[1][0] = theta_1[0];
		joint[1][1] = theta_2[1];
		joint[1][2] = theta_3[1];
		joint[1][3] = theta_4[1];
		joint[1][4] = theta_5[1];

		joint[2][0] = theta_1[1];
		joint[2][1] = theta_2[0];
		joint[2][2] = theta_3[2];
		joint[2][3] = theta_4[2];
		joint[2][4] = theta_5[2];

		joint[3][0] = theta_1[1];
		joint[3][1] = theta_2[1];
		joint[3][2] = theta_3[3];
		joint[3][3] = theta_4[3];
		joint[3][4] = theta_5[3];

		for(int j=0;j<4;j++)
		{
			printf("all solution:  %f %f %f %f %f\n",joint[j][0]*radTodeg,joint[j][1]*radTodeg,joint[j][2]*radTodeg,joint[j][3]*radTodeg,joint[j][4]*radTodeg);
		}

		for(int m=0;m<4;m++)
		{
			int solution_flag = 0;
			for(int n=0;n<5;n++)
			{
				if(joint[m][n] == 999)
				{
					break;
				}
				else
				{
					solution_flag ++;
				}
			}
			if(solution_flag == 5)
			{
				solution_flag = 0;
				//solution_count ++;
				solution_row = m;//找到最后一组有效解，并标记
			}
			
		}
		
		printf("solution_row:%d\n",solution_row);
		if(solution_row !=999)
		{
			printf("solution:  %f %f %f %f %f\n",joint[solution_row][0]*radTodeg,joint[solution_row][1]*radTodeg,joint[solution_row][2]*radTodeg,joint[solution_row][3]*radTodeg,joint[solution_row][4]*radTodeg);
			
			//Add verification function here
			float Verification_Value[5];
			for(int i=0;i<5;i++)
			{
				Verification_Value[i] = joint[solution_row][i];
			}
			int Verification_Flag = Direct_Kinematics_Verification(Verification_Value,P);
			if(Verification_Flag != 0)
			{
				solution_count ++;
				for(int q=0;q<5;q++)
				{
					(*p)[q] = joint[solution_row][q];
				}
			}
			else
			{
				printf("Out of work space!*************************** \n");
			}
		}
		else
		{
			printf("solution not found!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!! \n");
		}

		printf("solution_count:%d\n",solution_count);


		//返回有效解得组数（共多少组有效解）
		return solution_count;
}


//******************************************************************************************************************************************************
int PM_IK_Rotate_Matrix(float N[3],float O[3], float A[3], float s1,float s2, float (*J)[3])
{
	float r11, r12, r13, r21, r22, r23, r31, r32, r33;
	float Alpha, Beta, Gamma;
	float theta_3, theta_4, theta_5;
	float theta_3_temp1, theta_4_temp1, theta_5_temp1;
	float theta_3_temp2, theta_4_temp2, theta_5_temp2;
	int solution_flag = 0;
	
	r11 = (N[1]*cos(s1)) - (N[0]*sin(s1));
	r12 = (O[1]*cos(s1)) - (O[0]*sin(s1));
	r13 = (A[1]*cos(s1)) - (A[0]*sin(s1));
	r21 = - (N[2]*sin(s2)) - (N[0]*cos(s1)*cos(s2)) - (N[1]*cos(s2)*sin(s1));																															   
	r22 = - (O[2]*sin(s2)) - (O[0]*cos(s1)*cos(s2)) - (O[1]*cos(s2)*sin(s1));
	r23 = - (A[2]*sin(s2)) - (A[0]*cos(s1)*cos(s2)) - (A[1]*cos(s2)*sin(s1));
	r31 =   (N[2]*cos(s2)) - (N[0]*cos(s1)*sin(s2)) - (N[1]*sin(s1)*sin(s2));
	r32 =   (O[2]*cos(s2)) - (O[0]*cos(s1)*sin(s2)) - (O[1]*sin(s1)*sin(s2));
	r33 =   (A[2]*cos(s2)) - (A[0]*cos(s1)*sin(s2)) - (A[1]*sin(s1)*sin(s2));

	printf("r11-r33:  %f %f %f %f %f %f %f %f %f\n",r11, r12, r13, r21, r22, r23, r31, r32, r33);

	float temp = sqrt(r31*r31+r32*r32);
	printf("sqrt(r31*r31+r32*r32) = %f\n",temp);

	// Based on Z-Y-Z Euler Angles
	if(temp <= 0.01)
	{
		Alpha = 0;
		Beta = 0;
		Gamma = atan2(-r12,r11);
	}
	else
	{
		Beta = atan2(temp,r33);
		Alpha = atan2(r23,r13);

		printf("r23/r13 = %f\n",r23/r13);
		printf("tan(Alpha) = %f, tan(Alpha-pi) = %f  Alpha-pi = %f \n",tan(Alpha),tan(Alpha-pi),(Alpha-pi)*radTodeg);

		Gamma = atan2(r32,-r31);
	}

	printf("Alpha = %f, Beta = %f, Gamma = %f \n",Alpha*radTodeg,Beta*radTodeg,Gamma*radTodeg);

	//changge Alpha Beta Gamma to theta_3 theta_4 theta_5
	theta_3_temp1 = Alpha + pi/2;
	theta_4_temp1 = -Beta;
	theta_5_temp1 = Gamma;

	int k;
	for(k=-10;k<11;k++)
	{
		theta_3_temp2 = theta_3_temp1 +k*pi;
		if((0*degTorad<=theta_3_temp2) && (theta_3_temp2<=181*degTorad))//rang of angle: [0,180]
		{
			theta_3 = theta_3_temp2;
			solution_flag++;
		}
		theta_4_temp2 = theta_4_temp1 +k*pi;
		if(((-91*degTorad)<=theta_4_temp2)&&(theta_4_temp2<=0))//rang of angle: [-90,0]
		{
			theta_4 = theta_4_temp2;
			solution_flag++;
		}
		theta_5_temp2 = theta_5_temp1 +k*pi;
		if(((-91*degTorad)<=theta_5_temp2)&&(theta_5_temp2<=91*degTorad))//rang of angle: [-90,90]
		{
			theta_5 = theta_5_temp2;
			solution_flag++;
		}
	}

	if(solution_flag == 3)
	{
		(*J)[0] = theta_3;
		(*J)[1] = theta_4;
		(*J)[2] = theta_5;

		return 1;
	}
	else
	{
		return 0;
	}


}

//******************************************************************************************************************************************************
int Direct_Kinematics_Verification ( float J[5], float P_input[3])
{
	float s1, s2, s3, s4, s5;

	s1 = J[0];
	s2 = J[1];
	
	float P_out[4];
    //float N[4];
    //float O[4];
    //float A[4];

	float Difference;

	/*
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
	*/
	P_out[0] = 40*cos(s1)*cos(s2) - (764*cos(s1)*sin(s2))/5;
	P_out[1] = 40*cos(s2)*sin(s1) - (764*sin(s1)*sin(s2))/5;
	P_out[2] = (764*cos(s2))/5 + 40*sin(s2);
	P_out[3] = 1;

	Difference = sqrt((P_input[0]-P_out[0])*(P_input[0]-P_out[0]) + (P_input[1]-P_out[1])*(P_input[1]-P_out[1]) + (P_input[2]-P_out[2])*(P_input[2]-P_out[2]));

	printf("Difference = %f\n",Difference);

	//printf("(P_input[0]-P_out[0])*(P_input[0]-P_out[0]) = %f\n",(P_input[0]-P_out[0])*(P_input[0]-P_out[0]));
	//printf("(P_input[1]-P_out[1])*(P_input[1]-P_out[1]) = %f\n",(P_input[1]-P_out[1])*(P_input[1]-P_out[1]));
	//printf("(P_input[2]-P_out[2])*(P_input[1]-P_out[1]) = %f\n",(P_input[2]-P_out[2])*(P_input[2]-P_out[2]));

	if(Difference <= 150)
	{
		return 1;
	}
	else
	{
		return 0;
	}
}





