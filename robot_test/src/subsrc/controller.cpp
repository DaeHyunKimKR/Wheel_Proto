#include "controller.h"
#include <math.h>
#include <stdio.h>
#include <cmath>
#include <sync_read_write2.h>
#include <linear_read_write.h>


#define PI 3.141592

//pthread_mutex_t mutex;

CController::CController()
{		
    _mode = 0 , _mode2 = 0;
	_length_link1 = 0.3 , _length_link2 = 0.3 , _length_link3 = 0.2;
	_now_time4 = 0.0;
	_box =0 , _table =0 ;
	_z = 0.0 , _x = 0.0 , _y = 0.0 ;
	_x_e = 0.0, _y_e = 0.0, _z_e = 0.0, _alpha_e = 0.0;
	_gamma_1 = 0.0 , _k1_1 = 0.0 , _k2_1 = 0.0 ;
	_cos_q2 = 0.0 , _sin_q2_1 = 0.0 , _sin_q2_2 = 0.0 , _q2_1 = 0.0 , _q2_2 = 0.0 , _k1_1 = 0.0 , _k2_2 = 0.0 , _gamma_2 = 0.0 , _q1_1 = 0.0 , _q1_2 = 0.0 , _q3_1 = 0.0 , _q3_2 = 0.0 ;
	_linear_goal_position = 0;
	_new_mode = 0;
	_cnt = 0;
	_rev = true;
	for(int i = 0 ; i< 5 ; i ++)
	{
		_q[i] = 0.0 ; 
		_x_goal[i] = 0.0 ;
	}
	for(int i = 0 ; i< 5 ; i ++)
	{
		_dxl_present_position[i] = 0 ; 
		_goal_position[i] = 0 ;
	}

	_CurrentState = Ready_State;
	_PreviousState = Ready_State;
	_CurrentTask = No_Task;
	_init_time = 0.0;
	_operation_time = 0.0;

	_x_ready[0] = 0; //3000000
	_x_ready[1] = DEG2CNT(1,80.0);
	_x_ready[2] = DEG2CNT(2,-160.0);
	_x_ready[3] = DEG2CNT(3,-10.0);
	_x_ready[4] = 600;

	_x_init[0] = 0; //3000000
	_x_init[1] = DEG2CNT(1,70.0);
	_x_init[2] = DEG2CNT(2,-140.0);
	_x_init[3] = DEG2CNT(3,70.0);
	_x_init[4] = 0;

	//for moving left
	_x_init_left[0] = 0; //3000000
	_x_init_left[1] = DEG2CNT(1,-70.0);
	_x_init_left[2] = DEG2CNT(2,140.0);
	_x_init_left[3] = DEG2CNT(3,-70.0);
	_x_init_left[4] = 0;
	//for moving right
	_x_init_right[0] = 0; //3000000
	_x_init_right[1] = DEG2CNT(1,70.0);
	_x_init_right[2] = DEG2CNT(2,-140.0);
	_x_init_right[3] = DEG2CNT(3,70.0);
	_x_init_right[4] = 0;

	_x_Box_Lane[0] = 0;
	_x_Box_Lane[1] = DEG2CNT(1,-85.0);
	_x_Box_Lane[2] = DEG2CNT(2,-80.0);
	_x_Box_Lane[3] = DEG2CNT(3,-68.0);
	_x_Box_Lane[4] = 0;

	//1 : left front
	_x_Box_Lane_LF[0] = 0;
	_x_Box_Lane_LF[1] = DEG2CNT(1,45.0);
	_x_Box_Lane_LF[2] = DEG2CNT(2,110.0);
	_x_Box_Lane_LF[3] = DEG2CNT(3,58.0);
	_x_Box_Lane_LF[4] = 0;

	//2 : left back
	_x_Box_Lane_LB[0] = 0;
	_x_Box_Lane_LB[1] = DEG2CNT(1,85.0);
	_x_Box_Lane_LB[2] = DEG2CNT(2,80.0);
	_x_Box_Lane_LB[3] = DEG2CNT(3,68.0);
	_x_Box_Lane_LB[4] = 0;

	//3 : right front
	_x_Box_Lane_RF[0] = 0;
	_x_Box_Lane_RF[1] = DEG2CNT(1,-45.0);
	_x_Box_Lane_RF[2] = DEG2CNT(2,-110.0);
	_x_Box_Lane_RF[3] = DEG2CNT(3,-58.0);
	_x_Box_Lane_RF[4] = 0;

	//4 : right back
	_x_Box_Lane_RB[0] = 0;
	_x_Box_Lane_RB[1] = DEG2CNT(1,-85.0);
	_x_Box_Lane_RB[2] = DEG2CNT(2,-80.0);;
	_x_Box_Lane_RB[3] = DEG2CNT(3,-68.0);
	_x_Box_Lane_RB[4] = 0;

	//_x_Box_Lane_RF[0] = 0;
	//_x_Box_Lane_RF[1] = DEG2CNT(1,45.0);
	//_x_Box_Lane_RF[2] = DEG2CNT(2,110.0);
	//_x_Box_Lane_RF[3] = DEG2CNT(3,58.0);
	//_x_Box_Lane_RF[4] = 0;

	_x_Grab_Box[0] = M2CNT(0,-0.18); //8000000;
	_x_Grab_Box[1] = DEG2CNT(1,-85.0);
	_x_Grab_Box[2] = DEG2CNT(2,-80.0);
	_x_Grab_Box[3] = DEG2CNT(3,-68.0);
	_x_Grab_Box[4] = 0;

	//_x_Grab_Box_RF[0] = M2CNT(0,-0.18); //8000000;
	//_x_Grab_Box_RF[1] = DEG2CNT(1,45.0);
	//_x_Grab_Box_RF[2] = DEG2CNT(2,110.0);
	//_x_Grab_Box_RF[3] = DEG2CNT(3,58.0);
	//_x_Grab_Box_RF[4] = 0;
	
	for(int i = 0 ; i< 5 ; i ++)
	{
		_x_goal[i] = _x_init[i];
	}
	////////// Subscribe and Publish //////////
	/////////////// initialize ////////////////
	//_is_subscribed = false;
	///////////////////////////////////////////

	_task_state = 0;
	_task_tray_cmd = 0;
	_pre_tray_cmd = 0;
	_target_position_from_vision[0] = 0.0;
	_target_position_from_vision[1] = 0.0;
	_target_position_from_vision[2] = 0.0;

}

CController::~CController()
{
}

//void ROSinit(ros::NodeHandle n)
//{
	////////// Subscribe and Publish //////////
	/////////////// initialize ////////////////
	//_nh_ptr = &n;
	//_pub = _nh_ptr->advertise<std_msgs::Int32>("/pub_topic", 1);
	//_sub = _nh_ptr->subscribe("/sub_topic", 1, &CController::SubscribeCallBack, this);
//}

// subscribe call back routine function
// the variable '_is_subscribed; is become true when subscribed the topic
// so you need to set the variable to false when the specific task is finish!
//void CController::SubscribeCallBack(const std_msgs::Int32ConstPtr& msg)
//{
//	ROS_INFO("Received!! : [%ld]", msg->data);
//	_is_subscribed = true;
//}

double CController::RANGE(double angle) 
{
	while (angle > PI || angle <= -PI) 
	{
		if (angle > PI) 
		{
			angle = angle - 2 * PI;
		}
		else 
		{
			angle = angle + 2 * PI;
		}
	}
	return angle;
}

int CController::getValue()
{
	//도시락 고르기
	int Box_Number;
	std::cout << "\n 도시락 선택 4(↖)3(↗) \n             2(↙)1(↘) :";
	std::cin >> Box_Number;
	return Box_Number;
}

int CController::getValue2()
{
	//놓을 테이블 고르기
	int Table_Number;
	std::cout << "\n 놓을 테이블 선택 1(→),2(←):";
	std::cin >> Table_Number;
	return Table_Number;

}

double CController::M2CNT(double joint_num, double m)
{
	int m2cnt_result = 0;
	double resollution_LC_300_13 = 25466667.0;

	if(joint_num == 0)
	{
		m2cnt_result = (int) resollution_LC_300_13*(-m);
	}
	return m2cnt_result;

}

double CController::CNT2M(double joint_num, double cnt)
{
	double cnt2m_result = 0;
	double resollution_LC_300_13 = 25466667.0;

	if(joint_num == 0)
	{
		cnt2m_result = cnt/resollution_LC_300_13;
	}
	return cnt2m_result;

}

double CController::CNT2RAD(double joint_num, double cnt)
{
	double rad = 0.0;
	double resollution_PH42_020_S300_R = 607500.0;
	double resollution_PM42_010_S260_R = 526374.0;
	double cnt_per_deg_PH42_020_S300_R = resollution_PH42_020_S300_R/360.0;
	double cnt_per_deg_PM42_010_S260_R = resollution_PM42_010_S260_R/360.0;
	if(joint_num == 1 || joint_num == 2)//Dynamixel PH42-020-S300-R
	{
		rad = DEG2RAD(joint_num, (cnt / cnt_per_deg_PH42_020_S300_R));
	}
	else if(joint_num == 3) //Dynamixel PM42-010-S260-R
	{
		rad = DEG2RAD(joint_num, (cnt / cnt_per_deg_PM42_010_S260_R));
	}
	else{

	}
	return rad;
}

int CController::DEG2CNT(double joint_num, double deg)
{
	//this function is only for dynmamixel motor
	int cnt_result = 0;
	double resollution_PH42_020_S300_R = 607500.0;
	double resollution_PM42_010_S260_R = 526374.0;
	double cnt_per_deg_PH42_020_S300_R = resollution_PH42_020_S300_R/360.0;
	double cnt_per_deg_PM42_010_S260_R = resollution_PM42_010_S260_R/360.0;

	if(joint_num == 1 || joint_num == 2)//Dynamixel PH42-020-S300-R
	{
		cnt_result = (int) cnt_per_deg_PH42_020_S300_R*deg;
	}
	else if(joint_num == 3) //Dynamixel PM42-010-S260-R
	{
		cnt_result = (int) cnt_per_deg_PM42_010_S260_R*deg;
	}
	else{

	}
	return cnt_result;
}

double CController::DEG2RAD(double joint_num, double deg)
{
	double rad_result = 0;

	if(joint_num < 4)
	{
		rad_result = (double) deg*(PI/180);
	}
	return rad_result;
}

double CController::RAD2DEG(double joint_num, double rad)
{
	double deg_result = 0;

	if(joint_num < 4)
	{
		deg_result = (double) (180*rad)/PI;
	}
	return deg_result;
}

void CController::inverseKin(double x, double y, double alpha)
{
	_x = x - _length_link3 * cos(alpha);
	_y = y - _length_link3 * sin(alpha);

	_cos_q2 = (pow(_x, 2) + pow(_y, 2) - pow(_length_link1, 2) - pow(_length_link2, 2)) / (2 * _length_link1 * _length_link2);
	//double dist_xy = sqrt(_x*_x + _y*_y);

//if (abs(_cos_q2) > 1) || dist_xy > 0.79
	if (abs(_cos_q2) > 1 || _x < 0.25 || abs(_y) < 0.2 || abs(alpha) > 90.0*(PI/180))
	{
		std::cout << "Out of Workspace." << endl;
		_q[0] = _x_init[1];// {0.0, 0.0, 0.0 };
		_q[1] = _x_init[2];
		_q[2] = _x_init[3];
	}
	else {

		_sin_q2_1 = sqrt(1 - pow(_cos_q2, 2));
		_sin_q2_2 = -sqrt(1 - pow(_cos_q2, 2));

		_q2_1 = atan2(_sin_q2_1, _cos_q2);
		_q2_2 = atan2(_sin_q2_2, _cos_q2);

		double q2[2] = { _q2_1, _q2_2 };
		_k1_1 = _length_link2 * cos(_q2_1) + _length_link1;
		_k1_2 = _length_link2 * cos(_q2_2) + _length_link1;
		//double k1[2] = { k1_1,k1_2 };
		// k1
		_k2_1 = _length_link2 * sin(_q2_1);
		_k2_2 = _length_link2 * sin(_q2_2);
		//double k2[2] = { k2_1, k2_2 };
		// k2
		_gamma_1 = atan2(_k2_1, _k1_1);
		_gamma_2 = atan2(_k2_2, _k1_2);
		
		//double gamma[2] = { gamma_1,gamma_2 };
		_z = atan2(_y, _x);
		_q1_1 = _z - _gamma_1;
		_q1_2 = _z - _gamma_2;
		_q3_1 = alpha - _q1_1 - _q2_1;
		_q3_2 = alpha - _q1_2 - _q2_2;
		double q1[2] = { _q1_1,_q1_2 };
		//q1
		double q3[2] = { _q3_1,_q3_2 };
		//q3
		double q_1[4] = { 0, q1[0],q2[0],q3[0] };
		double q_2[4] = { 0, q1[1],q2[1],q3[1] };
		for (int i = 1; i < 4; i++) {
			q_1[i] = RANGE(q_1[i]);
			q_2[i] = RANGE(q_2[i]);
			if (_rev == true)
			{
				_q[i] = q_2[i];
				//std::cout << q_2[3] << std::endl;
			}
			else
			{
				_q[i] = q_1[i];
				//std::cout << "_reverse" << std::endl;
			}
			// _q[i]
			// 1 이냐 2냐 고를수 있음
		}
	}

}

void CController::forwardKin(double q1, double q2, double q3)
{
	_x_e = _length_link1 * cos(q1) + _length_link2 * cos(q1 + q2) + _length_link3 * cos(q1 + q2 + q3);
	_y_e = _length_link1 * sin(q1) + _length_link2 * sin(q1 + q2) + _length_link3 * sin(q1 + q2 + q3);
	_alpha_e = q1 + q2 + q3;
}

void CController::get_present_position(int32_t dxl_present_position[])
{
	//std::cout << dxl_present_position[1] << std::endl;

		for(int i = 1 ; i<5 ; i++)
	{
		_dxl_present_position[i] = dxl_present_position[i] ;		
	}
		//std::cout << _dxl_present_position[4] << std::endl;
}

void CController::get_linear_present_position(int linear_present_position)
{
		_dxl_present_position[0] = linear_present_position ;	
}


void CController::Finite_State_Machine( int button, float axes1, float axes2, double time)
{ 	//std::cout <<time << std::endl;
	//std::cout << _CurrentState << std::endl;
	//std::cout<< button << axes1 << axes2 << std::endl;
	//std::cout << _new_mode << std::endl;

	//TODO: ros joy => what is button, axes1, axes2? Check

	if(_CurrentState == Ready_State)
	{
		
		_x_goal[0] = _x_ready[0];
		_x_goal[1] = _x_ready[1];
		_x_goal[2] = _x_ready[2];
		_x_goal[3] = _x_ready[3];
		_x_goal[4] = 700;
		

		if( button == 1)
		{

			//cout << "Task Cmd: Initial Pose" <<endl;
			//_CurrentTask = Init_Pose_Task;
			//_CurrentState = Gripper_Close_State;
			//_init_time = time;						
			//cout << "Start <Gripper Close>" <<endl;
			//_PreviousState = Ready_State;
			
		}
		else if ( axes1 < 0 ) //왼쪽버튼 누를시
		{
			//cout << "Task Cmd: Pick and Load Snack" <<endl;
			//_CurrentTask = Pick_and_Load_Snack;
			//_CurrentState = Initial_State;
			//_init_time = time;			
			//cout << "Start <Grab Box>" <<endl;
			//_PreviousState = Ready_State;
		}
		else if ( axes2 < 0 ) //오른쪽버튼 누를시
		{
			//_CurrentState = TEST_State;
			//_init_time = time;

			//cout << "Task Cmd: Forward Kinematics" <<endl;
			//_CurrentTask = FK_Task;
			//_CurrentState = FK_State;
			//_init_time = time;
		}		
		else 
		{
			//cout << "Task Cmd: None" <<endl;
			_CurrentTask = No_Task;
			_CurrentState = Ready_State;
			_PreviousState = Ready_State;
		}

		if(time < 10.0)
		{
			_task_tray_cmd = 0;
		}

		if(_task_tray_cmd == 0 || _task_tray_cmd == 1 || _task_tray_cmd == 2 || _task_tray_cmd == 3 || _task_tray_cmd == 4)
		{
			//cout << _pre_tray_cmd << " " << _task_tray_cmd <<endl;
			if(_pre_tray_cmd !=_task_tray_cmd)
			{
				_CurrentTask = Pick_and_Load_Snack;
				_CurrentState = Initial_State;
				_init_time = time;			
				cout << "Start <Grab Box>" <<endl;
				_PreviousState = Ready_State;

				_task_state = 0;
			}
			_pre_tray_cmd = _task_tray_cmd;
		}
		
		_task_state = 2;
	}	


	// TEST State
	else if(_CurrentState == TEST_State)
	{
		double time_per = 10.0;
		if(time <= _init_time+time_per)
		{

			// 첫번째 좌표
			IK_x = 0.30; // x좌표
			IK_y = 0.0; // y좌표
			IK_z = 0.0; // z좌표
			//

			inverseKin(IK_x, IK_y, 0.0); //solve IK to get _q[1]~[3]
			IK_q1 = DEG2CNT(1,RAD2DEG(1,_q[1]));
			IK_q2 = DEG2CNT(2,RAD2DEG(1,_q[2]));
			IK_q3 = DEG2CNT(3,RAD2DEG(1,_q[3]));

			_x_goal[0] = M2CNT(0,IK_z);	
			_x_goal[1] = IK_q1;
       		_x_goal[2] = IK_q2;
        	_x_goal[3] = IK_q3;
		}
		else if(_init_time+time_per*2.0 < time && time <= _init_time+time_per*2.0)
		{
			// 21
			IK_x = 0.45; // x좌표
			IK_y = -0.50; // y좌표
			IK_z = 0.0; // z좌표
			//

			inverseKin(IK_x, IK_y, 0.0); //solve IK to get _q[1]~[3]
			IK_q1 = DEG2CNT(1,RAD2DEG(1,_q[1]));
			IK_q2 = DEG2CNT(2,RAD2DEG(1,_q[2]));
			IK_q3 = DEG2CNT(3,RAD2DEG(1,_q[3]));

			_x_goal[0] = M2CNT(0,IK_z);	
			_x_goal[1] = IK_q1;
       		_x_goal[2] = IK_q2;
        	_x_goal[3] = IK_q3;
		}

		else if(time > _init_time+time_per*2.0)
		{
			_CurrentState = Ready_State;
			_init_time = time;
		}
	}



	// FK state
	else if(_CurrentState == FK_State)
	{
		///////////////////////////////////////////////////////////////////////
		// FK_q1 = 90.0;
		// FK_q2 = 70.0;
		// FK_q3 = 0.0;
		
		// _x_goal[0] = 0;
    	// _x_goal[1] = DEG2CNT(1,FK_q1);
        // _x_goal[2] = DEG2CNT(2,FK_q2);
        // _x_goal[3] = DEG2CNT(3,FK_q3);

		// forwardKin( DEG2RAD(1,FK_q1), DEG2RAD(2,FK_q2), DEG2RAD(3,FK_q3));
		// cout << "" <<endl;
		// cout << "x = " << _x_e <<endl;
		// cout << "y = " << _y_e <<endl;
		// cout << "angle = " << FK_q1 + FK_q2 + FK_q3 <<endl;
		// cout << "q1 rad = " << DEG2RAD(1,FK_q1) <<endl;
		// cout << "q2 rad = " << DEG2RAD(2,FK_q2) <<endl;
		// cout << "q3 rad = " << DEG2RAD(3,FK_q3) <<endl;
		// cout << "" <<endl;
		/////////////////////////////////////////////////////////////////////////////


		std::cout << "Please enter x,y,z values : ";
		std::cin >> IK_x;
		std::cin >> IK_y;
		std::cin >> IK_z;
		std::cout << "x = " << IK_x << ", " << "y = " << IK_y << ", " << "z = " << IK_z << std::endl; 
		inverseKin(IK_x, IK_y, 0.0); //solve IK to get _q[1]~[3]
		IK_q1 = DEG2CNT(1,RAD2DEG(1,_q[1]));
		IK_q2 = DEG2CNT(2,RAD2DEG(1,_q[2]));
		IK_q3 = DEG2CNT(3,RAD2DEG(1,_q[3]));

		_x_goal[0] = M2CNT(0,IK_z);	
		_x_goal[1] = IK_q1;
        _x_goal[2] = IK_q2;
        _x_goal[3] = IK_q3;
		


		/*
		_operation_time = 7.0; //TODO: check time
		if(time < _init_time + _operation_time)
		{
			_x_goal[0] = 3000000;
    		_x_goal[1] = DEG2CNT(1,45.0);
        	_x_goal[2] = DEG2CNT(2,90.0);
        	_x_goal[3] = DEG2CNT(3,0.0);

			forwardKin( DEG2CNT(1,45.0), DEG2CNT(2,45.0), DEG2CNT(3,0.0));
			cout << "" <<endl;
			cout << _x_e <<endl;
			cout << _y_e <<endl;
			cout << _alpha_e <<endl;
			cout << "" <<endl;

			//	_x_e = _length_link1 * cos(q1) + _length_link2 * cos(q1 + q2) + _length_link3 * cos(q1 + q2 + q3);
			//	_y_e = _length_link1 * sin(q1) + _length_link2 * sin(q1 + q2) + _length_link3 * sin(q1 + q2 + q3);
			//	_alpha_e = q1 + q2 + q3;
		}
		else if(time >= _init_time + _operation_time)
		{
			_CurrentState = Ready_State;
			_init_time = time;
			cout << "Terminate <Forward Kinematics>" <<endl;
		}
		*/
	}



    // initial state
	else if(_CurrentState == Initial_State)
	{
		_operation_time = 5.0; //TODO: check time
		if(time < _init_time + _operation_time)
		{
			_x_goal[0] = _x_init[0];
            _x_goal[1] = _x_init[1];
            _x_goal[2] = _x_init[2];
            _x_goal[3] = _x_init[3];

			if(_task_tray_cmd == 1 || _task_tray_cmd == 2)
			{
				_x_goal[0] = _x_init[0];
				_x_goal[1] = _x_init[1];
				_x_goal[2] = _x_init[2];
				_x_goal[3] = _x_init[3];
			}
			else if(_task_tray_cmd == 3 || _task_tray_cmd == 4)
			{
				_x_goal[0] = _x_init[0];
				_x_goal[1] = _x_init[1];
				_x_goal[2] = _x_init[2];
				_x_goal[3] = _x_init[3];
			}			
		}
		else if(time >= _init_time + _operation_time)
		{
			cout << "Terminate <Initial>" <<endl;
			if(_CurrentTask == Init_Pose_Task || _CurrentTask == No_Task)
			{
				_CurrentState = Ready_State;
				_init_time = time;
				cout << "Start <Ready>" <<endl;
			}
			else if(_CurrentTask == Pick_and_Load_Snack)
			{
				if(_PreviousState == Ready_State)
				{
					_CurrentState = Box_Lane;
					cout << "Start <Move to Box Lane>" <<endl;
				}
				else if(_PreviousState == Box_Lane)
				{
					_CurrentState = Drop_Point;
					cout << "Start <Drop>" <<endl;
				}
				else if(_PreviousState == Linear_Up)
				{
					_task_state = 2;

					_CurrentState = Ready_State;
					cout << "Start <Ready>" <<endl;
				}				
				_init_time = time;				
			}
			_PreviousState = Initial_State;
		}
	}


    // Box Lane
	else if(_CurrentState == Box_Lane)
	{
		_operation_time = 3.5; //TODO: check time
		if(time < _init_time + _operation_time)
		{
			_x_goal[0] = _x_Box_Lane[0];
            _x_goal[1] = _x_Box_Lane[1];//-130017;
            _x_goal[2] = _x_Box_Lane[2];//274084;
            _x_goal[3] = _x_Box_Lane[3];

			if(_task_tray_cmd == 1)
			{
				_x_goal[0] = _x_Box_Lane_LF[0];
				_x_goal[1] = _x_Box_Lane_LF[1];//-130017;
				_x_goal[2] = _x_Box_Lane_LF[2];//274084;
				_x_goal[3] = _x_Box_Lane_LF[3];
			}
			else if(_task_tray_cmd == 2)
			{
				_x_goal[0] = _x_Box_Lane_LB[0];
				_x_goal[1] = _x_Box_Lane_LB[1];//-130017;
				_x_goal[2] = _x_Box_Lane_LB[2];//274084;
				_x_goal[3] = _x_Box_Lane_LB[3];
			}
			else if(_task_tray_cmd == 3)
			{
				_x_goal[0] = _x_Box_Lane_RF[0];
				_x_goal[1] = _x_Box_Lane_RF[1];//-130017;
				_x_goal[2] = _x_Box_Lane_RF[2];//274084;
				_x_goal[3] = _x_Box_Lane_RF[3];
			}
			else if(_task_tray_cmd == 4)
			{
				_x_goal[0] = _x_Box_Lane_RB[0];
				_x_goal[1] = _x_Box_Lane_RB[1];//-130017;
				_x_goal[2] = _x_Box_Lane_RB[2];//274084;
				_x_goal[3] = _x_Box_Lane_RB[3];
			}
		}
		else if(time >= _init_time + _operation_time)
		{
			cout << "Terminate <Box_Lane>" <<endl;	
			
			if(_PreviousState == Initial_State)
			{
				_CurrentState = Gripper_Open_State;
				_init_time = time;				
				cout << "Start <Gripper Open>" <<endl;
			}
            else if(_PreviousState == Gripper_Close_State)
			{
				_CurrentState = Initial_State;
				_init_time = time;
				cout << "Start <Initial>" <<endl;
			}			
			_PreviousState = Box_Lane;
		}		
	}

    // Grab Box
	else if(_CurrentState == Grab_Box)
	{
		_operation_time = 5.0; //TODO: check time
		if(time < _init_time + _operation_time)
		{
			_x_goal[0] = _x_Grab_Box[0];
            _x_goal[1] = _x_Grab_Box[1];//-130017;
            _x_goal[2] = _x_Grab_Box[2];//274084;
            _x_goal[3] = _x_Grab_Box[3];

			if(_task_tray_cmd == 1)
			{
				_x_goal[0] = _x_Grab_Box[0];
				_x_goal[1] = _x_Box_Lane_LF[1];//-130017;
				_x_goal[2] = _x_Box_Lane_LF[2];//274084;
				_x_goal[3] = _x_Box_Lane_LF[3];
			}
			else if(_task_tray_cmd == 2)
			{
				_x_goal[0] = _x_Grab_Box[0];
				_x_goal[1] = _x_Box_Lane_LB[1];//-130017;
				_x_goal[2] = _x_Box_Lane_LB[2];//274084;
				_x_goal[3] = _x_Box_Lane_LB[3];
			}
			else if(_task_tray_cmd == 3)
			{
				_x_goal[0] = _x_Grab_Box[0];
				_x_goal[1] = _x_Box_Lane_RF[1];//-130017;
				_x_goal[2] = _x_Box_Lane_RF[2];//274084;
				_x_goal[3] = _x_Box_Lane_RF[3];
			}
			else if(_task_tray_cmd == 4)
			{
				_x_goal[0] = _x_Grab_Box[0];
				_x_goal[1] = _x_Box_Lane_RB[1];//-130017;
				_x_goal[2] = _x_Box_Lane_RB[2];//274084;
				_x_goal[3] = _x_Box_Lane_RB[3];
			}
		}
		else if(time >= _init_time + _operation_time)
		{
			cout << "Terminate <Box_Lane>" <<endl;	
			
			if(_CurrentTask == Pick_and_Load_Snack)
			{
				_CurrentState = Gripper_Close_State;
				_init_time = time;				
				cout << "Start <Gripper Close>" <<endl;
			}
			_PreviousState = Grab_Box;
		}		
	}

    // Drop Point
	else if(_CurrentState == Drop_Point)
	{
		_operation_time = 4.0; //TODO: check time
		if(time < _init_time + _operation_time)
		{
			_x_goal[0] = 3000000;
            _x_goal[1] = DEG2CNT(1,30.0);//130017;
            _x_goal[2] = DEG2CNT(2,-60.0);//-274084;
            _x_goal[3] = DEG2CNT(3,30.0);
			

			//abs
			double dist_arm = sqrt(_target_position_from_vision[0]*_target_position_from_vision[0] + _target_position_from_vision[1]*_target_position_from_vision[1]);
			if(dist_arm > 0.78)
			{
				_target_position_from_vision[0] = 0.11;
				_target_position_from_vision[1] = -0.2;
			}

			double pos_threshold_vertical = 0.1;
			if(_target_position_from_vision[2] >= 0 - pos_threshold_vertical || _target_position_from_vision[2] <= -0.21) //z
			{
				_target_position_from_vision[2] = -0.21;
			}
			

			// if(_task_tray_cmd == 1 ||_task_tray_cmd == 2) //moving left
			// {
			// 	inverseKin(_target_position_from_vision[0], _target_position_from_vision[1], 60.0*DEG2RAD); //solve IK to get _q[1]~[3]
			// }
			// else if(_task_tray_cmd == 3 ||_task_tray_cmd == 4)
			// {
			// 	inverseKin(_target_position_from_vision[0], _target_position_from_vision[1], -60.0*DEG2RAD); //solve IK to get _q[1]~[3]
			// }

			inverseKin(_target_position_from_vision[0], _target_position_from_vision[1], -60.0*(PI/180)); //solve IK to get _q[1]~[3]
			_x_goal[0] = M2CNT(0, _target_position_from_vision[2]+pos_threshold_vertical-0.02);
			
			IK_q1 = DEG2CNT(1,RAD2DEG(1,_q[1]));
			IK_q2 = DEG2CNT(2,RAD2DEG(1,_q[2]));
			IK_q3 = DEG2CNT(3,RAD2DEG(1,_q[3]));
			
			_x_goal[1] = IK_q1;
            _x_goal[2] = IK_q2;
            _x_goal[3] = IK_q3;		
		}
		else if(time >= _init_time + _operation_time)
		{
			cout << "Terminate <Drop>" <<endl;
			
			if(_CurrentTask == Pick_and_Load_Snack)
			{
				_CurrentState = Gripper_Open_State;
				_init_time = time;				
				cout << "Start <Gripper Open>" <<endl;
			}
			_PreviousState = Drop_Point;
		}		
	}

	// Linear Up
	else if(_CurrentState == Linear_Up)
	{
		_operation_time = 1.0; //TODO: check time
		
		if(time < _init_time + _operation_time)
		{
			_x_goal[0] = 0;			
		}
		else if(time >= _init_time + _operation_time)
		{
			
			_CurrentState = Initial_State;
			_init_time = time;
			cout << "Start <Initial>" <<endl;

			_PreviousState = Linear_Up;
		}		
		_task_tray_cmd = 0;
		_pre_tray_cmd = 0;
	}

    // Gripper Close
	else if(_CurrentState == Gripper_Close_State)
	{
		_operation_time = 1.0; //TODO: check time
		
		if(time < _init_time + _operation_time)
		{
			_x_goal[4] = 400;			
		}
		else if(time >= _init_time + _operation_time)
		{
			cout << "Terminate <Gripper Close>" <<endl;

			if(_CurrentTask == Init_Pose_Task)
			{
				_CurrentState = Initial_State;
				_init_time = time;
				cout << "Start <Initial>" <<endl;
			}
            else if(_CurrentTask == Pick_and_Load_Snack)
			{
				_CurrentState = Box_Lane;
				_init_time = time;
				cout << "Start <Move To Box Lane>" <<endl;
			}
			_PreviousState = Gripper_Close_State;
		}		
	}
    
	// Gripper Open
	else if(_CurrentState == Gripper_Open_State)
	{
		_operation_time = 1.5; //TODO: check time
		
		if(time < _init_time + _operation_time)
		{
			_x_goal[4] = 0;			
		}
		else if(time >= _init_time + _operation_time)
		{
			cout << "Terminate <Gripper Open>" <<endl;
			if(_CurrentTask == Pick_and_Load_Snack)
			{
				if(_PreviousState == Box_Lane)
				{
					_CurrentState = Grab_Box;
					cout << "Start <Grab Box>" <<endl;
				}
				else if(_PreviousState == Drop_Point)
				{
					_task_state = 1;

					_CurrentState = Linear_Up;
					cout << "Start <Linear Up>" <<endl;
				}				
				_init_time = time;				
			}
			_PreviousState = Gripper_Open_State;
		}		
	}
	
}
