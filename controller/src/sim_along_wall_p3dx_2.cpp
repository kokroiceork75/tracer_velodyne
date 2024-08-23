//////////////////////////////  include  ///////////////////////////////////////////
#include <sstream>
#include <iostream>
#include <algorithm>
#include "ros/ros.h"
#include <std_msgs/Float64.h> 
#include "std_msgs/String.h"
#include "sensor_msgs/LaserScan.h"
#include "geometry_msgs/Twist.h"
#include <fstream>
#include <math.h>
#include <move_base_msgs/MoveBaseActionFeedback.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Int32.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
//////////////////////////////  include  ///////////////////////////////////////////


//////////////////////////////  define區  ////////////////////////////////////////
#define MIN(a,b) (a<b?a:b)
#define _max_rule 30
#define _rule_number 10   /* 規則數 */ 
int _in_clu;//FC rules
#define _in_varl 4
#define _out_varl 2  //輸出個數 
//#define max_step 5000   /* 最大步數 */  
#define max_step 40000
#define _rule_delta 10
#define _input_scale 1.0   //20200416   
#define PI 3.14159    
#define r 0.05
#define tran r*2*PI/60
#define left_wheel_speed 4  //如果控制器的後件部s有正規化則要*最高速
#define max_speed 50.0
//////////////////////////////  define區  /////////////////////////////////////////
// #define load_data_clu "controller/20211027/2/clu.txt"
// #define load_data_FC "controller/20211027/2/Final_new.txt"
// #define load_data_clu "controller/20220105/Ev1/clu_10.txt"
// #define load_data_FC "controller/20220105/Ev2/Final_new_19.txt"
//#define load_data_clu "controller/20220209/Ev2/_in_clu_19.txt"
//#define load_data_FC "controller/20220209/Ev2/Final_first_19.txt"
#define load_data_clu "controller/nsga_v1/10.txt"

//#define load_data_FC "controller/20230502/wall.txt"
//#define load_data_clu "controller/20230712/20230712experirments/MEFRL(simulation2)/_in_clu.txt"
//#define load_data_FC "controller/20230712/20230712experirments/MEFRL(simulation2)/s.txt"

#define load_data_FC "controller/nsga_v1/temp_w_50.txt"  // wei change for max speed = 50.0
// #define load_data_FC "controller/nsga_v1/temp_w1.txt"  // wei for max speed = 15.6
//#define load_data_FC "controller/20221118/test.txt"
// #define load_data_clu "controller/20210803/1/clu.txt"
// #define load_data_FC "controller/20210803/1/Final_first_pos.dat"
//#define save_wall "controller/20230502/wall.txt"
//#define save_vel "controller/20230502/vel.txt"
#define save_ave_vel "controller/nsga_v1/save_ave_speed.txt"
#define save_ave_error "controller/nsga_v1/save_ave_dis_error.txt" 
//#define save_data_path1  "save/along_wall.txt" 

//////////////////////////////  變數宣告區  ///////////////////////////////////////{
double in[_in_varl+1] ;

double out_y[_out_varl+1] ; //輸出項暫存變數//左右輪輸出
double robot_vel[max_step+1] ;
double  sick_1[max_step+1], 
		sick_2[max_step+1], 
		sick_3[max_step+1], 
		sick_4[max_step+1]; 

double  sick_all[max_step+1], 
		sick_wall[max_step+1] ;

float far=5;
//const float PI=3.14;
const float x=9;

const float dis= 0.57; // wei change width

double ave_speed=0.0;
float ave_distance_error=0;

const int _mem_length = 2*_in_varl+ _out_varl ;
long double fuzzy_real[_mem_length*_max_rule+1] ;
double min_m[_in_varl+1],max_m[_in_varl+1];//最小中心點 & 最大中心點
int ber=1 ; //代表第幾個解，此為一個替代的暫存變數
double deviation_whirl = 0.; //輪速差

int steps,status;


int counts=0;
int angle_left_min,angle_right_min ;
double left_min;
double right_min;
double stright_min;
//double error_z;
double back_left,back_right;
float laser_temp[361];
float laser_temp_scan[897];
double read_1;
double read_2;
double read_3;
double read_4;
double read_5;
double read_6;
double amcl_orientation_x,amcl_orientation_y,amcl_orientation_z,amcl_position_x,amcl_position_y,amcl_position_z;
double v1;
double v2;
double position_x1,position_y1,orientation_z1;

double roll_s,pitch_s,yaw_s;

int decision_left =0,decision_right =0;
int counts_left=0 ,counts_right=0;

using namespace std;
inline float phi(float x,float m,float v)
{ return(  exp( - (x-m)*(x-m)/(v*v) )) ; }

class C_Rule
{
public :

	double  in_mu ;
	double  con[_out_varl+1] ;
	void    In_mem_fir(double *,int) ;
	friend  void Fir_str(double *, int,int) ;
	friend  void fuzzy(double *,int,int);

} ;

C_Rule  _Rule[_max_rule+1] ;


void C_Rule::In_mem_fir(double *in,int _rule) 
{
  
	int i ;
	in_mu =  1. ;
	
		for(i=1;i<= _in_varl;i++)
		{
			if(in[i] < min_m[i] || in[i] > max_m[i])
			 {
			 	in_mu = in_mu * 1;
			 }
			 else
			 {
			 	in_mu = in_mu * phi(in[i] , fuzzy_real[(_rule-1)*_mem_length+i*2-1] , fuzzy_real[(_rule-1)*_mem_length+i*2] ) ;
			 }
			//in_mu = in_mu * phi(in[i] , fuzzy_real[(_rule-1)*_mem_length+i*2-1] , fuzzy_real[(_rule-1)*_mem_length+i*2] ) ;//20211215

		}
	//}
}




class controller{
	public:
////////////////////////////////////////////  Constructor  //////////////////////////////////////////////	
		controller(){

			x1_pub=n.advertise<std_msgs::Float64>("data_x1",3000);
			y1_pub=n.advertise<std_msgs::Float64>("data_y1",3000);
			sub_final_goal = n.subscribe("/move_base_simple/goal", 3000, &controller::Final_Goal,this);
		    sub=n.subscribe("/scan",3000,&controller::callback,this);
			//sub=n.subscribe("/scan",3000,&controller::callback,this);
	    	pub_data1=n.advertise<std_msgs::Float64>("data1",3000);
			pub_data2=n.advertise<std_msgs::Float64>("data2",3000);
			sub_amcl = n.subscribe("/move_base/feedback", 3000, &controller::amcl_Callback,this);	
			//pub=n.advertise<geometry_msgs::Twist>("/3_2",3000);

    		pub=n.advertise<geometry_msgs::Twist>("/cmd_vel_2",3000); //cmd_vel_2

			//sub_orientation =n.subscribe("chatter1",3000, &controller::chatter1Callback,this);
			sub_info1 =n.subscribe("chatter1",3000, &controller::chatterCallback,this);
		}
////////////////////////////////////////////  Constructor  //////////////////////////////////////////////	
// void Final_Goal(const geometry_msgs::PoseStamped::ConstPtr & F_goal) //全域目標的X Y Z 的數值
//   {
//     position_x1=F_goal->pose.position.x;
//     position_y1=F_goal->pose.position.y;
//     //orientation_z1=F_goal->pose.orientation.z;
// 	tf2::Quaternion s;
//   	tf2::convert(F_goal->pose.orientation,s);
// 	tf2::Matrix3x3(s).getRPY(roll_s,pitch_s,yaw_s);
// 	orientation_z1=yaw_s;
     
//     if( orientation_z1 <=-1.5707 && orientation_z1>=-3.14159 )
// 	    orientation_z1= -(1.5707+( 3.14159+orientation_z1)); //第四象限為90~180
//     else
// 	    orientation_z1 = -(orientation_z1 -1.5707) ;  
//   }
void Final_Goal(const geometry_msgs::PoseStamped::ConstPtr & F_goal) //全域目標的X Y Z 的數值
  {
    position_x1=F_goal->pose.position.x;
    position_y1=F_goal->pose.position.y;
    //orientation_z1=F_goal->pose.orientation.z;
	tf2::Quaternion s;
  	tf2::convert(F_goal->pose.orientation,s);
	tf2::Matrix3x3(s).getRPY(roll_s,pitch_s,yaw_s);
	orientation_z1=yaw_s;
     
    if( orientation_z1 <=-1.5707 && orientation_z1>=-3.14159 )
	    orientation_z1= (1.5*M_PI)+orientation_z1; //第四象限為90~180
    else
	    orientation_z1 = (orientation_z1 -1.5707) ;  
  }
///////////////////////////////  get distance value from different angles from sensor ///////////////////////
	double minimun(int i, int j , int &k  )
   {  
      double laser_min=100;
      for (k=i ;k<=j;k++){
         
         if (laser_min>laser_temp[k])
            { 
			  laser_min=laser_temp[k];
			  
			}
      }
	  return laser_min;
      

   }
		// void callback(const sensor_msgs::LaserScan::ConstPtr& scan){
		// 	int k;
		// 	for(int i=1;i<=360;i++){//180
		// 		laser_temp[i]=scan->ranges[i]; ///202009 180 to 270

		// 	}	
        //    left_min=minimun(210,270,k);

		//    right_min=minimun(90,150,k);

		//    stright_min=minimun(150,210,k);
		// }
		void callback(const sensor_msgs::LaserScan::ConstPtr& scan) //wei change,not sure
		{
			int k;
			for(int i=1;i<=scan->ranges.size();i++)
			{
				laser_temp_scan[i]=scan->ranges[i]; ///202009 180 to 270
			}
			for (int i = 0; i <= scan->ranges.size(); ++i)
			{
			double angle_rad = (scan->angle_min + i * scan->angle_increment) + M_PI;
			int angle_deg = angle_rad*180/M_PI;
			if (!std::isinf(laser_temp_scan[i]))
			{
						if (laser_temp_scan[i]!=0)
						{
							laser_temp[angle_deg] = laser_temp_scan[i];
						}
			}		
			}
           left_min=minimun(195,269,k);
		   right_min=minimun(90,165,k);
		   stright_min=minimun(165,195,k);
		}
		void decision_(int j){	
			double xx1,xx2;
			if (stright_min<=0.5 || left_min<=0.5 || right_min<=0.5){
				if (left_min<=0.5){
					decision_left = 1;
					decision_right= 0;
				}
				else if (right_min<=0.5){
					decision_left = 0;
					decision_right= 1;
				
				}
			    else if (left_min<right_min && left_min <0.5){
					decision_left = 1;
					decision_right= 0;
				}
				else if (left_min>right_min && right_min<0.5){
					decision_left = 0;
					decision_right= 1;
				}
			}
		
			 if (status==1){
				
				 if ( left_min <=0.5 ) 
				{
					decision_left = 1;
					decision_right = 0;
				}
				else if( right_min <= 0.5){
					decision_left = 0;
					decision_right = 1;
				}
			}

		    else if(right_min < 0.5 && laser_temp[270] > 0.5 )// && laser_temp[90] < 0.7)// && decision_left == 0  )
			{	
			 	decision_right = 1;
				decision_left  = 0;
			
			}
			else if(left_min <0.5 && laser_temp[90]> 0.5)// && laser_temp[270] < 0.7 )//&& decision_right == 0 )
		    {
				decision_left = 1;
				decision_right= 0;
				
			}
		
			if(decision_left == 1 ){	
			std::cout << "LLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLL" << std::endl;
			printf("left_min:  %lf\n",left_min);
			printf("right_min:  %lf\n",right_min);

			read_1 = laser_temp[192] ;
			read_2 = laser_temp[218] ;
			read_3 = laser_temp[232] ;
			read_4 = laser_temp[261] ;
			read_5 = laser_temp[270] ;
			read_6 = laser_temp[180] ;

			for(size_t i = 180; i < 200; i++)//25
			{	if(read_1>laser_temp[i])
				read_1=std::min(laser_temp[i],far);
			}
			//printf("%d\n",1);

			for(size_t i = 200; i < 225; i++)//20
			{	if(read_2>laser_temp[i])
				read_2=std::min(laser_temp[i],far);
			}
			//printf("%d\n",2);
			for(size_t i = 225; i < 245; i++)//25
			{	if(read_3>laser_temp[i])
				read_3=std::min(laser_temp[i],far);
			}
			//printf("%d\n",3);
			for(size_t i = 245; i < 270; i++)//20
			{	if(read_4>laser_temp[i])
				read_4=std::min(laser_temp[i],far);
			}
			//printf("%d\n",4);
			for(size_t i = 90; i <= 270; i++)
			{	if(read_5>laser_temp[i])
				read_5=std::min(laser_temp[i],far);
			}			
			//printf("%d\n",5);
			for(size_t i = 90; i <= 180; i++)//175 180
			{	if(read_6>laser_temp[i])
				read_6=std::min(laser_temp[i],far);
			}
			printf("left_along_wall\n");
			}
			if(decision_right == 1){
			std::cout << "RRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRR" << std::endl;
			read_1 = laser_temp[158] ;
			read_2 = laser_temp[137] ;
			read_3 = laser_temp[122] ;
			read_4 = laser_temp[105] ;
			read_5 = laser_temp[180] ;
			read_6 = laser_temp[1] ;

			for(size_t i = 160; i <179; i++)//25
			{	if(read_1>laser_temp[i])
				read_1=std::min(laser_temp[i],far);
			}
			//printf("%d\n",1);

			for(size_t i = 135; i < 160; i++)//20
			{	if(read_2>laser_temp[i])
				read_2=std::min(laser_temp[i],far);
			}
			//printf("%d\n",2);
			for(size_t i = 115; i < 135; i++)//25
			{	if(read_3>laser_temp[i])
				{
				read_3=std::min(laser_temp[i],far);
				}	
			}
			//printf("%d\n",3);
			for(size_t i = 90; i < 115; i++)//20
			{	if(read_4>laser_temp[i]){
				
				read_4=std::min(laser_temp[i],far);
				}
			}
			//printf("%d\n",4);
			for(size_t i = 90; i <=270; i++)
			{	if(read_5>laser_temp[i])
				read_5=std::min(laser_temp[i],far);
			}			
			//printf("%d\n",5);
			for(size_t i = 90; i <= 180; i++)//175 180
			{	if(read_6>laser_temp[i])
				read_6=std::min(laser_temp[i],far);
			}
			 printf("right_along_wall\n");	

		}
		//for(int i=1)
		//	cout<<<<endl;
		}
				
///////////////////////////////  get distance value from different angles from sensor  ///////////////////////

		
	

////////////////////////////////////  Output cmd_velocity  ////////////////////////////////////
		void test_run(int j){
			geometry_msgs::Twist msg;				
			// v1=tran*out_y[2];
			// v2=tran*out_y[1];
			// printf(" LeftWheel_Speed == %f\n",x*v2);
			// printf(" RightWheel_Speed == %f\n",x*v1);//左右輪輸出
			// msg.linear.x =x*(v1+v2)/2;
			// msg.angular.z=x*(v1-v2)/dis;
			if(decision_left == 1 ){
			v1 = out_y[1]*0.03*0.7;
			v2 = out_y[2]*0.03*0.7;
			}
			if(decision_right == 1 ){
			 v1 = out_y[2]*0.03*0.7;
			 v2 = out_y[1]*0.03*0.7;
			}
		
			//printf("linear=%lf\n",(v1+v2)/2);
			//printf("angular=%lf\n",(v1-v2)/dis);
		
			/*if(((v1-v2)/dis)>0.85){
				msg.linear.x =((v1+v2)/2)*0.5;//( 0.85/((v1-v2)/dis))
				msg.angular.z = 0.85;
			}
			else if(((v1-v2)/dis)<-0.85){
				msg.linear.x =((v1+v2)/2)*0.5;
				msg.angular.z = -0.85;
			}
			else {
				msg.linear.x =(v1+v2)/2;
				msg.angular.z = (v1-v2)/dis;

			}*/
			msg.linear.x =(v1+v2)/2 ;
			if((v1-v2)/dis > 1. )  msg.angular.z = 1.; 
			else if ((v1-v2)/dis <-1.) msg.angular.z =-1.;
			else
			 msg.angular.z = (v1-v2)/dis;
			pub.publish(msg);
     
			//if (((v1+v2)/2)==0 && (v1-v2)/dis==0 && decision_left==1) msg.angular.z = -0.2;
			//else if (((v1+v2)/2)==0 && (v1-v2)/dis==0 && decision_right==1) msg.angular.z = 0.2;
		
			printf("left=%d\t right=%d\n",decision_left,decision_right);
            printf("speed=%lf \t %lf \n",out_y[1],out_y[2]);
			printf("linear=%lf\n",msg.linear.x);
			printf("angular=%lf\n",msg.angular.z);
			pub.publish(msg);
////////////////////////////////////  Output cmd_velocity  ////////////////////////////////////

//////////////////////////////////// Record the average speed /////////////////////////////////				
			/*FILE *pfout;
			pfout=fopen(save_vel,"a");//"controller/20210701/vel.txt"
			if(pfout==NULL){
				printf("Fail to open file");
				exit (1);
			}
			fprintf(pfout,"%f\n",msg.linear.x);
			fclose(pfout);*/
//////////////////////////////////// Record the average speed /////////////////////////////////	
			ave_speed = ave_speed + (v1+v2)*0.5;			
		}
///////////////////////////////////  Open the file from leaning ///////////////////////////////
		void chatterCallback(const std_msgs::Int32::ConstPtr& info1){
         status =  info1->data;
   		}
		
		void publisher_info(int jj ) //印出訊息並傳遞數值給其他程式從pub1~pub10;
  		{        
    	std_msgs::Float64 info1,info2;
    	info1.data = (v1+v2)/2;  //車子與目標的角度差
    	info2.data = (v1-v2)/dis;              //經過轉換後的機器人Z軸角度
  
    	pub_data1.publish(info1); 
    	pub_data2.publish(info2);
  		}

		void test_open(){

			for(int ssss=1; ssss<=_in_varl; ssss++)
			{
					min_m[ssss]=1.0;  //為了找最小值，所以初始的最小值令為1，即為"最大"的最小值。
					max_m[ssss]=0.0;  //為了找最大值，所以初始的最大值令為0，即為"最小"的最大值。		
			////-1~1之間的中心點範圍 
			//max_m[ber][ssss]=0.0;  //為了找最大值，所以初始的最大值令為-1，即為"最小"的最大值。        
					
			}
			 
			FILE *fnoise1,*fnoise0;  
			printf("read file\n");		
			if((fnoise0=fopen(load_data_clu,"r"))==NULL)
			{
				printf("Nothing1\n");
				exit(1);
			}
			fscanf(fnoise0,"%d", &_in_clu);
			fclose(fnoise0);
			
			if((fnoise1=fopen(load_data_FC,"r"))==NULL)
			{
				printf("Nothing2\n");
				exit(1);
			}
			for(int i=1;i<=_mem_length*_in_clu;i++)
			{
				fscanf(fnoise1,"%Lf \n", &fuzzy_real[i]);
				//printf("check%Lf\t",fuzzy_real[i]);
			}  

			fclose(fnoise1);  

			for(int jj=1; jj<=_rule_delta; jj++)
			{
				for(int jjj=1; jjj<=_in_varl; jjj++)
					{   
					if(fuzzy_real[(jj-1)*_mem_length+jjj*2-1] < min_m[jjj])
					{   
						min_m[jjj]=fuzzy_real[(jj-1)*_mem_length+jjj*2-1];
					}
					if(fuzzy_real[(jj-1)*_mem_length+jjj*2-1] > max_m[jjj])
					{
						max_m[jjj]=fuzzy_real[(jj-1)*_mem_length+jjj*2-1];
					}
				}

			}
			
		}
///////////////////////////////////  Open the file from leaning ///////////////////////////////

///////////////////////////////////  Save sensor data to another vector & print out  /////////////////////////
		void get_sensor(int jj)
		{
			sick_1[jj] = read_1;
			sick_2[jj] = read_2; 
			sick_3[jj] = read_3;
			sick_4[jj] = read_4;	
			sick_all[jj] = read_5;
			sick_wall[jj] = read_6;
			if(jj>=5){
	
				ave_distance_error = ave_distance_error + fabs(read_6-0.35);
				printf("distance_error == %f\n",fabs(read_6-0.35));
			}
			robot_vel[jj] =out_y[1]+out_y[2]; //功能:左輪+右輪的輪速

			deviation_whirl=out_y[1]-out_y[2];
		}
		void sick_limit(int jj) 
		{
			double _limit = 1.0;  //20200505  5.0 to 1.0

			if (sick_1[jj] >= _limit)
				sick_1[jj] = _limit ;
			
			if (sick_2[jj] >= _limit)
				sick_2[jj] = _limit ;
			
			if (sick_3[jj] >= _limit)
				sick_3[jj] = _limit ;

			if (sick_4[jj] >= _limit)
				sick_4[jj] = _limit ;	

			if (sick_all[jj] >= _limit)
				sick_all[jj] = _limit ;
					
			if (sick_wall[jj] >= _limit)
				sick_wall[jj] = _limit ;


			printf(" ~~~~~~~~~~~~~~~~~~~~~~~~~~ \n" ) ;
			
			printf("Sick 1 === %f\n" , 		sick_1[counts] );  
			printf("Sick 2 === %f\n" , 		sick_2[counts] );  
			printf("Sick 3 === %f\n" , 		sick_3[counts] );
			printf("Sick 4 === %f\n" , 		sick_4[counts] );
			printf("Sick wall === %f\n" ,sick_wall[counts] ) ;
			printf("~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~");
			printf("\n");
			printf("laser[91] === %f\n" ,laser_temp[91] ) ;
			printf("laser[179] === %f\n" ,laser_temp[179] ) ;
			printf("laser[181] === %f\n" ,laser_temp[181] ) ;
			printf("laser[269] === %f\n" ,laser_temp[269] ) ;
			printf("laser[270] === %f\n" ,laser_temp[270] ) ;
			

			printf("\nleft_min===%f\n",left_min);
			printf("right_min===%f\n",right_min);
			printf("stright_min=%f\n",stright_min);
			
		}
///////////////////////////////////  Save sensor data to another vector & print out  ///////////////////////////


//////////////////////////////////  Rescale sensor data for fuzzy  /////////////////////////////////////////
		void fuzzy_in(int jj)
		{   
		//感測器讀值最大為5m (參閱副程式: get_sensor ) ，因此乘上1.0代表把所有讀值壓縮到 0~1 之間。  

			in[1] = sick_1[jj] * _input_scale ; 
			in[2] = sick_2[jj] * _input_scale ;  
			in[3] = sick_3[jj] * _input_scale ;  
			in[4] = sick_4[jj] * _input_scale ;
	//		in[5] = sick_wall[jj] * _input_scale ;//平移再normalize//2017_8_2
			
			//for(int i=1;i<=5;ㄑi++)
			//	printf("in[%d]= %lf\n",i,in[i]);//	//左右輪最高速為7.8			
		}
//////////////////////////////////  Rescale sensor data for fuzzy  /////////////////////////////////////////



///////////////////////////////// Calculate firing strengh  ////////////////////////////////////////////
		void Fir_str(double *in, int _rule, int ber)     
		{
			//這裡的ber，等於main函式內的ber，又ber的值為popsize的值。
			//這裡的 _rule，等於設定變數的_rule_number的值。
			//這裡的in，代表輸入的變數的數量

			int j,k ;

			for(k=1; k<=_rule; k++)
			{
				for (j=1; j<=_out_varl; j++)
				{
					_Rule[k].con[j] = fuzzy_real[k*_mem_length-_out_varl+j];
				} // 讀取後件部值
				_Rule[k].In_mem_fir(in,k) ; // 計算激發量
				
			}
		
		}
///////////////////////////////// Calculate firing strengh  ////////////////////////////////////////////

////////////////////////////////////// speed limit    //////////////////////////////////////
		void speed_limit()
		{
			if(out_y[1] >= max_speed)
				out_y[1] = max_speed;
			if(out_y[2] >= max_speed)
				out_y[2] = max_speed;
			if(out_y[1] <= -(max_speed))
				out_y[1] = -max_speed;
			if(out_y[2] <= -(max_speed))
				out_y[2] = -max_speed;
		}
////////////////////////////////////// speed limit    //////////////////////////////////////

///////////////////////////////////////  Defuzzifier  ///////////////////////////////////////
		void fuzzy(int _rule , int ber)  // 權重式平均法
		{
			int i , j;
			double den[_out_varl+1] , num[_out_varl+1] ;

			
			for (j=1; j<=_out_varl; j++)
			{
				den[j] = 0. ;
				num[j] = 0. ;
				
				for (i=1; i<=_rule; i++)
				{
					//if ( fuzzy_xrule[ber][i] == _true )
				//	{
						num[j] = num[j] + _Rule[i].in_mu * _Rule[i].con[j];
						den[j] = den[j] + _Rule[i].in_mu ;
				//	}
				}


				if ( fabs(den[j]) < 1e-8 )  //20211215
					out_y[j] = 0 ;
				else
					out_y[j] = num[j]/den[j] ;
				
				//out_y[j] = out_y[j]*left_wheel_speed ;//如果控制器的後件部有正規化則要*最高速
			}
			speed_limit();
			//out_y[1] = out_y[1]*0.4;
			//out_y[2] = out_y[2]*0.4;
			printf("out_y1=%lf \t out_y2=%lf\n ",out_y[1],out_y[2]);

		}
///////////////////////////////////////  Defuzzifier  ///////////////////////////////////////u


//////////////////////////////////  Stop the car  ///////////////////////////////////////
		void stop(){
			if(counts==max_step)
				{

					geometry_msgs::Twist msg;

					msg.linear.x =0;
					msg.angular.z=0;

					pub.publish(msg);
					
					//exit(1);
				}
		}
//////////////////////////////////  Stop the car  ///////////////////////////////////////
		void save_speed(){
			FILE *pfout2;
			pfout2=fopen(save_ave_vel,"a");//"controller/20210701/ave_ave_speed.txt"
			if(pfout2==NULL){
				printf("Fail to open file");
				exit (1);
			}
			fprintf(pfout2,"%f\n",(ave_speed/(double)(counts-4)));
			fclose(pfout2);
		}
		void save_wall_dis(){
			FILE *pfout3;
			printf("111111111111111111111\n");
			pfout3=fopen(save_ave_error,"a");//"controller/20210701/save_ave_dis_error.txt"
			if(pfout3==NULL){
				printf("Fail to open file");
				exit (1);
			}
			fprintf(pfout3,"%f\n",(ave_distance_error/(float)(counts-4)));
			fclose(pfout3);
		}
	    void amcl_Callback(const::move_base_msgs::MoveBaseActionFeedback::ConstPtr & amcl){
      		amcl_position_x = amcl->feedback.base_position.pose.position.x;
      	    amcl_position_y = amcl->feedback.base_position.pose.position.y;
      		amcl_position_z = amcl->feedback.base_position.pose.position.z;
      		amcl_orientation_x = amcl->feedback.base_position.pose.orientation.x;
     		amcl_orientation_y = amcl->feedback.base_position.pose.orientation.y;
      		amcl_orientation_z = amcl->feedback.base_position.pose.orientation.z; 
      		
			/*FILE *pfout5;
      		printf("\n\nread file\n");
      		pfout5=fopen(save_data_path1,"a");
			if(pfout5==NULL){
				printf("Fail to open file");
				exit(1);
			}
         fprintf(pfout5,"%lf \t %lf\n",amcl_position_x,amcl_position_y);
         fclose(pfout5);*/

			std_msgs::Float64 data_x1 ,data_y1;
			data_x1.data = amcl_position_x;
		    data_y1.data = amcl_position_y;
			cout<<"data_x1= "<<data_x1<<endl;
            
			cout<<"data_y1= "<<data_y1<<endl;
			x1_pub.publish(data_x1);
            y1_pub.publish(data_y1);
  		 }	

	private: 

		ros::NodeHandle n;
		
		ros::Subscriber sub;
		
		ros::Subscriber sub_amcl;
		
		ros::Publisher pub;
		ros::Publisher pub_data1;
		ros::Publisher pub_data2;
		ros::Subscriber sub_orientation;
		ros::Subscriber sub_info1;
		ros::Subscriber sub_final_goal;

		ros::Publisher x1_pub;
		ros::Publisher y1_pub;
		ros::Publisher x2_pub;
		ros::Publisher y2_pub;

};
//controller  _Rule[_max_rule+1] ;

int main(int argc,char ** argv)
{
    ros::init(argc,argv,"controller");
	
    controller ctrl;
	//ros::Rate a(100);  
	
	while(counts<max_step && ros::ok())
	{

		
			counts++;
		
		printf("counts:%d\n",counts);
		//ros::spinOnce();
		
		ctrl.test_open(); //to get max and min
		ctrl.decision_(counts);
		ctrl.get_sensor(counts) ;
		ctrl.sick_limit(counts) ;//20200505
		ctrl.fuzzy_in(counts) ;					//輸入是雷測數值，只是調整fuzzy的input比例			
		ctrl.Fir_str(in , _in_clu , ber) ;	//讀取後件部計算機發量
		ctrl.fuzzy(_in_clu , ber);  		//解模糊產生out_y
		ctrl.publisher_info(counts);
		//ctrl.test_run();  //to do work
		if(counts<5){
			ctrl.stop();
			out_y[1]=0;
			out_y[2]=0; ///202010
		}
		else{
			ctrl.test_run(counts);
		}
		
	   ros::spinOnce();
      ros::Duration(0.01).sleep();
	  //sros::Duration(0.01).sleep(); //訊息傳送間隔0.01秒

	//	a.sleep();
		
	}
	ctrl.save_speed();
	ctrl.save_wall_dis();

	ctrl.stop();
	

    return 0;
}
