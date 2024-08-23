//ave_distance_error_sum 改成用90度最小值的誤差 save_robot_wall 離牆距離存檔 改回原本的
//1018 速度項改了
//1023 out_y[]初始化
///202011 time 170 to 100 to 170 202012 雷射方向改
///min_right 加40度 延牆0.35m
///////////////// include 區 //////////////////{ 

#include <math.h>
#include <cmath>
#include <stdio.h> 
#include <stdlib.h>
#include <iostream>
#include <string>
#include <chrono>
#include <time.h>
#include <algorithm> 
//////////////Ros////////////////////
#include <ros/ros.h>   
//#include "b-spline.h"

////#include <std_msgs/String.h> 
#include <std_msgs/Float64.h> 
#include <std_msgs/Int32.h> 
#include <geometry_msgs/Twist.h>
#include "sensor_msgs/LaserScan.h"
#include <move_base_msgs/MoveBaseActionFeedback.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <nav_msgs/Odometry.h>

/////////////// include 區 //////////////////} 

//////////////////////////////  define區  ///////////////////////////////////////{

#define save_distance      "position_random/2easy-distance.txt"  //左/右/前測距離
#define save_local_target  "position_random/local_target-111.txt"//區域目標
#define save_local_target_change  "position_random/local_target-change1.txt"//區域目標
#define save_old "position_random/old_.txt"
#define save_new "position_random/new_.txt"
#define load_data_clu "controller/nsga_v1/10.txt"
#define load_data_FC "controller/search_target_controller/FUZZY_PID/temp_w.txt" //important controller.
#define position_target    "position_random/position_targetxx.txt"
//////////////////  演算法參數  /////////////////////{

#define max_step 40000
#define _out_varl 6   /*  輸出變數 (左輪&右輪) */
#define _in_varl  3   /*  輸入變數    */  
#define _rule_number 10   /* 規則數 */ 
#define  _max_rule  30    /* 開記憶體空間 */
#define _rule_delta 10 

///////////////////  演算法參數  /////////////////////}


///////////////////  速度項目  /////////////////////{

#define left_wheel_speed 50.0  /* 左輪輪速 */
#define right_wheel_speed  50.0   /* 右輪輪速  */

///////////////////  速度項目  /////////////////////}
 
#define Target_X -3
#define Target_Y -3


/////////////////  常見函數  /////////////////////{

#define MIN(a,b) (a<b?a:b)
#define _input_scale (1/sensor_range)  /* 給感測器讀值正規化的參數  */
#define randomize() srand((unsigned) time(NULL)) /*  以時間為基底產生隨機值的初始化，不要動這行  */
#define random(x) rand() %x   /*  隨機產生亂數，不要動這行  */

/////////////////   常見函數  /////////////////////}


//////////////////////////////  define區  ///////////////////////////////////////}


//////////////////////////////  變數宣告區  ///////////////////////////////////////{


////////////////  左右輪  //////////////////{

double weight_speed;
double in[_in_varl+1] ; //感測器正規化後的輸入
double out_y[_out_varl+1] ; //馬達輸出

////////////////  左右輪  //////////////////}



////////////////  演算法相關  //////////////////{

const int _mem_length = 2*_in_varl+ _out_varl ; // 一個rule的長度

int _in_clu ; //rule_number的暫存變數	
int status=100;
int counts=0;
int a,ik,ss=111;
int HH;
double final_z ;

double fuzzy_real[_mem_length*_max_rule+1] ; //  實際輸出規則的變數
double K[_out_varl+1] ; 
double  ave_motor_speed=0.0;  //平均輪速
double  ave_motor_displacement=0.0;  //平均位移 ///202011
double min_m[_in_varl+1],max_m[_in_varl+1];//最小中心點 & 最大中心點	
double trans_new_amcl;//重要
double trans_degree;
double error_z,trans_angle_z;
const float dis=0.57;
double orientation_z[max_step+1],
       position_x[max_step+1],
       position_y[max_step+1],
       orientation_w[max_step+1],
       error_orientation_z,error_final_z,Error_orientation_z[max_step+1];
double amcl_orientation_z, amcl_position_x, amcl_position_y, amcl_position_z;
double error_position_x ,error_position_y;
double amcl_x[max_step+1],amcl_y[max_step+1];
double position_x1, position_y1,orientation_z1;
double final_position_x,final_position_y,final_orientation_z;
double error, turn,new_turn ,intergral ,derivative,lastError,error_1 ,derivative_1 ,lastError_1,intergral_1;     
double v1,v2;
///double position_xx[40000],position_yy[40000],change_xx[40000],change_yy[40000];
double angular,linear;
double laser_temp[361],margin_laser[361],left_min,right_min,straight_min;
float laser_temp_scan[897];
double Tp=0,angle_deviate;
double rms,local_rms;
double roll,pitch,yaw;
double roll_g,pitch_g,yaw_g;
double roll_s,pitch_s,yaw_s;
double angular1,linear1;
double sum_error;
int js =0;
double position_xx1[max_step+1], position_yy1[max_step+1];
using namespace std;
geometry_msgs::Twist msg;
///////////////// 高斯函數 ////////////////////{

inline double phi(double x,double m,double v)
{ return(  exp( - (x-m)*(x-m)/(v*v) )) ; }

///////////////// 高斯函數 ////////////////////}

class C_Rule
{
friend class Population ;
public :

  double  in_mu_1, in_mu_2;
  double  con[_out_varl+1] ;
  void    In_mem_fir(double *,int) ;
  friend  void Fir_str(double *, int) ;
} ;

C_Rule  _Rule[_max_rule+1] ;
////////////////////// Fuzzy系統:讀取後件部 //////////////////////////}
void C_Rule::In_mem_fir(double *in,int _rule)  
{
//這裡的 _rule，等於設定變數的_rule_number的值，而且這裡的_rule是一個動態變數，所以會隨著 Fir_str 的 k 值做改變。
  
 
int i ,l;
in_mu_1 =  1. ;  // "1." 代表1的float值
in_mu_2 =  1. ;
    for(i=1;i<= _in_varl;i++)
    {      
      in_mu_1 =  in_mu_1 * phi(in[i] , fuzzy_real[(_rule-1)*_mem_length+i*2-1] , fuzzy_real[(_rule-1)*_mem_length+i*2] ) ;
   
     // in_mu_2 =  in_mu_2 * phi(in[l] , fuzzy_real[(_rule-1)*_mem_length+l*2-1] , fuzzy_real[(_rule-1)*_mem_length+l*2] ) ;
    }  
/////////////////////////// 其他8個rule的情況  /////////////////////////}    
}

////////////////////// Fuzzy系統:計算激發量 //////////////////////////}
class amcl_pose_sub_pub_1 
{
private:
    ros::NodeHandle n;  
    ros::Publisher pub; 
    ros::Publisher pub_goal; 
    ros::Publisher chatter_pub1;
    ros::Publisher chatter_pub2;
    ros::Publisher chatter_pub3;
    ros::Subscriber sub_final_goal;
    ros::Subscriber sub_path;
    ros::Subscriber sub_amcl;
    ros::Subscriber sub_laser;
    ros::Subscriber sub_odom0;
    ros::Subscriber sub_odom1;
    ros::Subscriber sub_data1;
    ros::Subscriber sub_data2;
    ros::Subscriber sub_data3;
    ros::Subscriber sub_data4;
public:
  amcl_pose_sub_pub_1()
  {
    pub=n.advertise<geometry_msgs::Twist>("/cmd_vel_1",3000); // cmd_vel_1

    pub_goal=n.advertise<geometry_msgs::PoseStamped>("/move_base_simple/goal",1);
    chatter_pub1=n.advertise<std_msgs::Int32>("chatter1",3000);
    chatter_pub2=n.advertise<std_msgs::Float64>("chatter2",3000);
    chatter_pub3=n.advertise<std_msgs::Float64>("chatter3",3000);
    
    sub_laser=n.subscribe("/scan",3000,&amcl_pose_sub_pub_1::lasercallback,this);
    //sub_laser=n.subscribe("/scan",3000,&amcl_pose_sub_pub_1::lasercallback,this);
    sub_final_goal = n.subscribe("/move_base_simple/goal", 3000, &amcl_pose_sub_pub_1::Final_Goal,this);

    //subxxx = n.subscribe("/move_base/global_costmap/costmap", 3000,&amcl_pose_sub_pub_1::costmapCallback,this);

    //sub_path = n.subscribe("/move_base/DWAPlannerROS/global_plan", 3000, &amcl_pose_sub_pub_1::Path_Callback,this);
    sub_path = n.subscribe("/move_base/GlobalPlanner/plan", 3000, &amcl_pose_sub_pub_1::Path_Callback,this);
    sub_amcl = n.subscribe("/move_base/feedback", 3000, &amcl_pose_sub_pub_1::amcl_Callback,this);
    //sub_odom0 = n.subscribe("/odom",3000, &amcl_pose_sub_pub_1::old_Callback,this);
   // sub_odom1 = n.subscribe("/odometry/filtered",3000, &amcl_pose_sub_pub_1::new_Callback,this);
    sub_data1 = n.subscribe("/info_1",3000, &amcl_pose_sub_pub_1::chatter1Callback1, this);
    sub_data2 = n.subscribe("/info_2",3000, &amcl_pose_sub_pub_1::chatter1Callback2, this); 
    sub_data3 = n.subscribe("/info_3",3000, &amcl_pose_sub_pub_1::chatter1Callback3, this);
    sub_data4 = n.subscribe("/info_4",3000, &amcl_pose_sub_pub_1::chatter1Callback4, this);
  


///////////////////////  Topic訂閱  ////////////////////////{
  
  void Final_Goal(const::geometry_msgs::PoseStamped::ConstPtr& F_goal);
  void lasercallback(const::sensor_msgs::LaserScan::ConstPtr& scan);
  void Path_Callback(const ::nav_msgs::Path::ConstPtr & msg);
  void amcl_Callback(const::move_base_msgs::MoveBaseActionFeedback::ConstPtr & amcl);

///////////////////////  Topic訂閱  ////////////////////////}

/////////////////////// 速度運算  ////////////////////////{
  void Search_Target();
  void speed_limit(void) ;   // 速度限制器
  void Final_stop();
/////////////////////// 速度運算  ////////////////////////}

///////////////////////  模糊相關  ////////////////////////{

  void Fir_str(double *, int) ; //Fuzzy系統:讀取後件部與計算激發量
  void fuzzy(int);    // Fuzzy系統:解模糊
  void fuzzy_in(int) ;  // Fuzzy系統:把感測器的讀值做正規化

///////////////////////  模糊相關  ////////////////////////}

///////////////////////  存檔相關  ////////////////////////{

  void test_open();
  void save_robot_wheelspeed(int);  //左右輪輪速的存檔
  void save_robot_displacement(int);  //離牆距離的存檔  ///202011
  void old_Callback(const nav_msgs::Odometry::ConstPtr& msg);
  void new_Callback(const nav_msgs::Odometry::ConstPtr& msg);


///////////////////////  存檔相關  ////////////////////////}

///////////////////////  常用 ///////////////////////////{

  void Information(int jj) ;///印出資訊;
  double minimun(int ,int ,int&);

///////////////////////  常用  ///////////////////////////} 
}

/////////////////////////////  抓取機器人位置  //////////////////////////{


  void chatter1Callback1(const std_msgs::Float64::ConstPtr& info_1){
    ss =  info_1->data;
  }
  void chatter1Callback2(const std_msgs::Int32::ConstPtr& info_2){
    HH =  info_2->data;
  }
  void chatter1Callback3(const std_msgs::Int32::ConstPtr& info_3){
    linear1 =  info_3->data;
  }
  void chatter1Callback4(const std_msgs::Int32::ConstPtr& info_4){
    angular1 =  info_4->data;
  }

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
	    orientation_z1= -(1.5707+( 3.14159+orientation_z1)); //第四象限為90~180
    else
	    orientation_z1 = -(orientation_z1 -1.5707) ;  
  }
  
  // void Final_Goal(const geometry_msgs::PoseStamped::ConstPtr & F_goal)
  // {
  //     position_x1 = F_goal->pose.position.x;
  //     position_y1 = F_goal->pose.position.y;

  //     tf2::Quaternion s;
  //     tf2::convert(F_goal->pose.orientation, s);
  //     tf2::Matrix3x3(s).getRPY(roll_s, pitch_s, yaw_s);
      
  //     // 將 yaw 角度從弧度轉換為度
  //     double yaw_degrees = yaw_s * 180.0 / M_PI;

  //     // 進行座標系轉換
  //     if (yaw_degrees >= 0 && yaw_degrees <= 90) {
  //         orientation_z1 = 90 - yaw_degrees;  // 右上象限
  //     } else if (yaw_degrees > 90 && yaw_degrees <= 180) {
  //         orientation_z1 = 450 - yaw_degrees; // 左上象限
  //     } else if (yaw_degrees >= -180 && yaw_degrees < -90) {
  //         orientation_z1 = -270 - yaw_degrees; // 左下象限
  //     } else { // -90 <= yaw_degrees < 0
  //         orientation_z1 = -90 - yaw_degrees;  // 右下象限
  //     }

  //     // 確保結果在 -180 到 180 度範圍內
  //     orientation_z1 = fmod(orientation_z1 + 180, 360) - 180;

  //     // 將結果轉回弧度
  //     orientation_z1 = orientation_z1 * M_PI / 180.0;
  // }
  void old_Callback(const nav_msgs::Odometry::ConstPtr& msg){
    double x = msg->pose.pose.position.x;
    double y = msg->pose.pose.position.y;

    
  }
  void new_Callback(const nav_msgs::Odometry::ConstPtr& msg ){
    double x = msg->pose.pose.position.x;
    double y = msg->pose.pose.position.y;
    
    FILE *pfoutnew; 
      pfoutnew=fopen(save_new,"a");
			if(pfoutnew==NULL)
      {
			  printf("Fail to open file");
			  exit(1);
		  }  
      fprintf(pfoutnew,"%lf \t %lf \n", x,y);
      fclose(pfoutnew);
  }
  void lasercallback(const sensor_msgs::LaserScan::ConstPtr& scan)//抓取雷射數值,右半邊為90~160,前方為160~200,左方為200~270
  {
		// int k ;
    // for(int i=1;i<=360;i++) laser_temp[i]=scan->ranges[i];

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
		

	}

  // void amcl_Callback(const::move_base_msgs::MoveBaseActionFeedback::ConstPtr & amcl){
  //   int ll =0;
  //   amcl_position_x = amcl->feedback.base_position.pose.position.x;
  //   amcl_position_y = amcl->feedback.base_position.pose.position.y;
  //   tf2::Quaternion q;
  //   tf2::convert(amcl->feedback.base_position.pose.orientation,q);
	//   tf2::Matrix3x3(q).getRPY(roll,pitch,yaw);
  //   amcl_orientation_z =yaw;

	//   if( amcl_orientation_z <=-1.5707 && amcl_orientation_z>=-3.14159 )
      
  //      amcl_orientation_z= -(1.5707+( 3.14159+amcl_orientation_z)); //第四象限為90~180
     
  //   else
	//     amcl_orientation_z = -(amcl_orientation_z -1.5707) ;  
    
  // }
  void amcl_Callback(const::move_base_msgs::MoveBaseActionFeedback::ConstPtr & amcl)
  {
    int ll =0;
    amcl_position_x = amcl->feedback.base_position.pose.position.x;
    amcl_position_y = amcl->feedback.base_position.pose.position.y;
    tf2::Quaternion q;
    tf2::convert(amcl->feedback.base_position.pose.orientation,q);
	  tf2::Matrix3x3(q).getRPY(roll,pitch,yaw);
    amcl_orientation_z =yaw;

    // 座標系轉換
	  if( amcl_orientation_z <=-1.5707 && amcl_orientation_z>=-3.14159 )
      
       amcl_orientation_z= -( 3.14159+1.5707+amcl_orientation_z); //第四象限為90~180
     
    else
	    amcl_orientation_z = -(amcl_orientation_z -1.5707) ;  
    
	  
    
  }

  
  void Path_Callback(const ::nav_msgs::Path::ConstPtr & msg) //規劃路徑後產生的所有點,這裡取第10個目標作為區域目標
  { 
    double xx1, yy1;
    int i=0;
    std::vector<geometry_msgs::PoseStamped> data = msg->poses;
    for(std::vector<geometry_msgs::PoseStamped>::const_iterator it= data.begin(); it!= data.end(); ++it)
    {  
      position_x[i]=msg->poses[i].pose.position.x;
      position_y[i]=msg->poses[i].pose.position.y;
      
      tf2::Quaternion g;
  	  tf2::convert(msg->poses[i].pose.orientation,g);
	    tf2::Matrix3x3(g).getRPY(roll_g,pitch_g,yaw_g);  
      double yaw_degrees_g = yaw_g * 180.0 / M_PI;

      // // 新的轉換邏輯
      // if (yaw_degrees_g >= -180 && yaw_degrees_g < 0) {
      //     // 原本的左半平面 (-180 到 0) 轉換為新的下半平面 (180 到 0)
      //     orientation_z[i] = yaw_degrees_g + 180;
      // } else {
      //     // 原本的右半平面 (0 到 180) 轉換為新的上半平面 (0 到 -180)
      //     orientation_z[i] = yaw_degrees_g - 180;
      // }
      if (yaw_degrees_g > -180 && yaw_degrees_g < -90) {
          // 原本的左半平面 (-180 到 0) 轉換為新的下半平面 (180 到 0)
          orientation_z[i] = -(yaw_degrees_g + 270);
      } else {
          // 原本的右半平面 (0 到 180) 轉換為新的上半平面 (0 到 -180)
          orientation_z[i] = -(yaw_degrees_g - 90);
          
      }
      


      // 確保角度在 -180 到 180 度範圍內
      if (orientation_z[i] > 180) {
          orientation_z[i] -= 360;
      } else if (orientation_z[i] <= -180) {
          orientation_z[i] += 360;
      }


	    orientation_z[i] = orientation_z[i] * 3.14159 / 180 ;
      local_rms = sqrt(pow(amcl_position_x-position_x[i],2)+pow(amcl_position_y-position_y[i],2));
      if ( local_rms <= 1 - 0.1*fabs(error_orientation_z)/1.5707)
      {
       
        ik=i;
      }
      i++;
    } 
      if(xx1!=position_x[ik])
      {
      FILE *pfoutold; 
      pfoutold=fopen(position_target,"a");
			if(pfoutold==NULL)
      {
			  printf("Fail to open file");
			  exit(1);
		  }  
      for(int ii=10;ii<20;ii++)
        fprintf(pfoutold,"%lf \t %lf\t",position_x[ii],position_y[ii]);
        fprintf(pfoutold,"\n");
      fclose(pfoutold);
      }

    
      xx1 = position_x[ik];
      yy1 = position_y[ik];

    //if(sqrt(pow(amcl_position_x-position_x1,2)+pow(amcl_position_y-position_y1,2))<1.) 
      if(i<=20)
      {
          
        ik=i;
        position_x[i] = position_x1;
        position_y[i] = position_y1; 
        orientation_z[i] = orientation_z1;
      }
    
  } 

  ///////////////// Fuzzy系統:把感測器的讀值做正規化 ///////////////////{

  void coordinate_transform (int jj) //R0S世界徑度角有問題,此程式將其轉換 
  { 
    error_position_x =position_x[ik]-amcl_position_x;
    error_position_y =position_y[ik]-amcl_position_y;
    final_position_x =position_x1-amcl_position_x;
    final_position_y =position_y1-amcl_position_y;
    error_final_z = orientation_z1-amcl_orientation_z;
    rms=sqrt(pow(final_position_x,2)+pow(final_position_y,2));
    
  }
 
  void fuzzy_in(int jj)
{ 

    if (error_position_y==0) error_z = 0;
    // else error_z = atan2(error_position_x , error_position_y);
    else error_z =atan(abs(error_position_x/error_position_y));
    
    trans_angle_z = error_z*180/3.14159 ; 
    
    trans_new_amcl =amcl_orientation_z;

     
    //用X、Y分量差位置判別角度差    四個象限對應到的角度與向量軸有差異->>>atan()無法轉換時無法判別分子/母正負 手動調整角度,以符合實際.
    if ( error_position_x>0 && error_position_y >0)   trans_angle_z =  trans_angle_z;
    else if( error_position_x<0 && error_position_y >0)  trans_angle_z = -trans_angle_z;
    else if( error_position_x<0 && error_position_y <0)  trans_angle_z = -180+trans_angle_z;
    else if( error_position_x>0 && error_position_y <0)  trans_angle_z =  180-trans_angle_z; 	
    
   // 車頭方向 - 位置的角度差 = 車頭需要旋轉的角度 ,判別左旋轉或右旋轉較小的的角度
    if ((trans_angle_z > 90) && (trans_new_amcl*180/3.14158) <-90) {
      trans_degree = -360+abs(trans_angle_z)+abs(trans_new_amcl*180/3.14159) ;
        cout<<"!1111111111111111111111111111"<<endl;
    } 
    else if ((trans_angle_z <-90) && (trans_new_amcl*180/3.14158) >90) {
      trans_degree = 360-abs(trans_angle_z)-abs(trans_new_amcl*180/3.14159) ;
      cout<<"!2222222222222222222222222222222222"<<endl;
   
    }
    else trans_degree = (trans_angle_z - trans_new_amcl*180/3.14159);




      cout<<"trans_degree=   "<<  trans_degree<<endl;

      
     error_orientation_z = trans_degree*3.14159/180 ;   
    if(abs(error_orientation_z)>3.14159) 
    {
      cout<<"this is bigger than 3.14159 "<<endl;
      cout<<"befor transfer angular is ="<<error_orientation_z<<endl;
      if (error_orientation_z>0 && left_min>0.5) error_orientation_z = -6.28 + error_orientation_z;
      else if (error_orientation_z<0 && right_min>0.5) error_orientation_z = 6.28 + error_orientation_z;
      cout<<"after transfer angular is == "<<error_orientation_z<<endl;
    }        
    
    cout<<"error_z========================= "<<error_z<<endl;
    sum_error = sqrt(pow(position_x[ik]-amcl_position_x,2)+pow(position_y[ik]-amcl_position_y,2));

    Error_orientation_z[jj] = error_orientation_z;
    in[1] = abs(error_orientation_z)/3.14159;
    in[2] = abs(Error_orientation_z[jj]-Error_orientation_z[jj-1])/3.14159; 
    in[3] = sum_error/1.5; 
    cout<<" in[1]= "<< in[1]<<endl;

    cout<<" in[2]= "<< Error_orientation_z[jj]<<"\t"<<Error_orientation_z[jj-1]<<endl;

    cout<<" in[3]= "<< in[3]<<endl;


}

///////////////// Fuzzy系統:把感測器的讀值做正規化 ///////////////////}
void decideRotation( int &kkk){
  int k;
  int rotation_left = 100;
  int rotation_right = 100; 
  int js=315;

  left_min = minimun(195,269,k);

  right_min = minimun(90,165,k);

  straight_min = minimun(165,195,k);

  margin_laser[0] = 0.553;///202012
  margin_laser[5] = 0.5551;//-0.05是因為被遮住，避免誤會碰撞
  margin_laser[10] = 0.5615;
  margin_laser[15] = 0.5725;
  margin_laser[20] = 0.4746-0.05;
  margin_laser[25] = 0.4921-0.05;
  margin_laser[30] = 0.5150-0.05;
  margin_laser[35] = 0.4969-0.05;
  margin_laser[40] = 0.4434;
  margin_laser[45] = 0.4031;
  margin_laser[50] = 0.3720-0.05;
  margin_laser[55] = 0.3479-0.05;
  margin_laser[60] = 0.3291-0.05;
  margin_laser[65] = 0.3145-0.05;
  margin_laser[70] = 0.3033-0.05;
  margin_laser[75] = 0.2951-0.05;
  margin_laser[80] = 0.2894-0.05;
  margin_laser[85] = 0.2861-0.05;
  margin_laser[90] = 0.285;
  margin_laser[95] = 0.2861;
  margin_laser[100] = 0.2894;
  margin_laser[105] = 0.2951;
  margin_laser[110] = 0.3033;
  margin_laser[115] = 0.3145;
  margin_laser[120] = 0.264;
  margin_laser[125] = 0.2301;
  margin_laser[130] = 0.2054;
  margin_laser[135] = 0.1867;
  margin_laser[140] = 0.1723;
  margin_laser[145] = 0.1611;
  margin_laser[150] = 0.1524;
  margin_laser[155] = 0.1456;
  margin_laser[160] = 0.1405;
  margin_laser[165] = 0.1367;
  margin_laser[170] = 0.1340;
  margin_laser[175] = 0.1325;
  margin_laser[180] = 0.132;
  margin_laser[185] = 0.1325;
  margin_laser[190] = 0.1340;
  margin_laser[195] = 0.1367;
  margin_laser[200] = 0.1405;
  margin_laser[205] = 0.1456;
  margin_laser[210] = 0.1524;
  margin_laser[215] = 0.1611;
  margin_laser[220] = 0.1723;
  margin_laser[225] = 0.1867;
  margin_laser[230] = 0.2054;
  margin_laser[235] = 0.2301;
  margin_laser[240] = 0.264;
  margin_laser[245] = 0.3145;
  margin_laser[250] = 0.3033;
  margin_laser[255] = 0.2951;
  margin_laser[260] = 0.2894;
  margin_laser[265] = 0.2861;
  margin_laser[269] = 0.285;
  margin_laser[270] = 0.285;

  
  for(int lsr=90;lsr<=165;lsr=lsr+5){
    if(laser_temp[lsr] < margin_laser[lsr]  && laser_temp[lsr] >= 0.15){
      kkk=1;
      printf("HITTTTTTTTTTTTTTTTTTTTT\n");
      printf("限制：觸碰最小距離: %lf\t 角度：%d\n",margin_laser[lsr],lsr);
      printf("右側車子最小距離：%lf\n",right_min); 
    }
  }
  for(int lsr=195;lsr<=270;lsr=lsr+5){
    if( laser_temp[lsr]< margin_laser[lsr] && laser_temp[lsr] >= 0.15){
      kkk=1;
      printf("HITTTTTTTTTTTTTTTTTTTTT\n");
      printf("限制：觸碰最小距離: %lf\t 角度：%d\n",margin_laser[lsr],lsr);
      printf("左側車子最小距離：%lf\n",left_min);
    }
  }
  for(int lsr=165;lsr<=195;lsr=lsr+5){
    if(laser_temp[lsr]<margin_laser[lsr] && laser_temp[lsr] >= 0.15){
      kkk=1;
      printf("HITTTTTTTTTTTTTTTTTTTTT\n");
      printf("限制：觸碰最小距離: %lf\t 角度：%d\n",margin_laser[lsr],lsr);
      printf("前方車子最小距離：%lf\n",straight_min); 
    }
	}
}
  void Search_Target(int jj) 
  {  
    int kkk =0 ;
    double safe_distance = 0.3; 
    decideRotation(kkk);

    if(HH!=1)
    {

      intergral = 0;
      intergral_1 = 0;
    }
    if ( kkk!=1 )
    { 
      error = error_orientation_z;
      error_1 = in[3];
      derivative = in[2]; 
      if( jj%10==0 ){
        intergral+= error/1.; 
        intergral_1 += error_1/1.;
      }
      if(fabs(intergral_1)>30) intergral_1=0;

      if(fabs(intergral)>30) 
      {
        intergral=0;
      }
      derivative_1 = error_1 - lastError_1;
      turn = K[1]*error + K[2]*intergral + K[3]*derivative ;
      Tp =  abs( K[4]*error_1+ K[5]*intergral_1+K[6]*derivative_1);


      if (abs(error)<0.1745) {   //5.7degree 內直線行走 0.175 
        intergral=0;
        intergral_1=0;
        printf("直走\n");
        status=1;
  	  } 
      else if ( (error>0.1745) ){ //一般情況下正為右旋轉，但跨過180度時則相反  || (error<-0.05 &&  angle_turn == 1)
        printf("右轉\n");
        lastError=error;
        lastError_1=error_1;	
        status = 3; 
      }
      else if ( (error<-0.1745)  ){ //一般情況下負為左旋轉，但跨過180度時則相反 //|| (error>0.05 &&  angle_turn == 1)
        printf("左轉\n");
        lastError=error; 
        lastError_1=error_1;
        status=2;
      } 
      out_y[1] = Tp+turn;          
      out_y[2] = Tp-turn; 
      cout<<" out_y[1]= "<< out_y[1]<<endl;
      cout<<" out_y[2]= "<< out_y[2]<<endl;
      speed_limit();
      v1=out_y[1]*0.03*1;
      v2=out_y[2]*0.03*1;

      cout<<"v1= "<<v1<<endl;
      cout<<"v2= "<<v2<<endl;
      msg.linear.x = ((v1+v2)/2);
      msg.angular.z =((v2-v1)/dis);

     if(msg.linear.x >0.6) msg.linear.x =0.6;
     if (msg.angular.z >0.85 ) msg.angular.z=0.85;
     else if (msg.angular.z < -0.85)
      msg.angular.z= -0.85;
	    pub.publish(msg);
    }    
    
    else 
    { 

      Tp=0;  
      intergral=0;

      if(straight_min <= safe_distance )
      {
      
         if (status==2 )
          {
            msg.angular.z=0.5;
            msg.linear.x=0;
            pub.publish(msg); 
            printf("\n距離過近 向左旋轉\n");
          }

          else if(status ==3)
          {
            msg.angular.z = -0.5;
            msg.linear.x=0;
            pub.publish(msg); 
            printf("\n距離過近 向右旋轉\n");
          }        
      }

      else if (left_min < safe_distance && right_min < safe_distance)
      {
        
        if (status==2 )
        {
          msg.angular.z = 0.2;
          msg.linear.x = 0.3;
          pub.publish(msg); 
          printf("\n距離過近 向左前方旋轉\n");
        }

        else 
        {
          msg.angular.z = -0.2;
          msg.linear.x = 0.3;
          pub.publish(msg); 
          printf("\n距離過近 向右前方旋轉\n");
        }        
    
      }
      else if (left_min <= safe_distance)
      {
        if(straight_min > safe_distance)
        { 
          msg.angular.z=-0.5;
          msg.linear.x= 0.;
          pub.publish(msg);
          printf("\n距離過近 向右前方旋轉\n");
        }
      }
      else if (right_min <= safe_distance)
      {
        if(straight_min > safe_distance)
        { 
          msg.angular.z= 0.5;
          msg.linear.x= 0.3;
          msg.linear.x= 0.;
          pub.publish(msg);
          printf("\n距離過近 向左前方旋轉\n");
        }
      }
      
    }
    
    linear=msg.linear.x ;  
    angular=msg.angular.z;   
    a=10;
  }
  void speed_limit()
  {
	/// 使 Robot 維持在最高速及最低速
	  if ( out_y[1] >= (  left_wheel_speed) )
		  out_y[1] =  left_wheel_speed ;
	  if ( out_y[2] >= (right_wheel_speed) )
		  out_y[2] = right_wheel_speed ;
	  if ( out_y[1] <= ( - left_wheel_speed) )
		  out_y[1] =  - left_wheel_speed ;
	  if ( out_y[2] <= ( - right_wheel_speed) )
	  	out_y[2] =  - right_wheel_speed ;	 
  }
  void Final_stop(int jj){      
    /*final_z = error_final_z ; 
    error = final_z; 
     if( jj%10==0 ){
        intergral+= error/1; 
      }
    
     derivative = (error - lastError);
     turn = K[1]*error + K[2]*intergral + K[3]*derivative ;

   if (abs(final_z)<0.175 ) 
    { 
      a=0;
      ROS_INFO("In the end!!!\n");
      cout<<"angular error = "<<final_z<<endl;
      msg.linear.x =0.;
      msg.angular.z=0.;
      pub.publish(msg);
      ros::shutdown();
    }
    else if ( final_z>0.175 && a!=0)
    {
      lastError=error;     
      status=11;
      a=1;
      printf("\nLeft rotation!!!n");
    }
    else if ( final_z<-0.175  && a!=0)
    {
      lastError=error; 
      status=11;
      a=1;
      printf("\nRight rotation!!!\n");            
    }
      out_y[1] = Tp+turn;          
      out_y[2] = Tp-turn;

      v1=out_y[1]*0.098/2.5;
      v2=out_y[2]*0.098/2.5;
      msg.linear.x = 0.;
      msg.angular.z =((v2-v1)/dis);

      if (msg.angular.z>0.1) msg.angular.z=0.1 ;
      else if(msg.angular.z<-0.1) msg.angular.z=-0.1 ; 
*/
      linear = msg.linear.x ;
      angular = msg.angular.z;
      msg.linear.x = 0.;
      msg.angular.z =0.;
	    pub.publish(msg);
  }
    
  void publisher_info(int jj ) //印出訊息並傳遞數值給其他程式從pub1~pub10;
  {        
   
    std_msgs::Float64 info2,info3;
    std_msgs::Int32  info1;
    
    angle_deviate=trans_degree;
    info1.data = status;  //車子與目標的角度差
    info2.data = error_orientation_z ;             //經過轉換後的機器人Z軸角度
  
    chatter_pub1.publish(info1); 
    chatter_pub2.publish(info2);
   
  }
 
  void local_target(int jj)//紀錄
  { 
    amcl_x[jj]=position_x[ik];
    amcl_y[jj]=position_y[ik];
    if (amcl_x[jj]!=amcl_x[jj-1] ||amcl_y[jj]!=amcl_y[jj-1] )
    {  
      FILE *pfout1; //all path ;
     // printf("\n\nread file\n");
      pfout1=fopen(save_local_target,"a");
			if(pfout1==NULL)
      {
			  printf("Fail to open file");
			  exit(1);
		  }  
      fprintf(pfout1,"%lf \t %lf \n", position_x[ik],position_y[ik]);
      fclose(pfout1);


        FILE *pfout100; //all path ;
     // printf("\n\nread file\n");
    }
  }
  void stop(){
	  if(counts==max_step)
	  {
	    msg.linear.x =0;
	    msg.angular.z=0;
	    pub.publish(msg);
    }
  }



////////////////////// Fuzzy系統:讀取後件部 //////////////////////////{
void Fir_str(double *in, int _rule)     
{
//這裡的ber，等於main函式內的ber，又ber的值為popsize的值。
//這裡的 _rule，等於設定變數的_rule_number的值。
//這裡的in，代表輸入的變數的數量

int j,k ;

 for(k=1; k<=_rule; k++)  //_rule值=_rule_number=10
 {
   for (j=1; j<=_out_varl; j++)
   {
     _Rule[k].con[j] = fuzzy_real[k*_mem_length-_out_varl+j] ; // 將左右輪的後件部值做"轉存"
  
   } 	             
  _Rule[k].In_mem_fir(in,k) ; // 計算激發量

       
  }
}

void fuzzy(int _rule ) 
{
int i , j;
double den[_out_varl+1] ,   //分母項
       num[_out_varl+1] ;   //分子項

 for (j=1; j<=_out_varl; j++)
 {
  den[j] = 0. ;  //初始化歸零
  num[j] = 0. ;  //初始化歸零
   
   for (i=1; i<=_rule; i++)
   {
   num[j] = num[j] + _Rule[i].in_mu_1 * _Rule[i].con[j];  //num為sigma的寫法
   den[j] = den[j] + _Rule[i].in_mu_1 ;   //den為sigma的寫法

   }

   if ( fabs(den[j]) < 1E-8 )
   K[j] = 0 ;     //如果den很小時
   else
   K[j] = num[j]/den[j] ;  //權重式平均法
 }
 
//   for (int kk=1; kk<=_out_varl; kk++) printf("den[%d]=%lf \t num=%lf\n",kk,den[kk],num[kk]);
}
//////////////////////// 解模糊 ///////////////////////////}

//////////////////////  速度限制器  //////////////////////////{

//////////////////////  速度限制器  //////////////////////////}



/////////////////////////////  開啟檔案  //////////////////////////}
void test_open()
{

			for(int ssss=1; ssss<=_in_varl; ssss++)
			{
					min_m[ssss]=1.0;  //為了找最小值，所以初始的最小值令為1，即為"最大"的最小值。
					max_m[ssss]=0.0;  //為了找最大值，所以初始的最大值令為0，即為"最小"的最大值。		
			////-1~1之間的中心點範圍 
			//max_m[ber][ssss]=0.0;  //為了找最大值，所以初始的最大值令為-1，即為"最小"的最大值。        
					
			}
			FILE *fnoise1,*fnoise0;  
			//printf("read file\n");		
			if((fnoise0=fopen(load_data_clu,"r"))==NULL)
			{
				printf("Nothing\n");
				exit(1);
			}
			fscanf(fnoise0,"%d", &_in_clu);
			fclose(fnoise0);
			if((fnoise1=fopen(load_data_FC,"r"))==NULL)
			{
				printf("Nothing1111111\n");
				exit(1);
			}
			for(int i=1;i<=_mem_length*_in_clu;i++)
			{
				fscanf(fnoise1,"%lf \n", &fuzzy_real[i]);
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

/////////////////////////////  左右輪輪速的存檔  //////////////////////////}

 void Information(int jj)
  {  
    cout<<"step= "<<jj<<endl;
    printf("\n規劃路徑總數量: %d個點 , status=%d Distance=%f\n",ik,status,sqrt(pow(amcl_position_x-position_x[ik],2)+pow(amcl_position_y-position_y[ik],2)));
    printf("x誤差=%lf \t     y誤差=%lf \t  區域誤差＝%lf\t rms=%lf\n",error_position_x,error_position_y,sqrt(pow(error_position_x,2)+pow(error_position_y,2)),rms);
    printf("left_min=%f \t right_min=%f \t straight_min=%f\t \n",left_min,right_min,straight_min);  
    printf("\n======================================================\n\n");
    printf("機器人位置是= (x,%.7lf),(y,%.7lf),(z,%.7lf)\n",amcl_position_x,amcl_position_y,amcl_orientation_z);
    printf("區域目標是=   (x,%.7lf),(y,%.7lf),(z,%.7lf)\n",position_x[ik],position_y[ik], orientation_z[ik]);
    printf("終點目標是=   (x,%.7lf),(y,%.7lf),(z,%.7lf)\n",position_x1,position_y1,orientation_z1);
    printf("\n======================================================\n\n");
    printf("機器人新座標角= %lf rad \t %lf degree\n",trans_new_amcl,trans_new_amcl*180/3.14159);
    printf("目標新座標角=   %lf rad \t %lf degree\n",trans_angle_z*3.14159/180,trans_angle_z);
    printf("新座標角度差=   %lf rad \t %lf degree\n",error_orientation_z ,error_orientation_z*180/3.14159);
    printf("終點角度差=     %lf rad \t %lf degree\n",error_final_z,error_final_z*180/3.14159);
    printf("\n======================================================\n\n");
    printf("\nerror=%lf \t intergral=%lf \t derivation=%lf \t    turn=%lf\n\n右輪速度=%lf \t        左輪速度=%lf \t\n\n",error,intergral,derivative,turn,v2,v1);
    printf("\nerror_1=%lf \t intergral_1=%lf \t derivation_1=%lf \t \n\n",error_1,intergral_1,derivative,_1);
   
    for (int ilk=1 ;ilk<=3;ilk++) cout<<"in["<<ilk<<"]= "<<in[ilk]<<" \t";
    cout<<"\n\n"; 


    for (int i=0;i<3;i++){
      printf("K[%d]=%lf \t",i+1,K[i+1]);
    }
    cout<<"\n";

    for (int i=3;i<6;i++){
      printf("K[%d]=%lf \t",i+1,K[i+1]);
    }
    printf("\n\n");

    if(HH=1) printf("\n\n linear=%lf \t angular=%lf\n\n", linear, angular);
    else  printf("\n\n linear=%lf \t angular=%lf\n\n", linear1, angular1);
  }

  double minimun(int i, int j ,int &k)
  {  
    double laser_min=100;
    for ( k=i ;k<=j;k++){        
      if (laser_min>laser_temp[k])
      { 
			  laser_min=laser_temp[k];       
			}    
    }
	
    return laser_min;    
  }

};
//----------------------------------------  main區 ------------------------------------------------//{
 
int main(int argc,char ** argv)
{ 
  auto start = chrono::high_resolution_clock::now();
  ros::init(argc,argv,"amcl_pose_sub_pub_1");
  amcl_pose_sub_pub_1 ctrl;
	while(counts<max_step && ros::ok())
	{
    if(position_x1!=0)
    {
		  counts++;
    }
    ctrl.test_open(); //to get max and min
    ctrl.coordinate_transform(counts);
    ctrl.fuzzy_in(counts) ;  //輸入是雷射數值，只是調整fuzzy的input比例(做正規化)			
    ctrl.Fir_str(in , _rule_number) ; //讀取後件部計算每條rule機發量
    ctrl.fuzzy(_rule_number); //解模糊產生out_y
    ctrl.publisher_info(counts);
		if(counts<5){
		  ctrl.stop(); //機器人前5個counts不動作
		}
    if(rms>0.1 && (a!=0 ||a!=1)){
      ctrl.Search_Target(counts);   
    }
    else if(rms<0.1 && counts>10) 
    {
     ctrl.Final_stop(counts);
    }
    
    
    ctrl.speed_limit() ; // 速度限制器
    ctrl.Information(counts); //印資料	
    ctrl.local_target(counts);
		ros::spinOnce();
		ros::Duration(0.01).sleep(); //訊息傳送間隔0.01秒
	}
  return 0;
}

//----------------------------------------  main區 ------------------------------------------------//}



