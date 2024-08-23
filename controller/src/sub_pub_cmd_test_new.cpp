#include <ctime> 
#include <math.h>
#include <vector>
#include <iostream>   

#include <ros/ros.h>          // 加入ROS公用程序
#include <std_msgs/String.h> 
#include <std_msgs/Float64.h> 
#include <std_msgs/Int32.h> 
#include <tf/transform_broadcaster.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h> 
#include <move_base_msgs/MoveBaseActionFeedback.h>
#include <nav_msgs/Path.h>
#include "sensor_msgs/LaserScan.h"

///#include <nav_msgs/OccupancyGrid.h>
using namespace std;
#define MIN(a,b) (a<b?a:b)
/////////////////////////////////////////////

#define save_data_path1    "position_random/real-map2s/position_all_fuzzy_pid_NEW2.txt"  //整條路徑
#define save_along_wall    "position_random/real-map2s/position_along_wall_fuzzy_pid_NEW2.txt" //沿牆
#define save_search_target "position_random/real-map2s/posititon_search_target_fuzzy_pid_NEW2.txt" //尋標
#define save_distance      "position_random/real-map2s/distance-NEW.txt"  //左/右/前測距離
#define save_velocity      "position_random/real-map2s/velocity-NEW.txt" 
///1//////////////////////////////////////////
#define max_step 40000
double fff=0;
int p[max_step+1]={0},counts=0,last[max_step],ss=111;
int HH;
int height,width;
double v_S,v_A;
double error_orientation_z;
double target_orientation_z, target_position_x, target_position_y;
double amcl_orientation_x,amcl_orientation_y,amcl_orientation_z,amcl_position_z,amcl_position_y,amcl_position_x;
double amcl_x[max_step],amcl_y[max_step];
double left_min,right_min,straight_min;
double decision_left,decision_right; 
double laser_temp[361];
float laser_temp_scan[897];
double vel_s,vel_a,angular_s,angular_a;
double rms,rms_x, rms_y;
int status=1000;
double integral,turn;
double roll,pitch,yaw;
double margin_laser[361];
double position_x[max_step],position_y[max_step];
double error_x,error_y,directions;
int ik ;
double x_,y_;
geometry_msgs::Twist msg;

bool recieved_message = false;
class cmd_sub_pub{
private:
   ros::NodeHandle n;
   ros::Publisher pub;
   ros::Subscriber sub_1;
   ros::Subscriber sub_2;
   ros::Subscriber sub_goal;
   ros::Subscriber sub_laser;
   ros::Subscriber sub_amcl;
   ros::Subscriber sub_info1;
   ros::Subscriber sub_info2;
   ros::Subscriber sub_path ;
   ros::Subscriber sub_obstacle_positions;
   ros::Subscriber sub_cmdvel;
   ros::Subscriber sub_odom0;
   ros::Subscriber sub_decision;
   ros::Publisher x1_pub;
   ros::Publisher y1_pub;
   ros::Publisher x2_pub;
   ros::Publisher y2_pub;
   ros::Publisher pub_1;
   ros::Publisher pub_2;

public:
   cmd_sub_pub()
   {
      pub = n.advertise<geometry_msgs::Twist>("/cmd_vel",3000);
      x1_pub=n.advertise<std_msgs::Float64>("data_x1",3000);
      y1_pub=n.advertise<std_msgs::Float64>("data_y1",3000);
      x2_pub=n.advertise<std_msgs::Float64>("data_x2",3000);
      y2_pub=n.advertise<std_msgs::Float64>("data_y2",3000);
      pub_1=n.advertise<std_msgs::Float64>("info_1",3000);
      pub_2=n.advertise<std_msgs::Int32>("info_2",3000);
      
      //pub = n.advertise<geometry_msgs::Twist>("/RosAria/cmd_velx",3000);
      sub_laser=n.subscribe("/scan",3000, &cmd_sub_pub::callback,this);
      //sub_laser=n.subscribe("/scan",3000, &cmd_sub_pub::lasercallback,this);
      sub_1 = n.subscribe("/cmd_vel_1",3000, &cmd_sub_pub::speed_Search, this);
      sub_2 = n.subscribe("/cmd_vel_2",3000, &cmd_sub_pub::speed_Along_wall, this);
      sub_goal = n.subscribe("/move_base_simple/goal",3000, &cmd_sub_pub::Search_Target, this);
      sub_info1 =n.subscribe("chatter1",3000, &cmd_sub_pub::chatter1Callback,this);
      sub_info2 =n.subscribe("chatter2",3000, &cmd_sub_pub::chatter2Callback,this);
      sub_amcl = n.subscribe("/move_base/feedback", 3000, &cmd_sub_pub::amcl_Callback,this);
    
     
      double minimun(int, int , int );
      void pathCallback1(const geometry_msgs::Twist& vel);
      void pathCallback2(const geometry_msgs::Twist& vel);
      void lasercallback(const sensor_msgs::LaserScan::ConstPtr& scan);
      void decision_path(double, double, double);
      void Search_Target(const geometry_msgs::PoseStamped::ConstPtr& goal);
      void Search_amcl(const nav_msgs::Odometry::ConstPtr& amcl);
      void chatter1Callback(const std_msgs::String::ConstPtr& info);
      void amcl_Callback(const::move_base_msgs::MoveBaseActionFeedback::ConstPtr & amcl);
      void publisher_info(int jj ) ;
      void Calcuate_positions();

   }

  
   // void lasercallback(const sensor_msgs::LaserScan::ConstPtr& scan)
   // {
	// 	int k ,g;
   //    for(int i=1;i<=360;i++)
   //    {
	// 		laser_temp[i]=scan->ranges[i]; ///202009 180 to 270
   //    }  
    
   //       left_min=minimun(210,315,k);

   //       right_min=minimun(45,150,k);

	// 	   straight_min=minimun(150,210,k);
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
      straight_min=minimun(165,195,k);


   }
   
   void decision_path(double i, double j, double k ,int jj)
   {  
      int js=315;
      // margin_laser[0] = 0.553;///202012
      // margin_laser[5] = 0.5551;//-0.05是因為被遮住，避免誤會碰撞
      // margin_laser[10] = 0.5615;
      // margin_laser[15] = 0.5725;
      // margin_laser[20] = 0.4746-0.05;
      // margin_laser[25] = 0.4921-0.05;
      // margin_laser[30] = 0.5150-0.05;
      // margin_laser[35] = 0.4969-0.05;
      // margin_laser[40] = 0.4434;
      // margin_laser[45] = 0.4031;
      // margin_laser[50] = 0.3720-0.05;
      // margin_laser[55] = 0.3479-0.05;
      // margin_laser[60] = 0.3291-0.05;
      // margin_laser[65] = 0.3145-0.05;
      // margin_laser[70] = 0.3033-0.05;
      // margin_laser[75] = 0.2951-0.05;
      // margin_laser[80] = 0.2894-0.05;
      // margin_laser[85] = 0.2861-0.05;
      // margin_laser[90] = 0.285;
      // margin_laser[95] = 0.2861;
      // margin_laser[100] = 0.2894;
      // margin_laser[105] = 0.2951;
      // margin_laser[110] = 0.3033;
      // margin_laser[115] = 0.3145;
      // margin_laser[120] = 0.264;
      // margin_laser[125] = 0.2301;
      // margin_laser[130] = 0.2054;
      // margin_laser[135] = 0.1867;
      // margin_laser[140] = 0.1723;
      // margin_laser[145] = 0.1611;
      // margin_laser[150] = 0.1524;
      // margin_laser[155] = 0.1456;
      // margin_laser[160] = 0.1405;
      // margin_laser[165] = 0.1367;
      // margin_laser[170] = 0.1340; // wei change for avoid
      // margin_laser[175] = 0.1325;
      // // margin_laser[180] = 0.132;
      // margin_laser[180] = 0.319627;
      // margin_laser[185] = 0.1325;
      // margin_laser[190] = 0.1340;  // wei change for avoid
      // margin_laser[195] = 0.1367;
      // margin_laser[200] = 0.1405;
      // margin_laser[205] = 0.1456;
      // margin_laser[210] = 0.1524;
      // margin_laser[215] = 0.1611;
      // margin_laser[220] = 0.1723;
      // margin_laser[225] = 0.1867;
      // margin_laser[230] = 0.2054;
      // margin_laser[235] = 0.2301;
      // margin_laser[240] = 0.264;
      // margin_laser[245] = 0.3145;
      // margin_laser[250] = 0.3033;
      // margin_laser[255] = 0.2951;
      // margin_laser[260] = 0.2894;
      // margin_laser[265] = 0.2861;
      // margin_laser[269] = 0.285;
      // margin_laser[270] = 0.285;
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
      margin_laser[90] = 0.3060;
      margin_laser[95] = 0.3136;
      margin_laser[100] = 0.3111;
      margin_laser[105] = 0.3267;
      margin_laser[110] = 0.3508;
      margin_laser[115] = 0.3609;
      margin_laser[120] = 0.3623;
      margin_laser[125] = 0.3702;
      margin_laser[130] = 0.3603;
      margin_laser[135] = 0.3776;
      margin_laser[140] = 0.4526;   // lidar disable start
      margin_laser[145] = 0.4078;
      margin_laser[150] = 0.3869;
      margin_laser[155] = 0.3836;
      margin_laser[160] = 0.3516;
      margin_laser[165] = 0.3423;
      margin_laser[170] = 0.3319; 
      margin_laser[175] = 0.3413;
      margin_laser[180] = 0.3420;
      margin_laser[185] = 0.3389;
      margin_laser[190] = 0.3337;  
      margin_laser[195] = 0.3343;
      margin_laser[200] = 0.3466;
      margin_laser[205] = 0.3712;
      margin_laser[210] = 0.4001;
      margin_laser[215] = 0.3932;
      margin_laser[220] = 0.4472;
      margin_laser[225] = 0.4534;  // lidar disable end
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

      p[jj]=0;
      int ln=0;
      amcl_x[jj]=amcl_position_x;
      amcl_y[jj]=amcl_position_y;
      rms_x=amcl_x[jj]-target_position_x;
      rms_y=amcl_y[jj]-target_position_y;
      rms= sqrt(pow(rms_x,2)+pow(rms_y,2));
      for(int lsr=90;lsr<=180;lsr=lsr+5){
         // if(laser_temp[lsr] < margin_laser[lsr]  && laser_temp[lsr] >= 0.15){  wei change 
         if(laser_temp[lsr] < margin_laser[lsr]){
            p[jj] =1; ////尋標
            ln=1; ////離牆太近
            printf("HITTTTTTTTTTTTTTTTTTTTT\n");
            printf("限制：觸碰最小距離: %lf\t 角度：%d\n",margin_laser[lsr],lsr);
            printf("右側車子最小距離：%lf\n",right_min); 
         }
      }
      for(int lsr=180;lsr<=270;lsr=lsr+5){
         // if( laser_temp[lsr]< margin_laser[lsr] && laser_temp[lsr] >= 0.15){ wei change
         if( laser_temp[lsr]< margin_laser[lsr]){
             p[jj] =1; ////尋標
            ln=1; ////離牆太近
            printf("HITTTTTTTTTTTTTTTTTTTTT\n");
            printf("限制：觸碰最小距離: %lf\t 角度：%d\n",margin_laser[lsr],lsr);
            printf("左側車子最小距離：%lf\n",left_min);
         }
      }
      for(int lsr=165;lsr<=195;lsr=lsr+5){
         // if(laser_temp[lsr]<margin_laser[lsr] && laser_temp[lsr] >= 0.15){  wei change
         if(laser_temp[lsr]<margin_laser[lsr]){
             p[jj] =1; ////尋標
            ln=1; ////離牆太近
            printf("HITTTTTTTTTTTTTTTTTTTTT\n");
            printf("限制：觸碰最小距離: %lf\t 角度：%d\n",margin_laser[lsr],lsr);
            printf("前方車子最小距離：%lf\n",straight_min); 
         }
      }
      

      
      if (status==1 && (left_min<0.5 || right_min<0.5) && ln!=1) { ///朝前方移動時候,考慮兩側是否有障礙物
     
         if( left_min<=0.5 )
         {
            printf("左前方區域有障礙物，左沿牆 \t ststus = %d\n",status);
            cout<<"11111111111111111111"<<endl;
         }
         else if ( right_min<=0.5 )
         {
            printf("右前方區域有障礙物，右沿牆 \t ststus = %d\n",status);
            cout<<"22222222222222222222222"<<endl;
         }
         p[jj]=2;
      }
      
     
      else if(  (status ==2 && left_min<=0.5) && ln!=1) { ///朝前方移動時候,考慮兩側是否有障礙物
      
         printf("左前方影障礙,左沿牆 \t status = %d\n",status);
         p[jj]=2;
       
      }
      else if( (status ==3 && right_min<=0.5) && ln!=1) { ///朝前方移動時候,考慮兩側是否有障礙物


         printf("右前方有障礙物,右沿牆 \t status = %d\n",status);
         p[jj]=2;
       
      }
      else{ 
         cout<<"staus= "<<status<<endl;
         p[jj]=1;
      
      } 
     if( (straight_min < 0.6 && straight_min > 0.3)  && p[jj]==1 && ln!=1 ) 
      
      {
         vel_s = vel_s*0.5;
         angular_s = angular_s*0.5;
         cout<<"========60％％％％speed========="<<endl;
      }
      if (p[jj]==1)
      {     
         ss=1;
        
         msg.linear.x=vel_s*0.6;   // wei change from 0.8 to ?
         msg.angular.z=angular_s*0.6; // wei change from 0.8 to ?
         pub.publish(msg);
         ROS_INFO("Search Target: cmd_vel_1" );
         publisher_info(jj);
         printf("left_min=%f \t right_min=%f \t straight_min=%f\t \n",left_min,right_min,straight_min); 
         printf("\n(linear,angular):(%lf ,%lf) \n",msg.linear.x,msg.angular.z);
         }
      else if (p[jj]==2){
          
         ss=0;
         msg.linear.x=vel_a*0.8; // wei change from 0.8 to ?
         msg.angular.z=angular_a*1.5; // wei change from 0.9 to ?
         integral =0;
         pub.publish(msg);
         ROS_INFO("Along_wall : cmd_vel_2");
         printf("left_min=%f \t right_min=%f \t straight_min=%f\t \n",left_min,right_min,straight_min);  
         printf("\n(linear,angular)(%lf ,%lf) \n",msg.linear.x,msg.angular.z);  
         }

   }

   void publisher_info(int jj ) //印出訊息並傳遞數值給其他程式從pub1~pub10;
   {        

    std_msgs::Float64 info_1;
    std_msgs:: Int32  info_2;

    info_1.data = ss;
    info_2.data = HH;
    pub_1.publish(info_1); 
    pub_2.publish(info_2); 
  }
 
  double minimun(int i, int j ,int &k)
   {  
      double laser_min=100;
      for ( k=i ;k<j;k++){
         
         if (laser_min>laser_temp[k])
         { 
			   laser_min=laser_temp[k];
			}
      }
	  return laser_min;
   }

	void stop()
   {
			 
			msg.linear.x =0;
			msg.angular.z=0;
			pub.publish(msg);
         ROS_INFO("Navigation is the  end!!!!\n");
			ros::shutdown();	
	
	} 
   void Search_Target(const geometry_msgs::PoseStamped::ConstPtr& goal)
   {
     int ik=10;
      target_position_x=goal->pose.position.x;
      target_position_y=goal->pose.position.y;
      tf2::Quaternion s;
  	   tf2::convert(goal->pose.orientation,s);
	   tf2::Matrix3x3(s).getRPY(roll,pitch,yaw);
	   target_orientation_z=yaw;
     
   }
  
   void Path_Callback(const ::nav_msgs::Path::ConstPtr & msg) //規劃路徑後產生的所有點,這裡取第10個目標作為區域目標
   { 
      int i=0;
      std::vector<geometry_msgs::PoseStamped> data = msg->poses;
      for(std::vector<geometry_msgs::PoseStamped>::const_iterator it= data.begin(); it!= data.end(); ++it)
      {  
         position_x[i]=msg->poses[i].pose.position.x;
         position_y[i]=msg->poses[i].pose.position.y;
         // orientation_z[i]=msg->poses[i].pose.orientation.z;
         i++;       
      }  
   
       ik=10;
   } 

   
   void chatter1Callback(const std_msgs::Int32::ConstPtr& info1){
         status =  info1->data;
   }
    void chatter2Callback(const std_msgs::Float64::ConstPtr& info2){
         error_orientation_z =  info2->data;
   }


   void speed_Search(const geometry_msgs::Twist::ConstPtr& vel1){
      vel_s=vel1->linear.x;
      angular_s=vel1->angular.z;
      recieved_message =true; 
   }
   void speed_Along_wall(const geometry_msgs::Twist::ConstPtr& vel2){
      vel_a=vel2->linear.x;
      angular_a=vel2->angular.z;
   }
   void amcl_Callback(const::move_base_msgs::MoveBaseActionFeedback::ConstPtr & amcl){
      //double x_,y_;
      amcl_position_x= amcl->feedback.base_position.pose.position.x;
      amcl_position_y = amcl->feedback.base_position.pose.position.y;
      amcl_position_z = amcl->feedback.base_position.pose.position.z;
      amcl_orientation_x = amcl->feedback.base_position.pose.orientation.x;
      amcl_orientation_y = amcl->feedback.base_position.pose.orientation.y;
      amcl_orientation_z = amcl->feedback.base_position.pose.orientation.z; 
           // printf("\n\nread file\n");
      for(int i=1;i<=600;i++){
      if (sqrt(pow(position_x[i]-amcl_position_x,2)+pow(position_y[i]-amcl_position_y,2))>0.1 && sqrt(pow(position_x[i]-amcl_position_x,2)+pow(position_y[i]-amcl_position_y,2))<=0.15)
      {
        x_=position_x[i];
        y_=position_y[i];
        //ik=i;
      }
       
      //ik++;

    }
 
      
   }
   void save_info( int jj){
      if(amcl_x[jj]!=0. && amcl_y[jj]!=0.)
      {
         FILE *pfout1; //all path ;
         printf("\nread file\n");
         pfout1=fopen(save_data_path1,"a");
            if(pfout1==NULL){
               printf("Fail to open file");
               exit(1);
            }
         fprintf(pfout1,"%lf \t %lf\n",amcl_x[jj],amcl_y[jj]);
         fclose(pfout1);
      }

      if (right_min<=0.3 || left_min<=0.3 ||straight_min<=0.3){
         if (amcl_x[jj]!=amcl_x[jj-1]){   
   
            std_msgs::Float64 data_x2,data_y2; //outpt x2,y2 locate's information;
            data_x2.data=amcl_x[jj];
            data_y2.data=amcl_y[jj];
            x2_pub.publish(data_x2);
            y2_pub.publish(data_y2);
         } 
      }
      else if(p[jj]==1){
         if (amcl_x[jj]!=amcl_x[jj-1]){   
           
            FILE *pfout2; //all path ;
            printf("\nread file\n");
            pfout2=fopen(save_search_target,"a");
               if(pfout2==NULL){
                  printf("Fail to open file");
                  exit(1);
               }
            fprintf(pfout2,"%lf \t %lf\n",amcl_x[jj],amcl_y[jj]);
            fclose(pfout2);
            

            std_msgs::Float64 data_x1,data_y1;  //outpt x1,y1 locate's information;
            data_x1.data=amcl_x[jj];
            data_y1.data=amcl_y[jj];
            x1_pub.publish(data_x1);
            y1_pub.publish(data_y1);
         }
      }
      else if (p[jj]==2){
         if (amcl_x[jj]!=amcl_x[jj-1]) {  
            

            FILE *pfout3; //all path ;
            printf("\nread file\n");
            pfout3=fopen(save_along_wall,"a");
               if(pfout3==NULL){
                  printf("Fail to open file");
                  exit(1);
               }
            fprintf(pfout3,"%lf \t %lf\n",amcl_x[jj],amcl_y[jj]);
            fclose(pfout3);
            
            std_msgs::Float64 data_x2,data_y2; //outpt x2,y2 locate's information;
            data_x2.data=amcl_x[jj];
            data_y2.data=amcl_y[jj];
            x2_pub.publish(data_x2);
            y2_pub.publish(data_y2);
         } 
      }  
         
         if(jj>5)
            fff += sqrt(pow(amcl_x[jj]-amcl_x[jj-1],2)+pow(amcl_y[jj]-amcl_y[jj-1],2));
      
   
    }
   } ;
int main(int argc,char ** argv)
{
   ros::init(argc, argv, "cmd_sub_pub"); 
	
   cmd_sub_pub obj;
	//ros::Rate a(100);  
	
	while(counts<max_step && ros::ok())
	{ 
      if(target_position_x!=0)
		{
         counts++;
      }
      cout<<"~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~"<<endl<<endl;
		printf("counts:%d \n",counts);
      
      obj.decision_path(right_min, left_min,straight_min,counts);
      obj.publisher_info(counts);
      obj.save_info(counts);
      if ( msg.linear.x ==0 & msg.angular.z==0 && counts>100 ){
         cout<<"in the end "<<endl;
          obj.stop();
      }
       
		ros::spinOnce();
      ros::Duration(0.01).sleep();
	}
      FILE *pfout4; //all path ;
      printf("\nread file\n");
      pfout4=fopen(save_distance,"a");
      if(pfout4==NULL){
         printf("Fail to open file");
         exit(1);
      }  
      fprintf(pfout4,"%lf \n",fff);
      fclose(pfout4);

      FILE *pfoutv; //all path ;
      printf("\nread file\n");
      pfoutv=fopen(save_velocity,"a");
      if(pfoutv==NULL){
         printf("Fail to open file");
         exit(1);
      }  
      fprintf(pfoutv,"%d \t %lf\t %lf\n",counts,(double)counts/100.,fff / ((double)counts/100.));
      fclose(pfoutv); //all path ;
    
    return 0;
}



