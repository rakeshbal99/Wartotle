#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "turtlesim/Pose.h"
#include "turtlesim/Spawn.h"
#include "blackcurrent.hpp"
#include <cstdlib>
#include <fstream>

using namespace std ;
using namespace ros;

const double g_length =1.11;
const double g_diagonal =1.57;
const double PI =3.141592654;

Publisher translatement_pub;
Subscriber translatement_sub;
turtlesim::Pose turtlesim_pose;

void translate(double speed, double distance , bool isForward);
void rotate(double angular_speed, double angle, bool clockwise);
void setDesiredOrientation(double desired_angle_in_radians);
void poseCallback(const turtlesim::Pose::ConstPtr& pose_message);

int present_item=1;
int level =0;

struct Position{
 	int x;
 	int y;
 };

 ofstream outf("TURTLE.txt");

int visited[10][10][10];
Position pos;
int state [10][10];
int grid_bound(Position current){
	if(current.x<0 || current.x>9 || current.y <0 || current.y>9)
		return 0;
	else 
		return 1;
}

void Search_Method(int m, int n, int o,int p_x,int p_y){
	if (level ==10) {
		outf<<"Search is finished";
		exit(0);
	}
	outf<<"The turtle is at "<<m<<","<<n<<" and is searing for :: "<<o+1<<endl;
	
	if (visited[m][n][o]==1)
	{
		outf<<"I have visited this room ,Damnn!\n";
		outf<<"So now I have to translate back to the block from where I came !\n";
		outf<<"I came here from "<<p_x<<","<<p_y<<endl;
		//Write the Code to translate back to the parent::
		if (p_y<n){
			if (p_x<m){
				setDesiredOrientation(5*PI/4);
				translate(1,g_diagonal,1);
			}
			if (p_x=m){
				setDesiredOrientation(3*PI/2);
				translate(1,g_length,1);
				
			}
			if (p_x>m){
				setDesiredOrientation(7*PI/4);
				translate(1,g_diagonal,1);
				
			}
		}
		if (p_y>n){
			if (p_x<m){
				setDesiredOrientation(3*PI/4);
				translate(1,g_diagonal,1);
				
			}
			if (p_x=m){
				setDesiredOrientation(PI/2);
				translate(1,g_length,1);
				
			}
			if (p_x>m){
				setDesiredOrientation(PI/4);
				translate(1,g_diagonal,1);
				
			}
		}
		if (p_y=n){
			if (p_x<m){
				setDesiredOrientation(PI);
				translate(1,g_length,1);
			}
	
			if (p_x>m){
				setDesiredOrientation(0);
				translate(1,g_length,1);
				
			}
		
		}
	outf<<"Now I am at "<<p_x<<","<<p_y<<endl;
	return;
	}

	visited[m][n][o]=1;
	Position right;
	Position left;
	Position up;
	Position down;
	Position up_right;
	Position up_left;
	Position down_right;
	Position down_left;

	right.x = pos.x+1;
	right.y = pos.y ;
	left.x = pos.x-1;
	left.y = pos.y;
	up.x = pos.x;
	up.y = pos.y+1;
	down.x = pos.x;
	down.y = pos.y-1;
	up_right.x=pos.x+1;
	up_right.y=pos.y+1;
	up_left.x=pos.x-1;
	up_left.y=pos.y+1;
	down_left.x=pos.x-1;
	down_left.y=pos.y-1;
	down_right.x=pos.x+1;
	down_right.y=pos.y-1;

	state[right.x][right.y]=(grid_bound(right))?decode(right.x,right.y):-1;

	state[left.x][left.y]=(grid_bound(left))?decode(left.x,left.y):-1;

	state[up.x][up.y]=(grid_bound(up))?decode(up.x,up.y):-1;
	state[down.x][down.y]=(grid_bound(down))?decode(down.x,down.y):-1;

	state[up_right.x][up_right.y]=(grid_bound(up_right))?decode(up_right.x,up_right.y):-1;

	state[up_left.x][up_left.y]=(grid_bound(up_left))?decode(up_left.x,up_left.y):-1;
	state[down_right.x][down_right.y]=(grid_bound(down_right))?decode(down_right.x,down_right.y):-1;
	state[down_left.x][down_left.y]=(grid_bound(down_left))?decode(down_left.x,down_left.y):-1;
	

/*	outf<<endl;
	for (int j=9;j>=0;j--){
		for(int i=0;i<10;i++){
			outf<<state[i][j]<<" ";
		}
		outf<<endl;
	}*/

	if (
		state[right.x][right.y] 				==2 ||
		state[left.x][left.y] 				==2 ||
		state[up.x][up.y] 			==2 ||
		state[down.x][down.y] 			==2 ||
		state[up_right.x][up_right.y] 	==2 ||
		state[up_left.x][up_left.y] 	==2 ||
		state[down_right.x][down_right.y] 	==2 ||
		state[down_left.x][down_left.y] 	==2
		)
	{
		outf<<"	Found "<<present_item<<"\n";
		present_item++;
		level++;
		outf<<"Searching for "<<present_item;
		Search_Method(pos.x,pos.y,level,m,n);
	}

	else{
		if (grid_bound(up) && state[up.x][up.y]==0){
			outf<<"Moving to "<<up.x<<","<<up.y<<endl;
			pos.x=up.x;
			pos.y=up.y;
			setDesiredOrientation(PI/2);
			translate(1,g_length,1);
			Search_Method(pos.x,pos.y,level,m,n);
		}
		if (grid_bound(right) && state[right.x][right.y]==0){
			outf<<"Moving to "<<right.x<<","<<right.y<<endl;
			pos.x=right.x;
			pos.y=right.y;
			setDesiredOrientation(0);
			translate(1,g_length,1);
			Search_Method(pos.x,pos.y,level,m,n);
		}
		if (grid_bound(left) && state[left.x][left.y]==0){
			outf<<"Moving to "<<left.x<<","<<left.y<<endl;
			pos.x=left.x;
			pos.y=left.y;
			setDesiredOrientation(PI);
			translate(1,g_length,1);
			Search_Method(pos.x,pos.y,level,m,n);
		}
		if (grid_bound(down) && state[down.x][down.y]==0){
			outf<<"Moving to "<<down.x<<","<<down.y<<endl;
			pos.x=down.x;
			pos.y=down.y;
			setDesiredOrientation(3*PI/2);
			translate(1,g_length,1);
			Search_Method(pos.x,pos.y,level,m,n);
		}
		if (grid_bound(up_right) && state[up_right.x][up_right.y]==0){
			outf<<"Moving to "<<up_right.x<<","<<up_right.y<<endl;
			pos.x=up_right.x;
			pos.y=up_right.y;
			setDesiredOrientation(PI/4);
			translate(1,g_diagonal,1);
			Search_Method(pos.x,pos.y,level,m,n);
		}
		if (grid_bound(up_left) && state[up_left.x][up_left.y]==0){
			outf<<"Moving to "<<up_left.x<<","<<up_left.y<<endl;
			pos.x=up_left.x;
			pos.y=up_left.y;
			setDesiredOrientation(3*PI/4);
			translate(1,g_diagonal,1);
			Search_Method(pos.x,pos.y,level,m,n);
		}
		if (grid_bound(down_right) && state[down_right.x][down_right.y]==0){
			outf<<"Moving to "<<down_right.x<<","<<down_right.y<<endl;
			pos.x=down_right.x;
			pos.y=down_right.y;
			setDesiredOrientation(7*PI/4);
			translate(1,g_diagonal,1);
			Search_Method(pos.x,pos.y,level,m,n);
		}
		if (grid_bound(down_left) && state[down_left.x][down_left.y]==0){
			outf<<"Moving to "<<down_left.x<<","<<down_left.y<<endl;
			pos.x=down_left.x;
			pos.y=down_left.y;
			setDesiredOrientation(5*PI/4);
			translate(1,g_diagonal,1);
			Search_Method(pos.x,pos.y,level,m,n);
		}
		outf<<"Looks like I have exhausted this room\n";
		outf<<"Lets go back to "<<p_x<<","<<p_y<<endl;
		if (p_y<n){
			if (p_x<m){
				setDesiredOrientation(5*PI/4);
				translate(1,g_diagonal,1);
			}
			if (p_x=m){
				setDesiredOrientation(3*PI/2);
				translate(1,g_length,1);
			}
			if (p_x>m){
				setDesiredOrientation(7*PI/4);
				translate(1,g_diagonal,1);
			}
		}
		if (p_y>n){
			if (p_x<m){
				setDesiredOrientation(3*PI/4);
				translate(1,g_diagonal,1);
			}
			if (p_x=m){
				setDesiredOrientation(PI/2);
				translate(1,g_length,1);
			}
			if (p_x>m){
				setDesiredOrientation(PI/4);
				translate(1,g_diagonal,1);
			}
		}
		if (p_y=n){
			if (p_x<m){
				setDesiredOrientation(PI);
				translate(1,g_length,1);
			}
	
			if (p_x>m){
				setDesiredOrientation(0);
				translate(1,g_length,1);
			}
		}

		outf<<"Now I am at "<<p_x<<","<<p_y<<endl;
	}
}

int main(int argc, char **argv)
{
	ros::init(argc, argv,"turtle_bot"); 
	ros::NodeHandle n;					
	translatement_pub = n.advertise<geometry_msgs::Twist>("/turtle_bot/cmd_vel",10);
	//news_publisher =n.advertise<krssg::news>("newsfeed",1000);
	translatement_sub =n.subscribe("/turtle_bot/pose",10,poseCallback);
	//create an object of type ros::ServiceClient , whose job is to actually carry out the service call.
	outf<<"Upto this done\n";
	ros::ServiceClient client =n.serviceClient<turtlesim::Spawn>("spawn");
	turtlesim::Spawn::Request req;
	turtlesim::Spawn::Response resp ;
	req.x = 1.10399043/2;
	req.y = 1.10399043/2;
	req.theta =0;
	req.name="turtle_bot";
	bool success = client.call(req,resp);
	if(success){
		ROS_INFO_STREAM("Spawned_a_turtle_named\n"<<resp.name);
	}
	else
	{
		ROS_ERROR_STREAM("Failed_to_spawn.");
	}

	outf<<"//initialise all visited rooms to zero\n";

	for(int i=0;i<10;i++){
		for(int j=0;j<10;j++){
			for(int k=0;k<10;k++){
				visited[i][j][k]=0;

			}
		}
	} 	
	outf<<"//initialise all elements in state state to 9\n";
	for (int i=0;i<10;i++){
		for(int j=0;j<10;j++){
			state[i][j]=9;
		}
	}
	pos.x=0;
	pos.y=0;
	outf<<"//but make 0,0 of state =0\n";
	state[0][0]=0;
	outf<<"//Lets start the search\n";
	Search_Method(pos.x,pos.y,level,0,0);	
}
void poseCallback(const turtlesim::Pose::ConstPtr& pose_message){
	turtlesim_pose.x=pose_message->x;
	turtlesim_pose.y=pose_message->y;
	turtlesim_pose.theta=pose_message->theta;
}
void translate (double speed, double distance , bool isForward){

	geometry_msgs::Twist vel_msg;
	if(isForward)
		vel_msg.linear.x=abs(speed);
	else
		vel_msg.linear.x=-abs(speed);
	vel_msg.linear.y=0;
	vel_msg.linear.z=0;

	vel_msg.angular.x =0;
	vel_msg.angular.y =0;
	vel_msg.angular.z =0;
	double t0=ros::Time::now().toSec();
	double current_distance=0;
	ros::Rate loop_rate(500000);
	do{
		translatement_pub.publish(vel_msg);
		double t1=ros::Time::now().toSec();
		current_distance=speed*(t1-t0);
		ros::spinOnce();
		loop_rate.sleep();

	}while(current_distance<distance);
	vel_msg.linear.x=0;
	translatement_pub.publish(vel_msg);
}


void rotate(double angular_speed, double angle, bool clockwise){
	outf<<"Ing_length rotate function\n";
	geometry_msgs::Twist vel_msg;
	vel_msg.linear.x=0;
	vel_msg.linear.y=0;
	vel_msg.linear.z=0;
	vel_msg.angular.x=0;
	vel_msg.angular.y=0;
	if (clockwise)
		vel_msg.angular.z=-abs(angular_speed);
	else
		vel_msg.angular.z=abs(angular_speed);
	outf<<"omegas to be published"<<vel_msg.angular.z<<"\n";
	double t0=ros::Time::now().toSec();
	double current_angle=0.0;
	ros::Rate loop_rate(500);
	do{
		translatement_pub.publish(vel_msg);
		double t1=ros::Time::now().toSec();
		current_angle=angular_speed*(t1-t0);
		ros::spinOnce();
		loop_rate.sleep();

	}while(current_angle<=angle);
	vel_msg.angular.z=0;
	translatement_pub.publish(vel_msg);
}

void setDesiredOrientation(double desired_angle_in_radians)
{
	static double ang=0.0;
	/*
	double relative_angle_radians=desired_angle_in_radians - turtlesim_pose.theta;*/
	double relative_angle_radians=desired_angle_in_radians - ang;
	outf<<"relative_angle_radians= "<<relative_angle_radians<<endl;
	bool clockwise =((relative_angle_radians<0)?true:false);
	outf<<"Its clockwise ="<<clockwise<<endl;
	rotate(abs(relative_angle_radians)/4,abs(relative_angle_radians),clockwise);
	ang+=relative_angle_radians;
	while(ang<=(2*PI)) ang-=(2*PI);

}