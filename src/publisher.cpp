#include <ros/ros.h>
#include <std_msgs/Float64MultiArray.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Int16.h>
#include <tf/transform_broadcaster.h>
#include <sstream>
#include <hanp_msgs/TrackedHumans.h>
#include <hanp_msgs/HumanTrajectoryArray.h>
#include <tf/transform_listener.h>
#include <std_msgs/Time.h>
#include <plot/StampedFloat.h>
#include <cmath>
#include "std_msgs/Header.h"
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/Twist.h>

//socket

#include <unistd.h>
#include <stdio.h>
#include <sys/socket.h>
#include <stdlib.h>
#include <netinet/in.h>
#include <string.h>
#define PORT 8383


using namespace std;
geometry_msgs::PoseWithCovariance  human_pos_old ;
geometry_msgs::PoseWithCovariance  human_pos_now ;
ros::Time old_time ;
int new_socket;
nav_msgs::Path human_path;




void humanPositionCallback(const hanp_msgs::TrackedHumansConstPtr humans) {
	human_pos_old = human_pos_now;
	for (auto human: humans->humans){
		if (human.track_id == 3){
			for (auto segment: human.segments){
				if (segment.type == 1){
					human_pos_now = segment.pose;
//					humanPoseVector.resize(3);
//					humanPoseVector[0] = segment.pose.pose.position.x;
//					humanPose(segment.pose.pose.position.x, ...);
//					humanx = segment.pose.pose.position.x;
					break;
				}
			}
			break;
		}
	}
}

void humanPathsCallback(const hanp_msgs::HumanTrajectoryArrayConstPtr humans_traj) {
	for (auto traj: humans_traj->trajectories){
		if (traj.id == 3){
			human_path.poses.clear();
			human_path.poses.resize(traj.trajectory.points.size());
			human_path.header = traj.header;
			for (int i = 0; i < traj.trajectory.points.size(); i++){

				human_path.poses[i].pose.orientation = traj.trajectory.points[i].transform.rotation;
				human_path.poses[i].pose.position.x = traj.trajectory.points[i].transform.translation.x;
				human_path.poses[i].pose.position.y = traj.trajectory.points[i].transform.translation.y;
				human_path.poses[i].pose.position.z = traj.trajectory.points[i].transform.translation.z;
			}
		}
	}
}


void velCallback (const geometry_msgs::TwistConstPtr cmd_vel ){

	float vel[3];
	vel[0] = cmd_vel->linear.x ;
	vel[1] = cmd_vel->linear.y ;
	vel[2] = cmd_vel->angular.z ;

	ROS_DEBUG_STREAM("x " << vel[0]);


	send(new_socket , vel , sizeof(vel) , 0 );

}


void timerCallback(const ros::TimerEvent&){

	ROS_INFO("Callback triggered");


}




inline double penaltyBoundFromBelow(const double& var, const double& a,const double& epsilon)
 {
   if (var >= a+epsilon)
   {
     return 0.;
   }
   else
   {
     return (-var + (a+epsilon));
   }
 }



int main(int argc, char **argv)
{
  /**
   * The ros::init() function needs to see argc and argv so that it can perform
   * any ROS arguments and name remapping that were provided at the command line.
   * For programmatic remappings you can use a different version of init() which takes
   * remappings directly, but for most command-line programs, passing argc and argv is
   * the easiest way to do it.  The third argument to init() is the name of the node.
   *
   * You must call one of the versions of ros::init() before using any other
   * part of the ROS system.
   */
// %Tag(INIT)%
  ros::init(argc, argv, "talker");
// %EndTag(INIT)%

  /**
   * NodeHandle is the main access point to communications with the ROS system.
   * The first NodeHandle constructed will fully initialize this node, and the last
   * NodeHandle destructed will close down the node.
   */
// %Tag(NODEHANDLE)%hatter_dist", 1000);

  ros::NodeHandle n("observer");
// %EndTag(NODEHANDLE)%

  tf::TransformListener listener;

   /**
     * The advertise() function is how you tell ROS that you want to
     * publish on a given topic name. This invokes a call to the ROS
     * master node, which keeps a registry of who is publishing and who
     * is subscribing. After this advertise() call is made, the master
     * node will notify anyone who is trying to subscribe to this topic name,
     * and they will in turn negotiate a peer-to-peer connection with this
     * node.  advertise() returns a Publisher object which allows you to
     * publish messages on that topic through a call to publish().  Once
     * all copies of the returned Publisher object are destroyed, the topic
     * will be automatically unadvertised.
     *
     * The second parameter to advertise() is the size of the message queue
     * used for publishing messages.  If messages are published more quickly
     * than we can send them, the numsqrt()now.ber here specifies how many messages to
     * buffer up before throwing some away.
     */
  // %Tag(PUBLISHER)%

 //ros::Timer timer = n.createTimer(ros::Duration(0.5), timerCallback);





  int server_fd, valread;
  struct sockaddr_in address;
  int opt = 1;
  int addrlen = sizeof(address);
  char buffer[1024] = {0};


  // Creating socket file descriptor
  if ((server_fd = socket(AF_INET, SOCK_STREAM, 0)) == 0)
  {
      perror("socket failed");
      exit(EXIT_FAILURE);
  }

  // Forcefully attaching socket to the port 8080
  if (setsockopt(server_fd, SOL_SOCKET, SO_REUSEADDR | SO_REUSEPORT,
                                                &opt, sizeof(opt)))
  {
      perror("setsockopt");
      exit(EXIT_FAILURE);
  }
  address.sin_family = AF_INET;
  address.sin_addr.s_addr = INADDR_ANY;
  address.sin_port = htons( PORT );

  // Forcefully attaching socket to the port 8080
  if (bind(server_fd, (struct sockaddr *)&address,
                               sizeof(address))<0)
  {
      perror("bind failed");
      exit(EXIT_FAILURE);
  }
  if (listen(server_fd, 3) < 0)
  {
      perror("listen");
      exit(EXIT_FAILURE);
  }
  if ((new_socket = accept(server_fd, (struct sockaddr *)&address,
                     (socklen_t*)&addrlen))<0)
  {
      perror("accept");
      exit(EXIT_FAILURE);
  }




//  ros::Publisher robot_posx_pub  = n.advertise<plot::StampedFloat> ("robot_x", 1000);
//  ros::Publisher robot_posy_pub  = n.advertise<plot::StampedFloat> ("robot_y", 1000);

  ros::Publisher robot_pose_pub  = n.advertise<nav_msgs::Path> ("robot_pose", 1000);
  ros::Publisher human_pose_pub  = n.advertise<nav_msgs::Path> ("human_pose", 1000);

  ros::Publisher robot_speed_pub  = n.advertise<plot::StampedFloat> ("robot_speed", 1000);
  ros::Publisher human_speed_pub  = n.advertise<plot::StampedFloat> ("human_speed", 1000);

  ros::Publisher ttc_pub  = n.advertise<plot::StampedFloat> ("ttc", 1000);
  ros::Publisher ttcl_pub  = n.advertise<plot::StampedFloat> ("ttcl", 1000);

  ros::Publisher ttc_cost_pub  = n.advertise<plot::StampedFloat> ("ttc_cost", 1000);
  ros::Publisher ttc_plus_cost_pub  = n.advertise<plot::StampedFloat> ("ttc_plus_cost", 1000);
  ros::Publisher ttclosest_cost_pub  = n.advertise<plot::StampedFloat> ("ttclosest_cost", 1000);

  ros::Publisher i_pub  = n.advertise<plot::StampedFloat> ("i", 1000);
  ros::Publisher j_pub  = n.advertise<plot::StampedFloat> ("j", 1000);

  ros::Publisher hr_distance_pub = n.advertise<plot::StampedFloat> ("hr_distance", 1000);

  ros::Publisher speeds_dot_product_pub = n.advertise<plot::StampedFloat> ("speeds_dot_prod", 1000);

  ros::Subscriber human_position_sub = n.subscribe("/move_humans_node/humans", 1000, humanPositionCallback );
  ros::Subscriber human_paths_sub = n.subscribe("/move_base_node/TebLocalPlannerROS/human_local_trajs", 1000, humanPathsCallback);
  ros::Subscriber cmd_vel_sub = n.subscribe("/cmd_vel", 1000, velCallback );

  ros::Publisher vel_pub  = n.advertise<std_msgs::Float64MultiArray> ("vel_cmd", 1000);



  ros::Rate loop_rate(10);

  while (ros::ok())
  {
	 ros::Time now = ros::Time::now();

     tf::StampedTransform robot_pos_now;
     tf::StampedTransform robot_pos_old;
	 try{
	   robot_pos_old = robot_pos_now;
	   listener.lookupTransform("map", "base_footprint", ros::Time(0), robot_pos_now);

	 }
	 catch (tf::TransformException &ex) {
			  ROS_ERROR("%s",ex.what());
			  ros::Duration(1.0).sleep();
			  continue;
	 }


	 nav_msgs::Path path;
	 geometry_msgs::PoseStamped pose;


	 pose.header.stamp = now;
	 pose.header.frame_id = 1 ;
	 pose.pose.position.x = robot_pos_now.getOrigin().x();
	 pose.pose.position.y = robot_pos_now.getOrigin().y();

	 path.header.stamp = now;
	 pose.header.frame_id = 1;
	 path.poses.push_back(pose);
	 robot_pose_pub.publish(path);

//	 human_path.header.stamp = now;
//     human_path.header.frame_id = "map" ;
//	 human_pose_pub.publish(human_path);

	 nav_msgs::Path hpath;
	 geometry_msgs::PoseStamped hpose;

	 hpose.header.stamp = now;
	 hpose.header.frame_id = "odom" ;
	 hpose.pose.position.x = human_pos_now.pose.position.x;
	 hpose.pose.position.y = human_pos_now.pose.position.y;
	 hpose.pose.orientation.w = human_pos_now.pose.orientation.w ;
	 hpose.pose.orientation.x = human_pos_now.pose.orientation.x ;
	 hpose.pose.orientation.y = human_pos_now.pose.orientation.y ;
	 hpose.pose.orientation.z = human_pos_now.pose.orientation.z ;


	 hpath.header.stamp = now;
	 hpath.header.frame_id = "odom";
	 hpath.poses.push_back(hpose);
	 //hpath.poses = human_path.poses;
	 human_pose_pub.publish(hpath);

//	 geometry_msgs::PoseStamped pose;
//	 plot::StampedFloat robot_posx_msg;
//	 robot_posx_msg.header.stamp = now;
//	 robot_posx_msg.data = robot_pos_now.getOrigin().x();
//	 robot_posx_pub.publish(robot_posx_msg);
//
//
//	 plot::StampedFloat robot_posy_msg;
//	 robot_posy_msg.header.stamp = now;
//	 robot_posy_msg.data = robot_pos_now.getOrigin().y();
//	 robot_posy_pub.publish(robot_posy_msg);



	 double hr_dist = sqrt(pow(human_pos_now.pose.position.x - robot_pos_now.getOrigin().x(), 2) + pow(human_pos_now.pose.position.y - robot_pos_now.getOrigin().y(), 2));
	 plot::StampedFloat hr_distance_msg;
	 hr_distance_msg.header.stamp = now;
	 hr_distance_msg.data = hr_dist;
	 hr_distance_pub.publish(hr_distance_msg);


	 ros::Duration dt = now - old_time ;
	 old_time = now ;

	 double human_speed = sqrt(pow(human_pos_now.pose.position.x - human_pos_old.pose.position.x, 2) + pow(human_pos_now.pose.position.y - human_pos_old.pose.position.y , 2))/ dt.toSec() ;
	 plot::StampedFloat human_speed_msg;
	 human_speed_msg.header.stamp = now;
	 human_speed_msg.data = human_speed;
	 human_speed_pub.publish(human_speed_msg);


//	 ros::Timer timer = n.createTimer(ros::Duration(0.1), callback1);
//	 ros::Duration temp(1);
//	 double c;
//	 bool reset;
//	 timer.setPeriod( temp , reset) ;
//     timer.start();
//     c = c+1;
//	 plot::StampedFloat crono;
//	 crono.header.stamp = now ;
//	 crono.data = c ;
//	 crono_pub.publish(crono);

	 double robot_speed = sqrt(pow(robot_pos_now.getOrigin().x() - robot_pos_old.getOrigin().x(), 2) + pow(robot_pos_now.getOrigin().y() - robot_pos_old.getOrigin().y() , 2))/ dt.toSec() ;
	 plot::StampedFloat robot_speed_msg;
	 robot_speed_msg.header.stamp = now;
	 robot_speed_msg.data = robot_speed;
	 robot_speed_pub.publish(robot_speed_msg);




	 double r_speed_x = (robot_pos_now.getOrigin().x() - robot_pos_old.getOrigin().x()) /dt.toSec() ;
	 double r_speed_y = (robot_pos_now.getOrigin().y() - robot_pos_old.getOrigin().y()) /dt.toSec() ;
	 double h_speed_x = (human_pos_now.pose.position.x - human_pos_old.pose.position.x) /dt.toSec() ;
	 double h_speed_y = (human_pos_now.pose.position.y - human_pos_old.pose.position.y) /dt.toSec() ;

	 double speeds_dot_prod = (r_speed_x * h_speed_x) + (r_speed_y * h_speed_y);
	 plot::StampedFloat speeds_dot_prod_msg;
	 speeds_dot_prod_msg.header.stamp = now;
	 speeds_dot_prod_msg.data = speeds_dot_prod;
	 speeds_dot_product_pub.publish(speeds_dot_prod_msg);

     //V.dot(V)
	 double v_x = (r_speed_x - h_speed_x);
	 double v_y = (r_speed_y - h_speed_y);



	 plot::StampedFloat ttc_msg;
	 plot::StampedFloat ttcl_msg;

	 double robot_radius, human_radius;
	 if (!n.getParam("/move_base_node/TebLocalPlannerROS/footprint_model/radius", robot_radius)){
		 ROS_ERROR("No footprint radius set for the robot");
		 robot_radius = 0.34;
	 }
	 if (!n.getParam("/move_base_node/TebLocalPlannerROS/human_radius", human_radius)){
		 ROS_ERROR("No footprint radius set for the human");
		 human_radius = 0.2;
	 }
	 double radius_sum_sq_ = (robot_radius + human_radius) * (robot_radius + human_radius);
	 double ttc = std::numeric_limits<double>::infinity();
	 double ttcl = std::numeric_limits<double>::infinity();
	 int i , j;
	 double d=20;
	 double cost_ttc , cost_ttcplus , cost_ttcl;
	 double C_sq = hr_dist * hr_dist;
	    if (C_sq <= radius_sum_sq_) {
	      ttc = 0.0;
	    } else {
	    	double dist_speeds_dot_prod = ((human_pos_now.pose.position.x - robot_pos_now.getOrigin().x())* v_x) + ((human_pos_now.pose.position.y - robot_pos_now.getOrigin().y())* v_y);
	    	double V_sq =(v_x*v_x) +( v_y*v_y);

	        if (dist_speeds_dot_prod  > 0) { // otherwise ttc is infinite
	        double f = (dist_speeds_dot_prod * dist_speeds_dot_prod) - (V_sq * (C_sq - radius_sum_sq_));//25
	        double fe = (dist_speeds_dot_prod * dist_speeds_dot_prod) - (V_sq * (C_sq  - (0.5+radius_sum_sq_)));//30

	        	if (fe > 0) {
	        		//ttclosest
	        		                      ttcl = dist_speeds_dot_prod/ V_sq ;
	        			        		  ttcl_msg.header.stamp = now;
	        			        		  ttcl_msg.data = ttcl;
	        			        		  ttcl_pub.publish(ttcl_msg);
	        		if(f>0){
	        		// if(i==0){ d = C_sq;  }
	        		 i=i+1;

	        		 ttc = (dist_speeds_dot_prod - std::sqrt(f)) / V_sq;
	        		 ttc_msg.header.stamp = now;
	        		 ttc_msg.data = ttc;
	        		 ttc_pub.publish(ttc_msg);

                           }
	        	 }
	        }
	    }

	    plot::StampedFloat ttcl_cost_msg;
	    plot::StampedFloat ttc_cost_msg;
	    plot::StampedFloat ttc_plus_cost_msg;
	    plot::StampedFloat i_msg;
	    plot::StampedFloat j_msg;

//	    ttc_msg.header.stamp = now;
//	   	ttc_msg.data = ttc;
//	   	ttc_pub.publish(ttc_msg);

	   	i_msg.header.stamp = now;
	   	i_msg.data = i;
	   	i_pub.publish(i_msg);

	   	j_msg.header.stamp = now;
	   	j_msg.data = j;
	   	j_pub.publish(j_msg);


	if (ttc < std::numeric_limits<double>::infinity()) {
	    	 //plot::StampedFloat ttc_msg;
//	    	 ttc_msg.header.stamp = now;
//	    	 ttc_msg.data = ttc;
//	    	 ttc_pub.publish(ttc_msg);

		if( i > 5){                                                   //cfg_->human.ttcplus_timer
			    	j=j+1 ;
			    	i=0 ;
   // i=i+1;
	cost_ttc = penaltyBoundFromBelow(ttc, 5.0, 0.1);                  //5 is the threshold  cfg_->human.ttc_threshold
	cost_ttcplus = penaltyBoundFromBelow(ttc, 5.0, 0.1) *j/C_sq;      //5 is the threshold  cfg_->human.ttcplus_threshold


	//plot::StampedFloat ttc_cost_msg;
	ttc_cost_msg.header.stamp = now;
	ttc_cost_msg.data = cost_ttc;
	ttc_cost_pub.publish(ttc_cost_msg);

	//plot::StampedFloat ttc_plus_cost_msg;
	ttc_plus_cost_msg.header.stamp = now;
	ttc_plus_cost_msg.data = cost_ttcplus;
	ttc_plus_cost_pub.publish(ttc_plus_cost_msg);

    }
	    else {

	        cost_ttc = penaltyBoundFromBelow(ttc, 5.0, 0.1);              //5 is the treeshold
		    //cost_ttcplus = 0.0;                                         //5 is the treeshold

		    ttc_cost_msg.data = cost_ttc;
		    ttc_plus_cost_msg.data = cost_ttcplus;
		    ttc_cost_pub.publish(ttc_cost_msg);
		    ttc_plus_cost_pub.publish(ttc_plus_cost_msg);

	    }
	}
		else {
if( C_sq >2){
			      // no collision possible
			    	i=0;
			    	j=0;
			    	d=20;
			    	cost_ttcplus = 0.0;
			    	cost_ttc  = 0.0;
			    	ttc = std::numeric_limits<double>::infinity();
			    	ttc_msg.data = -1;
			    	ttc_pub.publish(ttc_msg);
			    	ttc_cost_pub.publish(ttc_cost_msg);
			    	ttc_plus_cost_pub.publish(ttc_plus_cost_msg);
			         }
		}


	//TTCLOSEST DISTANCE
	if (ttcl < std::numeric_limits<double>::infinity()) {
	      // if (ttcl > 0) {
	      //   // valid ttcl
	      //   _error[0] = penaltyBoundFromBelow(ttcl, cfg_->human.ttcl_threshold,
	      //                                 cfg_->optim.penalty_epsilon);
	      // } else {
	      //   // already in collision
	      //   _error[0] = cfg_->optim.max_ttcl_penalty;
	      // }

		    cost_ttcl = penaltyBoundFromBelow(ttcl, 5.0, 0.1);
	     	ttcl_cost_msg.header.stamp = now;
			ttcl_cost_msg.data = cost_ttcl;
			ttclosest_cost_pub.publish(ttcl_cost_msg);

	    } else {
	      // no collsion possible
	    	cost_ttcl = 0.0;
	    	ttcl = std::numeric_limits<double>::infinity();
	    	ttcl_msg.data = -1.5;
	    	ttcl_pub.publish(ttcl_msg);
	    	ttcl_cost_msg.header.stamp = now;
	    			ttcl_cost_msg.data = cost_ttcl;
	    			ttclosest_cost_pub.publish(ttcl_cost_msg);
	    }





//	    ttc_cost_pub.publish(ttc_cost_msg);
//	    ttc_plus_cost_pub.publish(ttc_plus_cost_msg);
//	    i_msg.header.stamp = now;
//	   	i_msg.data = i;
//	    i_pub.publish(i_msg);


// %Tag(SPINONCE)%
    ros::spinOnce();
// %EndTag(SPINONCE)%

// %Tag(RATE_SLEEP)%
    loop_rate.sleep();
// %EndTag(RATE_SLEEP)%

  }


  return 0;
}
// %EndTag(FULLTEXT)%
