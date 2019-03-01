#include"subscriber.h"


Subscriber::Subscriber(){
  sub = nodeHandle.subscribe("/cmd_vel" , 1000 , &Subscriber::topicCallback, this);
}

void Subscriber::topicCallback(const std_msgs::Float64::ConstPtr&  msg ){

	ROS_INFO(" ",  msg->Float64_)    ;

}

void Subscriber::run() {

	ros::spin();
}

int main(int argc, char **argv){

	ROS_INFO (" listener started");
    ros::init(argc, argv, "subscriber");
    Subscriber subscriber ;
    subscriber.run ;


//ros::NodeHandle n;

//ros::Subscriber sub = n.subscribe("/cmd_vel", 1000, &Subscriber::topicCallback);

//ros::spin();

return 0;


}

