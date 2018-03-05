#include "ros/ros.h"
#include "std_msgs/Int32.h"
#include "sensor_sim/sensor_data.h"

int main(int argc, char **argv)
{
  ros::init(argc, argv, "Spoofer");

  ros::NodeHandle n;

  ros::Publisher Spoof_pub = n.advertise<sensor_sim::sensor_data>("wheeler_velocity", 10);

  ros::Rate loop_rate(7);

	bool countup=true;
  double count = 1.0;
  double step = 1.0;
  while (ros::ok())
  {
	sensor_sim::sensor_data vel;
	if(countup){
		count+=step;
	
		if(count>50){
		count=50.0;
		countup=false;	
		}
	}else{
		count-=step;
	
		if(count<1.0){
		count=1.0;
		countup=true;	
		}
	}
	vel.header.stamp=ros::Time::now();
	vel.header.frame_id="frames";

	vel.velocity=int(count);
    Spoof_pub.publish(vel);

    ros::spinOnce();

    loop_rate.sleep();
  }


  return 0;
}
