#include "ros/ros.h"
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <std_msgs/Int32.h>
#include <std_msgs/UInt32.h>
#include <std_msgs/String.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/Image.h>

#include <boost/foreach.hpp>
#define foreach BOOST_FOREACH

using namespace std;

static void usage()
{

	cout << "Usage:" << endl;
	cout << "rosrun realsense2_camera rosbag_converter _input_bag:=\"input_bag\" _output_bag:=\"output_bag\" _fake_bag:=\"fake_bag\"" << endl;
}

int main(int argc,char **argv)
{
	ros::init(argc, argv, "rosbag_converter");
  	ros::NodeHandle nh("~");

  	std::string input_file, output_file, fake_file;

	if(nh.getParam("input_bag", input_file) && nh.getParam("output_bag", output_file)  && nh.getParam("fake_bag", fake_file))
	{
		//cout<< "OK" << endl;
		//
	
	}else{

		ROS_ERROR("Could not locate files...");
		usage();

		return -1;
	}

	// BAGS
	rosbag::Bag input_bag, output_bag, fake_bag;

	// Open Fake
	fake_bag.open(fake_file.c_str(), 	rosbag::bagmode::Read);
	input_bag.open(input_file.c_str(), 	rosbag::bagmode::Read);
	output_bag.open(output_file.c_str(),rosbag::bagmode::Write);

	ROS_INFO("Files found");

	sensor_msgs::CameraInfo::ConstPtr info_depth, info_infra_1, info_infra_2; 
	bool info_depth_taken = false, info_infra_1_taken = false, info_infra_2_taken = false;

	// Rosbag view for fake file
 	rosbag::View fake_msgs(fake_bag);

 	ros::Time begin_time(0.001);

 	// Save file version
 	std_msgs::UInt32 file_version_msg;
    file_version_msg.data = 4;

    output_bag.write("/file_version", begin_time, file_version_msg);
	
	// Scan for topics of interest and Write to basics to output
	for(rosbag::MessageInstance const m: fake_msgs)
    {
 
   		std::string const msg_type  = m.getDataType(); 
   		std::string const msg_topic = m.getTopic(); 
   		
   		if((strcmp(msg_topic.c_str(), "/device_0/sensor_0/Depth_0/info/camera_info") == 0) && (info_depth_taken == false)) {
   			
   			info_depth_taken = true;

   			info_depth = m.instantiate<sensor_msgs::CameraInfo>();

   			//output_bag.write(m.getTopic(), m.getTime(), *info_depth);

   		}else if((strcmp(msg_topic.c_str(),  "/device_0/sensor_0/Infrared_1/info/camera_info") == 0) && (info_infra_1_taken == false)) {
   			
   			info_infra_1_taken = true;

   			info_infra_1 = m.instantiate<sensor_msgs::CameraInfo>();

   			//output_bag.write(m.getTopic(), m.getTime(), *info_infra_1);
   			
   		}else if((strcmp(msg_topic.c_str(),  "/device_0/sensor_0/Infrared_2/info/camera_info") == 0) && (info_infra_2_taken == false)) {
   			
   			info_infra_2_taken = true;

   			info_infra_2 = m.instantiate<sensor_msgs::CameraInfo>();
			
			//output_bag.write(m.getTopic(), m.getTime(), *info_infra_2);   			
   		}else{

   			if(strcmp(msg_type.c_str(), "sensor_msgs/Image") != 0){

   				output_bag.write(msg_topic, begin_time, m);
   			}
   			
   		}
    	
    }

    fake_bag.close();

    if(!info_depth || !info_infra_1 || !info_infra_2){

    	ROS_ERROR("Could not load info from fake file");

		input_bag.close();
		output_bag.close();

    	return -2;
    }
    
    // Rosbag view for input file
    rosbag::View input_msgs(input_bag);

    ros::Time first_time;
    bool first_time_taken = false;

    foreach(rosbag::MessageInstance const m, input_msgs)
	{
		std::string const msg_type  = m.getDataType(); 
   		std::string const msg_topic = m.getTopic(); 

   		if(strcmp(msg_type.c_str(), "sensor_msgs/Image") == 0) {

   			first_time = m.getTime();
   			first_time_taken = true;
   			break;
   		}   	
    }

    if(!first_time_taken){
    	
    	ROS_ERROR("Could not grab time...");

		input_bag.close();
		output_bag.close();

    	return -3;

    }

    ros::Duration time_offset = first_time - begin_time;

	foreach(rosbag::MessageInstance const m, input_msgs)
	{
		std::string const msg_type  = m.getDataType(); 
   		std::string const msg_topic = m.getTopic(); 

   		ros::Time time_now = m.getTime() - time_offset;

   		if(strcmp(msg_topic.c_str(), "/device_0/sensor_0/Depth_0/image/data") == 0) {

   			output_bag.write("/device_0/sensor_0/Depth_0/info/camera_info", time_now, *info_depth);
   			output_bag.write(m.getTopic(), time_now, m);

   		}else if(strcmp(msg_topic.c_str(), "/device_0/sensor_0/Infrared_1/image/data") == 0) {

   			output_bag.write("/device_0/sensor_0/Infrared_1/info/camera_info", time_now, *info_infra_1);
   			output_bag.write(m.getTopic(), time_now, m);

   		}else if(strcmp(msg_topic.c_str(), "/device_0/sensor_0/Infrared_2/image/data") == 0) {

   			output_bag.write("/device_0/sensor_0/Infrared_2/info/camera_info", time_now, *info_infra_2);
   			output_bag.write(m.getTopic(), time_now, m);

   		}
	}

	ROS_INFO("Done... closing files...");

	input_bag.close();
	output_bag.close();

	return 0;

}











