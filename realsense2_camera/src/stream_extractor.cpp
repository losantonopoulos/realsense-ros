#include "ros/ros.h"
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <std_msgs/Int32.h>
#include <std_msgs/UInt32.h>
#include <std_msgs/String.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/Image.h>

#include <sys/stat.h>
#include <sys/statvfs.h>
#include <sys/types.h>
#include <dirent.h>

#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <boost/foreach.hpp>
#define foreach BOOST_FOREACH

using namespace std;

static void usage()
{

	cout << "Usage:" << endl;
	cout << "rosrun realsense2_camera stream_extractor _input_bag:=\"input_bag\" _output_path:=\"output_path\"" << endl;
}

int main(int argc,char **argv)
{
	ros::init(argc, argv, "stream_extractor");
	ros::NodeHandle nh("~");

	std::string input_file, output_path;

	if(nh.getParam("input_bag", input_file))
	{
		//cout<< "OK" << endl;
		//
	
	}else{

		ROS_ERROR("Could not locate file...");
		usage();

		return -1;
	}

  if(nh.getParam("output_path", output_path))
  {
    //cout<< "OK" << endl;
    //
  
  }else{

    ROS_ERROR("Could not locate path...");
    usage();

    return -1;
  }

  int count_temp = strlen(output_path.c_str()) - 1;

  char file_path[500];

  while(count_temp>=0){

    if(output_path.c_str()[count_temp] == ' '){

      count_temp--;
      continue;

    }else if(output_path.c_str()[count_temp] == '/'){

      strncpy(file_path, output_path.c_str(), count_temp + 1);
      file_path[count_temp + 1] = NULL;

      break;
    }else{

      strncpy(file_path, output_path.c_str(), count_temp + 1);
      file_path[count_temp + 1] = '/';
      file_path[count_temp + 2] = NULL;

      break;
    }

  }

  stringstream file_space_1, file_space_2;

  file_space_1 << file_path << "infra_1";
  file_space_2 << file_path << "infra_2";

  DIR *save_dir = opendir(file_path);
  if (save_dir == NULL)
  {
    
    cerr << "Could not locate directory" << endl;
    free(file_path);
    exit(1);
  
  }else{

    int dir_err = mkdir(file_space_1.str().c_str(), S_IRWXU | S_IRWXG | S_IROTH | S_IXOTH);
    if (dir_err == -1)
    {
      cerr << "\nError creating directory!! \n" << endl;
      free(file_path);
      exit(1);
    }

    dir_err = mkdir(file_space_2.str().c_str(), S_IRWXU | S_IRWXG | S_IROTH | S_IXOTH);
    if (dir_err == -1)
    {
      cerr << "\nError creating directory!! \n" << endl;
      free(file_path);
      exit(1);
    }

  }
  
	// BAGS
	rosbag::Bag input_bag;

	// Open Bag
	input_bag.open(input_file.c_str(), 	rosbag::bagmode::Read);

	ROS_INFO("File found");

  // Rosbag view for input file
  rosbag::View input_msgs(input_bag);

  cv_bridge::CvImagePtr cv_ptr;

  int counter_infra_1 = 0, counter_infra_2 = 0; 

	foreach(rosbag::MessageInstance const m, input_msgs)
	{
		std::string const msg_type  = m.getDataType(); 
   	std::string const msg_topic = m.getTopic(); 


   		if(strcmp(msg_topic.c_str(), "/device_0/sensor_0/Infrared_1/image/data") == 0) {     

        cv_bridge::CvImagePtr cv_ptr;
        try
        {
          
          sensor_msgs::Image::ConstPtr imgMsgPtr = m.instantiate<sensor_msgs::Image>();
          
          cv_ptr = cv_bridge::toCvCopy(imgMsgPtr);

          stringstream filename;
          filename << file_space_1.str() << "/img_" << counter_infra_1 << ".png";

          //ROS_INFO("filename: %s", filename.str().c_str());

          imwrite(filename.str().c_str(), cv_ptr->image);

          counter_infra_1++;

        }
        catch (cv_bridge::Exception& e)
        {
          continue;
        }
        
   		}else if(strcmp(msg_topic.c_str(), "/device_0/sensor_0/Infrared_2/image/data") == 0) {

        cv_bridge::CvImagePtr cv_ptr;
        try
        {
          
          sensor_msgs::Image::ConstPtr imgMsgPtr = m.instantiate<sensor_msgs::Image>();
          
          cv_ptr = cv_bridge::toCvCopy(imgMsgPtr);

          stringstream filename;
          filename << file_space_2.str() << "/img_" << counter_infra_2 << ".png";

          //ROS_INFO("filename: %s", filename.str().c_str());

          imwrite(filename.str().c_str(), cv_ptr->image);

          counter_infra_2++;

        }
        catch (cv_bridge::Exception& e)
        {
          continue;
        }

   		}
	}

	ROS_INFO("Done... closing files...");

	input_bag.close();

  closedir(save_dir);

	return 0;

}
