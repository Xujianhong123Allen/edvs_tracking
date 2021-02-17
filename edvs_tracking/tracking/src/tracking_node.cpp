/**********************************************************
 * Author        : Allen
 * Email         : 1059074199@qq.com
 * Last modified : 2019-02-23 18:40
 * File Name      : tracking_node.cpp
 * Description   : 
 * *******************************************************/


#include "tracking/tracking.h"

int main(int argc, char * argv[])
{
	ros::init(argc, argv,"tracking");
	ros::NodeHandle nh;
	ros::NodeHandle nh_private("~");	
	
	Track::tracking* t = new Track::tracking(nh,nh_private);
	ros::spin();
	ros::shutdown();
	return 0;
}
