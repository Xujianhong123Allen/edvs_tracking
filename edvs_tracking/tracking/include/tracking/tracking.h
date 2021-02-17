/**********************************************************
 * Author        : Allen
 * Email         : 1059074199@qq.com
 * Last modified : 2019-02-23 18:41
 * File Name      : tracking.h
 * Description   : 
 * *******************************************************/

#ifndef TRACKING_H
#define TRACKING_H

#include <ros/ros.h>
#include <dvs_msgs/EventArray.h>
#include <dvs_msgs/Event.h>
#include <std_msgs/Time.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <geometry_msgs/Pose.h>
#include <image_transport/image_transport.h>
#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <cv_bridge/cv_bridge.h>
#include <iostream>
#include <math.h>
#include <vector>
#include <Eigen/Core>
#include <g2o/core/base_vertex.h>
#include <g2o/core/base_unary_edge.h>
#include <g2o/core/sparse_optimizer.h>
#include <g2o/core/block_solver.h>
#include <g2o/core/optimization_algorithm_levenberg.h>
#include <g2o/solvers/csparse/linear_solver_csparse.h>
#include <g2o/types/sba/types_six_dof_expmap.h>
#include <sophus/se3.hpp>

namespace Track
{

	inline int get_length(int x)
	{
		int leng = 0;
		while (x)
		{
			x /= 10;
			leng++;
		}
		return leng;
	}

	struct event_xy
	{
		uint16_t x;
		uint16_t y;
		bool operator<(const struct event_xy &Pair) const
		{
			double X = x * pow(10, get_length(y)) + y;
			double Y = Pair.x * pow(10, get_length(Pair.y)) + Pair.y;

			return X < Y;
		}
	};

	struct Hypertransition
	{
		uint16_t x;
		uint16_t y;
		uint64_t deltaT;
		ros::Time ts;
	};

	// struct Freq_ROI
	// {
	// 	cv::Rect roi;
	// 	double freq;
	// };

	const int SIZE = 4;
	const int Total_point_num = 4;
	class tracking
	{
	public:
		tracking(ros::NodeHandle &nh, ros::NodeHandle nh_private);
		~tracking();

		bool if_ROI(struct event_xy e);

	private:
		ros::NodeHandle nh_;
		ros::Subscriber EventArray_sub_;
		ros::Publisher track_image_pub_;
		ros::Publisher Pose_pub_;

		std::map<event_xy, bool> event_map_;
		std::map<event_xy, uint64_t> event_negative_map_;
		std::map<event_xy, uint64_t> event_positive_map_;
		std::map<event_xy, uint64_t> event_interval_map_;
		std::vector<cv::Point2f> Estimate_Points_Centers;

		// std::vector<struct  Freq_ROI> Freq_ROI_vector;
		std::vector<cv::Rect> ROI_vector;
		// std::map<cv::Rect, int> ROI_map;

		void Tracking_GenerateCb(const dvs_msgs::EventArray::ConstPtr &msg);

		dvs_msgs::Event getTransition(dvs_msgs::Event event);

		// dvs_msgs::Hypertransition getInterval(dvs_msgs::Event t);
		struct Hypertransition getInterval(dvs_msgs::Event t);

		int dataNum; //数据集中数据记录数目

		cv::Point2f Mean_Point_origin;
		cv::Point2f Mean_Point_new;

		bool Find = false;

		// std::vector<cv::Point2f> Points2D_origin;
		// std::vector<cv::Point2f> Points2D_new;

		std::vector<cv::Point3f> Points3D;
		std::vector<cv::Point2f> Points2D;

		std::vector<cv::Point2f> Points_sorted;
	};

	double get_distance(cv::Point2f p1, cv::Point2f p2);

	/* 四元数结构体 */
	struct Quaternion
	{
		double w;
		double x;
		double y;
		double z;
	};
	struct Quaternion rotMatrix2Quaternion(cv::Mat M);
	void set_world_points(std::vector<cv::Point3f> &Points3D);
	void set_image_points(std::vector<cv::Point2f> &Points2D, std::vector<cv::Point2f> &Points_sorted);
	Eigen::MatrixXd bundleAdjustment(const std::vector<cv::Point3f> points_3d, const std::vector<cv::Point2f> points_2d, const cv::Mat &K, cv::Mat &R, cv::Mat &t);

} // namespace Track

#endif
