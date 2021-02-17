/**********************************************************
 * Author        : Allen
 * Email         : 1059074199@qq.com
 * Last modified : 2019-02-23 18:40
 * File Name      : tracking.cpp
 * Description   :
 * *******************************************************/

#include "tracking/tracking.h"

namespace Track
{
	/* 初始化相机参数 */
	/* 内参矩阵K */
	double camD[9] = {
		400.1061765571045, 0, 314.9471185496827,
		0, 400.5917805246362, 166.5324674387784,
		0, 0, 1};
	cv::Mat camera_matrix = cv::Mat(3, 3, CV_64FC1, camD);
	/* 畸变参数D */
	double distCoeffD[5] = {-0.384620026472871, 0.13140385701034, -0.0004526959001432161, 0.001907582118624881, 0};
	cv::Mat distortion_coefficients = cv::Mat(5, 1, CV_64FC1, distCoeffD);

	tracking::tracking(ros::NodeHandle &nh, ros::NodeHandle nh_private)
	{
		EventArray_sub_ = nh.subscribe("/dvs/events", 1, &tracking::Tracking_GenerateCb, this);
		// track_points_pub_ = nh.advertise<dvs_msgs::Pt>("/edvs/track_points",1,true);
		track_image_pub_ = nh.advertise<sensor_msgs::Image>("/dvs/track_image", 1, true);
		Pose_pub_ = nh.advertise<geometry_msgs::Pose>("/dvs/pose", 1, true);

		Find = true;

		set_world_points(Points3D);

		std::cout << "tracking node runs!" << std::endl;
	}

	tracking::~tracking()
	{

		std::cout << "\nTracker node quit!" << std::endl;

		track_image_pub_.shutdown();
	}

	void tracking::Tracking_GenerateCb(const dvs_msgs::EventArray::ConstPtr &msg)
	{
		struct event_xy event_key
		{
			0, 0
		};
		// std::cout << "generate!" << std::endl;
		// 计算一个事件组中每个像素的时间间隔与频率，并根据频率来去除噪声
		for (int i = 0; i < msg->events.size(); i++)
		{
			dvs_msgs::Event t = getTransition(msg->events[i]);
			if (t.ts.toNSec() == 0)
				continue;
			struct Hypertransition h = getInterval(t);
			if (h.deltaT == 0)
				continue;
			event_key.x = h.x;
			event_key.y = h.y;
			double freq = (1.0 / (h.deltaT * 1e-9));

			// std::cout << h.x << "\t" << h.y << "\t" << freq <<std::endl;
			// if((int)freq <= 150)
			if ((int)freq <= 200)
			{
				event_interval_map_.erase(event_key);
			}
			else
			{
				event_interval_map_[event_key] = h.deltaT;
			}
		}
		//DBSCN基于点的密度滤波
		for (std::map<event_xy, uint64_t>::iterator it = event_interval_map_.begin(); it != event_interval_map_.end();)
		{
			int num_points = 0;
			for (uint j = it->first.x - 1; j <= (it->first.x + 1); j++)
			{
				for (uint k = it->first.y - 1; k <= (it->first.y + 1); k++)
				{
					event_key.x = j;
					event_key.y = k;
					if ((event_interval_map_.find(event_key) != event_interval_map_.end()))
					// if (event_map.find(event_key) != event_map.end())
					{
						num_points++;
					}
				}
			}
			event_key.x = it->first.x;
			event_key.y = it->first.y;
			if (num_points <= 2)
			{
				event_interval_map_.erase(it++);
			}
			else
			{
				it++;
			}
		}

		for (auto i : event_interval_map_)
		{
			bool if_in_ROI = false;

			// 判断该事件是否在ROI区域内
			for (int x = 0; x < ROI_vector.size(); x++)
			{
				if (i.first.x >= ROI_vector[x].x && i.first.x <= (ROI_vector[x].x + ROI_vector[x].width) && i.first.y >= ROI_vector[x].y && i.first.y <= (ROI_vector[x].y + ROI_vector[x].height))
				{
					if_in_ROI = true;
					break;
				}
			}

			if (if_in_ROI == false)
			{
				bool if_not_ROI = false;
				if_not_ROI = if_ROI(i.first);
				if (if_not_ROI == true)
				{
					// 画出ROI区域
					cv::Rect roi = cv::Rect(i.first.x, i.first.y - 5, 12, 12);

					double max = 0;
					double average = 0;
					double average_x = 0;
					double average_y = 0;
					int count = 0;
					for (int x = i.first.x; x <= (i.first.x + 11); x++)
					{
						for (int y = i.first.y - 5; y <= (i.first.y + 6); y++)
						{
							event_key.x = x;
							event_key.y = y;

							if (event_interval_map_.find(event_key) != event_interval_map_.end())
							{
								average = average + (1.0 / (event_interval_map_[event_key] * 1e-9));
								count++;
								// average_x = average_x + (event_key.x * (1.0 / (event_interval_map_[event_key] * 1e-9)));
								// average_y = average_y + (event_key.y * (1.0 / (event_interval_map_[event_key] * 1e-9)));？
								average_x = average_x + event_key.x;
								average_y = average_y + event_key.y;
								// if (max < (1.0 / (event_interval_map_[event_key] * 1e-9)))
								// {
								// 	max = (1.0 / (event_interval_map_[event_key] * 1e-9));
								// }
							}
						}
					}
					// average_x = average_x / average;
					// average_y = average_y / average;
					average_x = average_x / count;
					average_y = average_y / count;

					// event_key.x = ceil(average_x);
					// event_key.y = ceil(average_y);
					// max = (1.0/(event_interval_map_[event_key] * 1e-9));
					// The max is max frequency within the region of interest
					// struct Freq_ROI freq_roi
					// {
					// 	roi,  (average/count)
					// 	// roi, max
					// };

					Estimate_Points_Centers.push_back(cv::Point2f(average_x, average_y));
					// Freq_ROI_vector.push_back(freq_roi);
					ROI_vector.push_back(roi);
				}
			}
		}

		cv_bridge::CvImage cv_image;

		if (msg->events.size() > 0)
		{
			cv_image.header.stamp = msg->events[msg->events.size() / 2].ts;
		}

		cv_image.encoding = "bgr8";

		cv_image.image = cv::Mat(msg->height, msg->width, CV_8UC3);
		cv_image.image = cv::Scalar(0, 0, 0);

		// 绘制有效事件点
		/* 		for (auto i : event_interval_map_)
		{

			const int x = i.first.x;
			const int y = i.first.y;
			int value = i.second;
			// std::cout<<"interval(ns) : "<< (i.second ) <<std::endl;
			double freq = (1.0 / (i.second * 1e-9));
			// std::cout<<x<<"\t"<<y<<"\t"<<freq<<std::endl;
			// int color = (int)freq;
			int color = int(freq);
			// std::cout<<"color : "<< color <<std::endl;
			// if(color>255)
			// {
			// 	color = 255;
			// }
			// if(color<0)
			// {
			// 	color = 0;
			// }

			// if(color != 0)
			// std::cout<<x<<"\t"<<y<<"\t"<<color<<std::endl;
			// std::cout<<x<<"\t"<<y<<"\t"<<freq<<std::endl;
			// cv_image.image.at<cv::Vec3b>(cv::Point(x, y)) = (cv::Vec3b(color, color, color));
			cv_image.image.at<cv::Vec3b>(cv::Point(x, y)) = cv::Vec3b(255, 255, 255);
		} */

		//检测同一特征点，并去除
		for (std::vector<cv::Point2f>::iterator it = Estimate_Points_Centers.begin(); it != Estimate_Points_Centers.end();)
		{
			int center_num = 0;

			for (std::vector<cv::Point2f>::iterator jt = Estimate_Points_Centers.begin(); jt != Estimate_Points_Centers.end(); jt++)
			{
				if (get_distance((*it), (*jt)) < 15)
				{
					center_num++;
				}
			}

			if (center_num >= 2)
			{
				it = Estimate_Points_Centers.erase(it);
			}
			else
			{
				it++;
			}
		}

		// for (int i = 0; i < Estimate_Points_Centers.size(); i++)
		// {
		// 	std::cout << "Estimate_Points_Centers[" << i << "] : " << Estimate_Points_Centers[i].x << "\t" << Estimate_Points_Centers[i].y << std::endl;
		// }

		// 搜索特征点的对应关系
		Points_sorted.clear();
		for (int i = 0; i < Total_point_num; i++)
		{
			Points_sorted.push_back(cv::Point2f(0, 0));
		}
		if (Estimate_Points_Centers.size() == 4)
		{
			// 1.确保估计特征点在不同的ROI区域中
			bool Possbile_Points = true;
			for (int i = 0; i < Total_point_num; i++)
			{
				for (int j = i + 1; j < Total_point_num; j++)
				{
					// 	if(sqrt((Estimate_Points_Centers[j].x-Estimate_Points_Centers[i].x)*(Estimate_Points_Centers[j].x-Estimate_Points_Centers[i].x)
					// 		+(Estimate_Points_Centers[j].y-Estimate_Points_Centers[i].y)*(Estimate_Points_Centers[j].y-Estimate_Points_Centers[i].y))<5)
					if (get_distance(Estimate_Points_Centers[j], Estimate_Points_Centers[i]) < 5)
					{
						Possbile_Points = false;
					}
				}
			}
			// std::cout << "Possbile_Points:" << Possbile_Points << std::endl;
			if (Possbile_Points == true)
			{
				// 2.求出四个特征点围成的多边形形心
				double average_x = 0, average_y = 0;
				for (int i = 0; i < Total_point_num; i++)
				{
					average_x = average_x + Estimate_Points_Centers[i].x;
					average_y = average_y + Estimate_Points_Centers[i].y;
				}
				average_x = average_x / Total_point_num;
				average_y = average_y / Total_point_num;
				cv::Point2f Center_point = cv::Point2f(average_x, average_y);

				cv::circle(cv_image.image, Center_point, 1, CV_RGB(255, 255, 255), -1); //形心为白色

				// 3.搜索特征点到形心的最小距离的特征点o
				double distance[Total_point_num];
				for (int i = 0; i < Total_point_num; i++)
				{
					// distance[i] = sqrt((Estimate_Points_Centers[i].x - Center_point.x)*(Estimate_Points_Centers[i].x - Center_point.x) +
					// 	(Estimate_Points_Centers[i].y - Center_point.y)*(Estimate_Points_Centers[i].y - Center_point.y) );
					distance[i] = get_distance(Estimate_Points_Centers[i], Center_point);
				}
				int min_distance_id_1 = -1;
				double min_distance_1 = 128;
				for (int i = 0; i < Total_point_num; i++)
				{
					if (min_distance_1 > distance[i])
					{
						min_distance_1 = distance[i];
						min_distance_id_1 = i;
					}
				}
				if (min_distance_id_1 != -1)
				{
					Points_sorted[0] = Estimate_Points_Centers[min_distance_id_1];
				}

				//4.search the min distance of detected point and the first point(Pt[0])
				// 4.搜索特征点0与其余特征点最小距离，为特征点1
				for (int i = 0; i < Total_point_num; i++)
				{
					distance[i] = 0;
				}
				for (int i = 0; i < Total_point_num; i++)
				{
					if (i != min_distance_id_1)
					{
						// distance[i] = sqrt((Estimate_Points_Centers[i].x - PT[0].x)*(Estimate_Points_Centers[i].x - PT[0].x) +
						// 	(Estimate_Points_Centers[i].y - PT[0].y)*(Estimate_Points_Centers[i].y - PT[0].y) );
						distance[i] = get_distance(Estimate_Points_Centers[i], Points_sorted[0]);
					}
				}
				int min_distance_id_2 = -1;
				double min_distance_2 = 0;
				if (distance[0] == 0)
				{
					min_distance_2 = distance[1];
					min_distance_id_2 = 1;
				}
				else
				{
					min_distance_2 = distance[0];
					min_distance_id_2 = 0;
				}
				for (int i = 0; i < Total_point_num; i++)
				{
					if ((min_distance_2 > distance[i]) && (distance[i] != 0))
					{
						min_distance_2 = distance[i];
						min_distance_id_2 = i;
					}
				}
				if (min_distance_id_2 != -1)
				{
					Points_sorted[1] = Estimate_Points_Centers[min_distance_id_2];
				}

				//5.search the second min distance of detected point and the second point(Pt[1])
				// 5.搜索特征点1与其余特征点第二短距离的点，为特征点2，剩余特征点3
				for (int i = 0; i < Total_point_num; i++)
				{
					distance[i] = 0;
				}
				for (int i = 0; i < Total_point_num; i++)
				{
					if ((min_distance_id_1 != i) && (min_distance_id_2 != i))
					{
						// distance[i] = sqrt((Estimate_Points_Centers[i].x - PT[1].x)*(Estimate_Points_Centers[i].x - PT[1].x) +
						// 	(Estimate_Points_Centers[i].y - PT[1].y)*(Estimate_Points_Centers[i].y - PT[1].y) );
						distance[i] = get_distance(Estimate_Points_Centers[i], Points_sorted[1]);
					}
				}
				double min_distance_3;
				int distance_no_zero_id[2];
				int min_distance_id_3 = -1;
				int max_distance_id_4 = -1;

				for (int i = 0, j = 0; i < Total_point_num; i++)
				{
					if (distance[i] != 0)
					{
						distance_no_zero_id[j] = i;
						j++;
					}
				}
				if (distance[distance_no_zero_id[0]] < distance[distance_no_zero_id[1]])
				{
					min_distance_id_3 = distance_no_zero_id[0];
					max_distance_id_4 = distance_no_zero_id[1];
				}
				else
				{
					min_distance_id_3 = distance_no_zero_id[1];
					max_distance_id_4 = distance_no_zero_id[0];
				}
				if (min_distance_id_3 != -1)
				{
					Points_sorted[2] = Estimate_Points_Centers[min_distance_id_3];
				}
				if (max_distance_id_4 != -1)
				{
					Points_sorted[3] = Estimate_Points_Centers[max_distance_id_4];
				}

				// for(auto i:Pt)
				// {
				// 	std::cout << i.x << "\t" <<i.y <<std::endl;
				// }

				//6.Detect if the four points meet the requirements : cos<Pt[0]Pt[1],Pt[0]Pt[3]> < 0
				Eigen::Vector2d V_01, V_03;
				V_01 << (Points_sorted[1].x - Points_sorted[0].x), (Points_sorted[1].y - Points_sorted[0].y);
				V_03 << (Points_sorted[3].x - Points_sorted[0].x), (Points_sorted[3].y - Points_sorted[0].y);

				bool find_correct = false;
				if (V_01.dot(V_03) < 0)
				{
					// for (auto i : Points_sorted)
					// {
					// 	std::cout << "特征点" << i << ": " << i.x << "\t" << i.y << std::endl;
					// }
					std::cout << std::endl;
					// //Send four tracking points with the correct order to the ros topic
					// dvs_msgs::Pt p;
					// for(int i=0;i < Total_point_num;i++)
					// {
					// 	p.point[i].x = PT[i].x;
					// 	p.point[i].y = PT[i].y;
					// 	p.point[i].z = 0;
					// }
					// track_points_pub_.publish(p);
				}
			}
		}

		// std::cout << "tracking successfully!" <<std::endl;
		// for (int i = 0; i < Total_point_num; i++)
		// {
		// 	std::cout << i << ": " << Points_sorted[i].x << "\t" << Points_sorted[i].y << std::endl;
		// }
		if ((Points_sorted[0].x != 0) && (Points_sorted[0].y != 0))
		{
			// Draw four tracking points with the correct
			for (int i = 0; i < Total_point_num; i++)
			{
				switch (i)
				{
				case 0:
					cv::circle(cv_image.image, Points_sorted[i], 4, cv::Scalar(255, 0, 0), -1, 8, 0);
					break; //red
				case 1:
					cv::circle(cv_image.image, Points_sorted[i], 4, cv::Scalar(0, 255, 0), -1, 8, 0);
					break; //green
				case 2:
					cv::circle(cv_image.image, Points_sorted[i], 4, cv::Scalar(0, 0, 255), -1, 8, 0);
					break; //bule
				case 3:
					cv::circle(cv_image.image, Points_sorted[i], 4, cv::Scalar(0, 255, 255), -1, 8, 0);
					break; //yellow
				}
			}

			//Draw the lines
			//画出多边形
			cv::line(cv_image.image, Points_sorted[0], Points_sorted[1], CV_RGB(128, 128, 128));
			cv::line(cv_image.image, Points_sorted[1], Points_sorted[2], CV_RGB(128, 128, 128));
			cv::line(cv_image.image, Points_sorted[2], Points_sorted[3], CV_RGB(128, 128, 128));
			cv::line(cv_image.image, Points_sorted[3], Points_sorted[0], CV_RGB(128, 128, 128));

			/* //确保ROI容器对应着每个特征点
			std::vector<cv::Rect> ROI_vector_new;
			for (int i = 0; i < Total_point_num; i++)
			{
				for (int j = 0; j < Total_point_num; j++)
				{
					if ((Points_sorted[i].x > ROI_vector[j].x) && (Points_sorted[i].x < (ROI_vector[j].x + ROI_vector[j].width)) &&
						(Points_sorted[i].y > ROI_vector[j].y) && (Points_sorted[i].y < (ROI_vector[j].y + ROI_vector[j].height)))
					{
						ROI_vector_new.push_back(ROI_vector[j]);
						break;
					}
				}
			}
			// 画出ROI
			cv::rectangle(cv_image.image, ROI_vector_new[0], cv::Scalar(255, 0, 0), 1);
			cv::rectangle(cv_image.image, ROI_vector_new[1], cv::Scalar(0, 255, 0), 1);
			cv::rectangle(cv_image.image, ROI_vector_new[2], cv::Scalar(0, 0, 255), 1);
			cv::rectangle(cv_image.image, ROI_vector_new[3], cv::Scalar(0, 255, 255), 1); */

			std::cout << std::endl;

			// 估计位姿
			Points2D.clear();
			set_image_points(Points2D, Points_sorted);
			if ((Points2D.size() == 4) && (Points3D.size() == 4))
			{

				cv::Mat tvec = cv::Mat::zeros(3, 1, CV_64FC1);
				cv::Mat rvec = cv::Mat::zeros(3, 1, CV_64FC1);
				double rm[9];
				cv::Mat rotM(3, 3, CV_64FC1, rm);
				solvePnP(Points3D, Points2D, camera_matrix, distortion_coefficients, rvec, tvec, false, cv::SOLVEPNP_P3P);
				Rodrigues(rvec, rotM); //罗德里格斯公式，旋转向量转化为旋转矩阵
				// Quaternion quat;
				// quat = rotMatrix2Quaternion(rotM);
				// std::cout << "优化前的位姿：" << std::endl;
				// std::cout << "四元数:\nw:" << quat.w << "\tx:" << quat.x
				// 		  << "\ty:" << quat.y << "\tz:" << quat.z << std::endl;
				// std::cout << "平移向量：\n" << tvec << std::endl;

				// 利用g2o进行bundle Adjustment优化
				Eigen::MatrixXd T_optimized = bundleAdjustment(Points3D, Points2D, camera_matrix, rotM, tvec);
				Eigen::Matrix3d R_optimized;
				Eigen::Vector3d t_optimized;
				R_optimized << T_optimized(0, 0), T_optimized(0, 1), T_optimized(0, 2), T_optimized(1, 0), T_optimized(1, 1), T_optimized(1, 2),
					T_optimized(2, 0), T_optimized(2, 1), T_optimized(2, 2);
				t_optimized << T_optimized(0, 3), T_optimized(1, 3), T_optimized(2, 3);
				Eigen::Quaterniond quat_optimized = Eigen::Quaterniond(R_optimized);

				geometry_msgs::Pose pose;
				pose.orientation.w = quat_optimized.w();
				pose.orientation.x = quat_optimized.x();
				pose.orientation.y = quat_optimized.y();
				pose.orientation.z = quat_optimized.z();

				pose.position.x = t_optimized(0);
				pose.position.y = t_optimized(1);
				pose.position.z = t_optimized(2);

				std::cout << "优化后的位姿：" << std::endl;
				std::cout << "四元数:\nw:" << pose.orientation.w << "\tx:" << pose.orientation.x
						  << "\ty:" << pose.orientation.y << "\tz:" << pose.orientation.z << std::endl;
				std::cout << "平移向量:\nx:" << pose.position.x << "\ty:" << pose.position.y << "\tz:"
						  << pose.position.z << std::endl
						  << std::endl;

				Pose_pub_.publish(pose);
			}
		}

		track_image_pub_.publish(cv_image.toImageMsg());

		ROI_vector.clear();
		Estimate_Points_Centers.clear();
		event_interval_map_.clear();
	}

	dvs_msgs::Event tracking::getTransition(dvs_msgs::Event event)
	{
		struct event_xy event_key
		{
			event.x, event.y
		};

		// std::cout<<event.x<<"\t"<<event.y<<std::endl;
		if (event_map_.find(event_key) == event_map_.end())
		{
			event_map_[event_key] = event.polarity;
			event.ts = ros::Time(0);
		}

		else
		{
			if (event_map_[event_key] != event.polarity)
			{
				event_map_[event_key] = event.polarity;
			}

			else
			{
				event.ts = ros::Time(0);
			}
		}
		return event;
	}

	// dvs_msgs::Hypertransition tracking::getInterval(dvs_msgs::Event t)
	struct Hypertransition tracking::getInterval(dvs_msgs::Event t)
	{
		struct event_xy event_key
		{
			t.x, t.y
		};

		struct Hypertransition h;
		if (t.polarity == true)
		{
			if (event_positive_map_.find(event_key) == event_positive_map_.end())
			{
				event_positive_map_[event_key] = t.ts.toNSec();
				h.deltaT = 0;
			}
			else
			{
				h.x = t.x;
				h.y = t.y;

				h.deltaT = (t.ts.toNSec() - event_positive_map_[event_key]);

				h.ts = t.ts;

				event_positive_map_[event_key] = t.ts.toNSec();
			}
		}
		else
		{
			if (event_negative_map_.find(event_key) == event_negative_map_.end())
			{
				event_negative_map_[event_key] = t.ts.toNSec();
				h.deltaT = 0;
			}

			else
			{
				h.x = t.x;
				h.y = t.y;
				h.deltaT = (t.ts.toNSec() - event_positive_map_[event_key]);
				h.ts = t.ts;

				event_negative_map_[event_key] = t.ts.toNSec();
			}
		}
		return h;
	}

	bool tracking::if_ROI(struct event_xy e)
	{
		int count = 0;

		struct event_xy event_key
		{
			0, 0
		};

		for (int x = (e.x); x < (e.x + 12); x++)
		{
			for (int y = (e.y - 6); y < (e.y + 6); y++)
			{
				event_key.x = x;
				event_key.y = y;
				if (event_interval_map_.find(event_key) != event_interval_map_.end() && (event_interval_map_[event_key] > 200))
				{
					count++;
				}
			}
		}
		// std::cout << "ROI内事件数： " << count << std::endl;
		if (count >= 10)
			return true;
		else
			return false;
	}

	/* 获取两个点之间的距离 */
	double get_distance(cv::Point2f p1, cv::Point2f p2)
	{
		double distance = sqrt((p1.x - p2.x) * (p1.x - p2.x) +
							   (p1.y - p2.y) * (p1.y - p2.y));
		return distance;
	}

	/* 从旋转矩阵变换到四元数 */
	struct Quaternion rotMatrix2Quaternion(cv::Mat M)
	{
		double t;

		int i = 0, j = 1, k = 2;

		if (M.at<double>(1, 1) > M.at<double>(0, 0))
			i = 1, j = 2, k = 0;

		if (M.at<double>(2, 2) > M.at<double>(i, i))
			i = 2;
		j = 0;
		k = 1;

		t = M.at<double>(i, i) - M.at<double>(j, j) + M.at<double>(k, k) + 1;

		double q[4];
		q[0] = M.at<double>(k, j) - M.at<double>(j, k);
		q[i + 1] = t;
		q[j + 1] = M.at<double>(i, j) + M.at<double>(i, k);
		q[k + 1] = M.at<double>(k, i) + M.at<double>(i, k);

		static struct Quaternion q_;

		q_.w = q[0] * 0.5 / sqrt(t);
		q_.x = q[1] * 0.5 / sqrt(t);
		q_.y = q[2] * 0.5 / sqrt(t);
		q_.z = q[3] * 0.5 / sqrt(t);

		return q_; //q = [qw qx qy qz]
	}

	/* 设置世界坐标点 */
	void set_world_points(std::vector<cv::Point3f> &Points3D)
	{
		Points3D.push_back(cv::Point3f(113, 0, 0));	  //P1 三维坐标的单位是毫米
		Points3D.push_back(cv::Point3f(0, 0, 0));	  //P2
		Points3D.push_back(cv::Point3f(0, 165, 0));	  //P3
		Points3D.push_back(cv::Point3f(230, 165, 0)); //P4
	}

	/* 设置图像座标点 */
	void set_image_points(std::vector<cv::Point2f> &Points2D, std::vector<cv::Point2f> &Points_sorted)
	{
		for (size_t i = 0; i < Points_sorted.size(); i++)
		{
			Points2D.push_back(cv::Point2f(Points_sorted[i].x, Points_sorted[i].y));
		}
	}

	/* 使用BA优化位姿 */
	Eigen::MatrixXd bundleAdjustment(const std::vector<cv::Point3f> points_3d, const std::vector<cv::Point2f> points_2d, const cv::Mat &K, cv::Mat &R, cv::Mat &t)
	{
		// 初始化g2o
		typedef g2o::BlockSolver<g2o::BlockSolverTraits<6, 3>> Block;												  // pose 维度为 6, landmark 维度为 3
		std::unique_ptr<Block::LinearSolverType> linearSolver(new g2o::LinearSolverCSparse<Block::PoseMatrixType>()); // 线性方程求解器
		std::unique_ptr<Block> solver_ptr(new Block(std::move(linearSolver)));										  // 矩阵块求解器
		g2o::OptimizationAlgorithmLevenberg *solver = new g2o::OptimizationAlgorithmLevenberg(std::move(solver_ptr));
		g2o::SparseOptimizer optimizer;
		optimizer.setAlgorithm(solver);

		// vertex
		g2o::VertexSE3Expmap *pose = new g2o::VertexSE3Expmap(); // camera pose
		Eigen::Matrix3d R_mat;
		R_mat << R.at<double>(0, 0), R.at<double>(0, 1), R.at<double>(0, 2), R.at<double>(1, 0), R.at<double>(1, 1), R.at<double>(1, 2),
			R.at<double>(2, 0), R.at<double>(2, 1), R.at<double>(2, 2);
		pose->setId(0);
		pose->setEstimate(g2o::SE3Quat(R_mat, Eigen::Vector3d(t.at<double>(0, 0), t.at<double>(1, 0), t.at<double>(2, 0))));
		optimizer.addVertex(pose);

		int index = 1;
		for (const cv::Point3f p : points_3d) // landmarks
		{
			g2o::VertexSBAPointXYZ *point = new g2o::VertexSBAPointXYZ();
			point->setId(index++);
			point->setEstimate(Eigen::Vector3d(p.x, p.y, p.z));
			point->setMarginalized(true); // g2o 中必须设置 marg 参见第十讲内容
			optimizer.addVertex(point);
		}

		// parameter: camera intrinsics
		g2o::CameraParameters *camera = new g2o::CameraParameters(K.at<double>(0, 0), Eigen::Vector2d(K.at<double>(0, 2), K.at<double>(1, 2)), 0);
		camera->setId(0);
		optimizer.addParameter(camera);

		// edges
		index = 1;
		for (const cv::Point2f p : points_2d)
		{
			g2o::EdgeProjectXYZ2UV *edge = new g2o::EdgeProjectXYZ2UV();
			edge->setId(index);
			edge->setVertex(0, dynamic_cast<g2o::VertexSBAPointXYZ *>(optimizer.vertex(index)));
			edge->setVertex(1, pose);
			edge->setMeasurement(Eigen::Vector2d(p.x, p.y));
			edge->setParameterId(0, 0);
			edge->setInformation(Eigen::Matrix2d::Identity());
			optimizer.addEdge(edge);
			index++;
		}

		// chrono::steady_clock::time_point t1 = chrono::steady_clock::now();
		optimizer.setVerbose(true);
		optimizer.initializeOptimization();
		optimizer.optimize(100);
		// chrono::steady_clock::time_point t2 = chrono::steady_clock::now();
		// chrono::duration<double> time_used = chrono::duration_cast<chrono::duration<double>>(t2 - t1);
		// std::cout<<"optimization costs time: "<<time_used.count() <<" seconds."<<endl;

		// std::cout << "after optimization:" << std::endl;
		// std::cout << "T=\n"
		// 		  << Eigen::Isometry3d(pose->estimate()).matrix() << std::endl;

		return Eigen::Isometry3d(pose->estimate()).matrix();
	}

} // namespace Track
