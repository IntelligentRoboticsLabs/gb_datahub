/*********************************************************************
*  Software License Agreement (BSD License)
*
*   Copyright (c) 2019, Intelligent Robotics
*   All rights reserved.
*
*   Redistribution and use in source and binary forms, with or without
*   modification, are permitted provided that the following conditions
*   are met:

*    * Redistributions of source code must retain the above copyright
*      notice, this list of conditions and the following disclaimer.
*    * Redistributions in binary form must reproduce the above
*      copyright notice, this list of conditions and the following
*      disclaimer in the documentation and/or other materials provided
*      with the distribution.
*    * Neither the name of Intelligent Robotics nor the names of its
*      contributors may be used to endorse or promote products derived
*      from this software without specific prior written permission.

*   THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*   "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*   LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*   FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*   COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*   INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*   BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*   LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*   CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*   LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*   ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*   POSSIBILITY OF SUCH DAMAGE.
*********************************************************************/

/* Author: Lorena Bajo Rebollo lbajo9@gmail.com */

#include "ros/ros.h"
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <tf/tfMessage.h>
#include <geometry_msgs/PoseStamped.h>
#include "boost/date_time/posix_time/posix_time.hpp"
#include <bica_graph/graph_client.h>

#include <string>
#include <list>
#include <vector>
#include <set>

#include <gb_datahub/gb_datahub.h>
#include <gb_datahub/GetMenu.h>
#include <gb_datahub/Menu.h>
#include <gb_datahub/Product.h>
#include <boost/algorithm/string.hpp>
#include "geometry_msgs/PoseWithCovarianceStamped.h"

using json = nlohmann::json;

class CoffeeShopDelivery
{
public:
	CoffeeShopDelivery()
	{
		srv_ = nh_.advertiseService("/gb_datahub/menu", &CoffeeShopDelivery::getMenu, this);
		team_id_= "gentlebots";
		team_key_ = "ea7bfa2e-77e3-4948-80b6-5b84af77a4b2";
    seq = 0;
		last_status_ = "";

		amcl_pose_ = nh_.subscribe("/amcl_pose", 1, &CoffeeShopDelivery::poseAMCLCallback, this);
	}

	~CoffeeShopDelivery()
	{
	}

	void poseAMCLCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msgAMCL)
{

		robotLocation robotLocation_;
		robotLocation_.id = "SonnyLocation";
		robotLocation_.type = "RobotLocation";
		robotLocation_.episode = "EPISODE3";
		robotLocation_.team = team_id_;
		robotLocation_.timestamp = magicHour(boost::posix_time::to_iso_extended_string(ros::Time::now().toBoost()));
		robotLocation_.x = msgAMCL->pose.pose.position.x;
		robotLocation_.y = msgAMCL->pose.pose.position.y;
		robotLocation_.z = msgAMCL->pose.pose.orientation.w;

		gb_datahub::postRobotLocation(robotLocation_);
    //ROS_INFO(msgAMCL);

		//////////////////ROBOTSTATUS////////////////////////////

		auto interest_edges = graph_.get_string_edges_from_node("sonny");
		std::string status_;
		for (int i=0; i< interest_edges.size(); i++){
			if (interest_edges[i].get().find("robot_status") != std::string::npos){
				std::cout << interest_edges[i].get().c_str() << std::endl;
				std::cout << "------------------------" << std::endl;

				std::string delimiter = ":";
				std::string response_raw = interest_edges[i].get().c_str();
				response_raw.erase(0, response_raw.find(delimiter) + delimiter.length());
				status_ = response_raw;

				if (last_status_ != status_){
					last_status_ = status_;

					robotStatus robotStatus_;
					robotStatus_.id =  std::to_string(seq++);
					robotStatus_.type = "RobotStatus";
					robotStatus_.message =  status_;
					robotStatus_.episode = "EPISODE3";
					robotStatus_.team = "gentlebots";
					robotStatus_.timestamp = magicHour(boost::posix_time::to_iso_extended_string(ros::Time::now().toBoost()));
					robotStatus_.x = msgAMCL->pose.pose.position.x;
					robotStatus_.y = msgAMCL->pose.pose.position.y;
					robotStatus_.z = msgAMCL->pose.pose.orientation.w;

					gb_datahub::postRobotStatus(robotStatus_);

					graph_.remove_edge(interest_edges[i]);
				}
			}
		}
}

	std::string magicHour(std::string text)
	{
		std::vector<std::string> results;
		std::vector<std::string> results_hour;

		boost::split(results, text, [](char c){return c == ':';});
		boost::split(results_hour, results[0], [](char c){return c == 'T';});
		std::string gmt = results_hour[0] + "T"+ std::to_string(atoi(results_hour[1].c_str())+1) +":" + results[1] + ":" + results[2];

		return gmt;
	}

	geometry_msgs::PoseStamped stampedTransform2poseStamped(tf::StampedTransform bf2map)
	{
		geometry_msgs::PoseStamped ps_;
		geometry_msgs::Quaternion quat_;
		geometry_msgs::Point point_;
		tf::Quaternion tf_quat_;
		tf::Vector3 vect_;

		tf_quat_ = bf2map.getRotation();
		quat_.x = tf_quat_.x();
		quat_.y = tf_quat_.y();
		quat_.z = tf_quat_.z();
		quat_.w = tf_quat_.w();
		ps_.pose.orientation = quat_;

		vect_ = bf2map.getOrigin();
		point_.x = vect_.getX();
		point_.y = vect_.getY();
		point_.z = vect_.getZ();
		ps_.pose.position = point_;
		ps_.header.frame_id = bf2map.frame_id_;
		ps_.header.stamp = bf2map.stamp_;

		return ps_;
	}

	void poseCallback(tf::StampedTransform& bf2map)
	{
		geometry_msgs::PoseStamped ps_;

		try
		{
			tf_listener_.lookupTransform("/base_footprint", "/map", ros::Time(0), bf2map);

			ps_ = stampedTransform2poseStamped(bf2map);

				//pose_pub_.publish(ps_);
			robotLocation robotLocation_;
			robotLocation_.id = "SonnyLocation";
			robotLocation_.type = "RobotLocation";
			robotLocation_.episode = "EPISODE3";
			robotLocation_.team = team_id_;
			robotLocation_.timestamp = magicHour(boost::posix_time::to_iso_extended_string(ps_.header.stamp.toBoost()));
			robotLocation_.x = ps_.pose.position.x;
			robotLocation_.y = ps_.pose.position.y;
			robotLocation_.z = ps_.pose.position.z;

			gb_datahub::postRobotLocation(robotLocation_);

			auto interest_edges = graph_.get_string_edges_from_node("sonny");
			std::string status_;
			for (int i=0; i< interest_edges.size(); i++){
				if (interest_edges[i].get().find("robot_status") != std::string::npos){
						std::cout << "------------------------" << std::endl;
						std::cout << interest_edges[i].get().c_str() << std::endl;
						std::cout << "------------------------" << std::endl;

						std::string delimiter = ":";
						std::string response_raw = interest_edges[i].get().c_str();
						response_raw.erase(0, response_raw.find(delimiter) + delimiter.length());
						status_ = response_raw;

					if (last_status_ != status_)
					{
						//ROS_INFO("robot_status: %s", status_);
						last_status_ = status_;

						robotStatus robotStatus_;
						robotStatus_.id =  std::to_string(seq++);
						robotStatus_.type = "RobotStatus";
						robotStatus_.message =  status_;
						robotStatus_.episode = "EPISODE3";
						robotStatus_.team = "gentlebots";
						robotStatus_.timestamp = magicHour(boost::posix_time::to_iso_extended_string(ps_.header.stamp.toBoost()));
																		 //2019-09-18T10:33:08.400779
						robotStatus_.x = ps_.pose.position.x;
						robotStatus_.y = ps_.pose.position.y;
						robotStatus_.z = ps_.pose.position.z;

						gb_datahub::postRobotStatus(robotStatus_);

						graph_.remove_edge(interest_edges[i]);
					}
				}
			}

		}
		catch (tf::TransformException& ex)
		{
			ROS_ERROR("%s",ex.what());
		}
	}


	bool getMenu(gb_datahub::GetMenu::Request &req, gb_datahub::GetMenu::Response &res)
	{
		menu menu_ = gb_datahub::getMenu();

		gb_datahub::Menu m_;

		m_.id = menu_.id ;
		m_.type = menu_.type;

		for(int i=0; i< menu_.products.size(); i++)
		{
			gb_datahub::Product pr_;

			pr_.id = menu_.products[i].id;
			pr_.type = menu_.products[i].type;
			pr_.label = menu_.products[i].label;
			pr_.descriptions = menu_.products[i].descriptions;
			pr_.price = menu_.products[i].price;

			m_.products.push_back(pr_);
		}

		res.menu = m_;

		return true;
	}



	void step()
	{
		tf::StampedTransform bf2map;
		//poseCallback(bf2map);
		//poseAMCLCallback();
	}

protected:

	ros::NodeHandle nh_;
	ros::ServiceServer srv_;
  ros::Subscriber robot_location_sub_, amcl_pose_;
	ros::Subscriber tf_sub_;
	tf::TransformListener tf_listener_;
	std::string team_id_;
	std::string team_key_ ;
  bica_graph::GraphClient graph_;
  int seq;
	std::string last_status_;

};


int main(int argc, char** argv)
{
  ros::init(argc, argv, "coffee_shop_delivery");

  CoffeeShopDelivery coffee_shop_delivery;

	ros::Rate loop_rate(1);

  while (ros::ok())
  {
    coffee_shop_delivery.step();

    ros::spinOnce();
  	loop_rate.sleep();
  }

	return 0;

}
