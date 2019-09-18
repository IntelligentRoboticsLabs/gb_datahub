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

#include <ros/ros.h>

#include <string>
#include <list>
#include <vector>
#include <set>

#include "ros/ros.h"
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <tf/tfMessage.h>
#include <geometry_msgs/PoseStamped.h>
#include "boost/date_time/posix_time/posix_time.hpp"

#include <gb_datahub/gb_datahub.h>
#include <gb_datahub/GetShopsList.h>
#include <gb_datahub/Shop.h>
#include <bica_graph/graph_client.h>
#include <boost/algorithm/string.hpp>


class TakeTheElevator
{
public:
	TakeTheElevator()
	{
		srv_ = nh_.advertiseService("/gb_datahub/shops", &TakeTheElevator::getShopsList, this);
		//tf_sub_ = nh_.subscribe("/tf", 1, &TakeTheElevator::tfCallback, this);
		team_id_= "gentlebots";
		team_key_ = "ea7bfa2e-77e3-4948-80b6-5b84af77a4b2";
    seq = 0;
		last_status_ = "";
	}

	~TakeTheElevator()
	{
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
			robotLocation_.id = "Sonny Location";
			robotLocation_.type = "RobotLocation";
			robotLocation_.episode = "EPISODE4";
			robotLocation_.team = team_id_;
			robotLocation_.timestamp = magicHour(boost::posix_time::to_iso_extended_string(ps_.header.stamp.toBoost()));
			robotLocation_.x = ps_.pose.position.x;
			robotLocation_.y = ps_.pose.position.y;
			robotLocation_.z = ps_.pose.position.z;

			gb_datahub::postRobotLocation(robotLocation_);

			auto interest_edges = graph_.get_string_edges_from_node_by_data("sonny", "robot_status: [[:alnum:]_]*");
			if (!interest_edges.empty() && last_status_ != interest_edges[0].get().c_str())
			{
				ROS_INFO("robot_status: %s", interest_edges[0].get().c_str());
				last_status_ = interest_edges[0].get().c_str();

				robotStatus robotStatus_;
				robotStatus_.id =  std::to_string(seq++);
				robotStatus_.type = "RobotStatus";
				robotStatus_.message =  interest_edges[0].get().c_str();
				robotStatus_.episode = "EPISODE4";
				robotStatus_.team = "gentlebots";
				robotStatus_.timestamp = magicHour(boost::posix_time::to_iso_extended_string(ps_.header.stamp.toBoost()));
																 //2019-09-18T10:33:08.400779
				robotStatus_.x = ps_.pose.position.x;
				robotStatus_.y = ps_.pose.position.y;
				robotStatus_.z = ps_.pose.position.z;

				gb_datahub::postRobotStatus(robotStatus_);

				graph_.remove_edge(interest_edges[0]);
			}

		}
		catch (tf::TransformException& ex)
		{
			ROS_ERROR("%s",ex.what());
		}
	}

	void robotLocationUpdate()
	{
		ROS_INFO("[%s] step", ros::this_node::getName().c_str());
	}

	bool getShopsList(gb_datahub::GetShopsList::Request &req, gb_datahub::GetShopsList::Response &res)
	{
		std::vector<shop> shops = gb_datahub::getShopsList();

		std::cout << shops.size() << std::endl;

		for(int i=0; i< shops.size(); i++)
		{
			gb_datahub::Shop shop_;
			shop_.id = shops[i].id ;
			shop_.type = shops[i].type;
			shop_.floor = shops[i].floor;
			shop_.description = shops[i].description;
			shop_.goal = shops[i].goal;
			res.shops.push_back(shop_);
		}

		return true;
	}

	void prueba(){
		std::vector<shop> shops = gb_datahub::getShopsList();

		std::cout << shops.size() << std::endl;

				for(int i=0; i< shops.size(); i++)
				{
					gb_datahub::Shop shop_;
					shop_.id = shops[i].id ;
					shop_.type = shops[i].type;
					shop_.floor = shops[i].floor;
					shop_.description = shops[i].description;
					shop_.goal = shops[i].goal;
					//res.shops.push_back(shop_);
				}
	}

	void step()
	{
		 tf::StampedTransform bf2map;
		 poseCallback(bf2map);
	 }

protected:

	ros::NodeHandle nh_;
	ros::ServiceServer srv_;
  ros::Subscriber robot_location_sub_;
	ros::Subscriber tf_sub_;
	tf::TransformListener tf_listener_;
	std::string team_id_;
	std::string team_key_;
  int seq;
	bica_graph::GraphClient graph_;
	std::string last_status_;

};


int main(int argc, char** argv)
{
	ros::init(argc, argv, "take_the_elevator");

	TakeTheElevator take_the_elevator;

	ros::Rate loop_rate(1);

	while (ros::ok())
  {
    take_the_elevator.step();
    ros::spinOnce();
    loop_rate.sleep();
  }
	return 0;
}
