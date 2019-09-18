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
	}

	~CoffeeShopDelivery()
	{
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
			robotLocation_.id = std::to_string(seq++);
			robotLocation_.type = "RobotLocation";
			robotLocation_.episode = "EPISODE3";
			robotLocation_.team = team_id_;
			robotLocation_.timestamp = boost::posix_time::to_iso_extended_string(ps_.header.stamp.toBoost());
			robotLocation_.x = ps_.pose.position.x;
			robotLocation_.y = ps_.pose.position.y;
			robotLocation_.z = ps_.pose.position.z;

      /*ROS_INFO("[coffee_shop_delivery] robotLocation_.id %s", robotLocation_.id.c_str());
      ROS_INFO("[coffee_shop_delivery] robotLocation_.type %s", robotLocation_.type.c_str());
      ROS_INFO("[coffee_shop_delivery] robotLocation_.episode %s", robotLocation_.episode.c_str());
      ROS_INFO("[coffee_shop_delivery] robotLocation_.team %s", robotLocation_.team.c_str());
      ROS_INFO("[coffee_shop_delivery] robotLocation_.timestamp %s", robotLocation_.timestamp.c_str());
      ROS_INFO("[coffee_shop_delivery] robotLocation_.x %i", robotLocation_.x);
      ROS_INFO("[coffee_shop_delivery] robotLocation_.y %i", robotLocation_.y);
      ROS_INFO("[coffee_shop_delivery] robotLocation_.z %i", robotLocation_.z);

			ROS_INFO("[coffee_shop_delivery] postStatus %i", gb_datahub::postRobotLocation(robotLocation_));*/

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
		poseCallback(bf2map);



    auto interest_edges = graph_.get_string_edges_from_node_by_data("sonny", "robot_status: [[:alnum:]_]*");
    if (!interest_edges.empty())
    {
      ROS_INFO("robot_status: %s", interest_edges[0].get().c_str());
    }
	}

protected:

	ros::NodeHandle nh_;
	ros::ServiceServer srv_;
  ros::Subscriber robot_location_sub_;
	ros::Subscriber tf_sub_;
	tf::TransformListener tf_listener_;
	std::string team_id_;
	std::string team_key_ ;
  bica_graph::GraphClient graph_;
  int seq;

};


int main(int argc, char** argv)
{
  ros::init(argc, argv, "coffee_shop_delivery");

  CoffeeShopDelivery coffee_shop_delivery;

	ros::Rate loop_rate(1);

	//coffee_shop_delivery.getMenu():

  while (ros::ok())
  {
    coffee_shop_delivery.step();

    ros::spinOnce();
  	loop_rate.sleep();
  }

	return 0;

}
