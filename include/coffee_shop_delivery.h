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

/* Author:Lorena Bajo Rebollo lbajo9@gmail.com */

#ifndef GB_DATAHUB_COFFEE_SHOP_DELIVERY_H
#define GB_DATAHUB_COFFEE_SHOP_DELIVERY_H

#include <ros/ros.h>

#include <bica_graph/graph_client.h>
//#include <cpprest/http_client.h>
#include <boost/network/protocol/http/client.hpp>

#include <string>
#include <list>
#include <set>

namespace gb_datahub
{

class CoffeeShopDelivery
{
public:

	void update();

protected:

	ros::NodeHandle nh_;
	bica_graph::GraphClient graph_;

  ros::Subscriber robot_location_sub_;

/*
  std::string working_frame_;
  ros::Publisher attention_points_pub_;
	ros::Publisher markers_pub_;
	ros::ServiceClient remove_instance_service_;
	std::string class_;

	std::set<std::string> instances_sent_;

  */

  std::string team_id_;
  std::string team_key_;

};

};

#endif  // GB_DATAHUB_COFFEE_SHOP_DELIVERY_H
