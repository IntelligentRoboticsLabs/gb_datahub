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

#include <gb_datahub/gb_datahub.h>


class CoffeeShopDelivery
{
public:
	CoffeeShopDelivery()
	{
    //nh_();
    team_id_= "gentlebots";
    team_key_ = "ea7bfa2e-77e3-4948-80b6-5b84af77a4b2";
	}

	~CoffeeShopDelivery()
	{
	}

	void step()
	{
		ROS_INFO("[%s] step", ros::this_node::getName().c_str());
	}

protected:

  ros::NodeHandle nh_;

  ros::Subscriber robot_location_sub_;

  std::string team_id_;
  std::string team_key_;

};



int main(int argc, char** argv)
{

  ros::init(argc, argv, "coffee_shop_delivery");

  CoffeeShopDelivery coffee_shop_delivery;

  std::cout << "---------------GET-------------------" << std::endl;
  gb_datahub::getMenu();
  std::cout << "--------------END GET---------------" << std::endl;

  /*
	ros::init(argc, argv, "coffee_shop_delivery");

	CoffeeShopDelivery coffee_shop_delivery;

	ros::Rate loop_rate(1);
	while(coffee_shop_delivery.ok())
	{
		coffee_shop_delivery.step();

		ros::spinOnce();
		loop_rate.sleep();
	}

*/
	return 0;

}
