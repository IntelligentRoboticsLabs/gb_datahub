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

class TakeTheElevator
{
public:
	TakeTheElevator()
	{
    team_id_= "gentlebots";
    team_key_ = "ea7bfa2e-77e3-4948-80b6-5b84af77a4b2";
	}

	~TakeTheElevator()
	{
	}

	void step()
	{
		ROS_INFO("[%s] step", ros::this_node::getName().c_str());
	}

protected:

  std::string team_id_;
  std::string team_key_;


};


int main(int argc, char** argv)
{
	ros::init(argc, argv, "take_the_elevator");

	TakeTheElevator take_the_elevator;
/*
	ros::Rate loop_rate(1);

	while(take_the_elevator.ok())
	{
		take_the_elevator.step();

		ros::spinOnce();
		loop_rate.sleep();
	}

*/
	std::cout << "---------------GET SHOP LIST-------------------" << std::endl;
	std::string info = gb_datahub::getShopList();
	gb_datahub::prettyJson(info);
	gb_datahub::shopJsonToObject(info);
	//gb_datahub::prettyJson(info);
	std::cout << "--------------END GET SHOP LIST---------------" << std::endl;

	return 0;
}

/*

[{"@id":"SHOP00","@type":"Shop","_datasetid":"5a05b34b-6772-4583-8d1a-427d2c72e330","_id":"shop00/shop","_timestamp":1565974816,"_timestamp_day":16,"_timestamp_hour":18,"_timestamp_minute":0,"_timestamp_month":8,"_timestamp_second":16,"_timestamp_year":2019,"_updated":true,"description":"Tesco","floor":1,"goal":false},{"@id":"SHOP01","@type":"Shop","_datasetid":"5a05b34b-6772-4583-8d1a-427d2c72e330","_id":"shop01/shop","_timestamp":1565974816,"_timestamp_day":16,"_timestamp_hour":18,"_timestamp_minute":0,"_timestamp_month":8,"_timestamp_second":16,"_timestamp_year":2019,"_updated":true,"description":"Waitrose","floor":2,"goal":false},{"@id":"SHOP02","@type":"Shop","_datasetid":"5a05b34b-6772-4583-8d1a-427d2c72e330","_id":"shop02/shop","_timestamp":1565974816,"_timestamp_day":16,"_timestamp_hour":18,"_timestamp_minute":0,"_timestamp_month":8,"_timestamp_second":16,"_timestamp_year":2019,"_updated":true,"description":"Marks and Spencer","floor":3,"goal":false},{"@id":"SHOP03","@type":"Shop","_datasetid":"5a05b34b-6772-4583-8d1a-427d2c72e330","_id":"shop03/shop","_timestamp":1565974816,"_timestamp_day":16,"_timestamp_hour":18,"_timestamp_minute":0,"_timestamp_month":8,"_timestamp_second":16,"_timestamp_year":2019,"_updated":true,"description":"Costa","floor":4,"goal":true}]
[
    {
        "@id": "SHOP00",
        "@type": "Shop",
        "_datasetid": "5a05b34b-6772-4583-8d1a-427d2c72e330",
        "_id": "shop00/shop",
        "_timestamp": 1565974816,
        "_timestamp_day": 16,
        "_timestamp_hour": 18,
        "_timestamp_minute": 0,
        "_timestamp_month": 8,
        "_timestamp_second": 16,
        "_timestamp_year": 2019,
        "_updated": true,
        "description": "Tesco",
        "floor": 1,
        "goal": false
    },
    {
        "@id": "SHOP01",
        "@type": "Shop",
        "_datasetid": "5a05b34b-6772-4583-8d1a-427d2c72e330",
        "_id": "shop01/shop",
        "_timestamp": 1565974816,
        "_timestamp_day": 16,
        "_timestamp_hour": 18,
        "_timestamp_minute": 0,
        "_timestamp_month": 8,
        "_timestamp_second": 16,
        "_timestamp_year": 2019,
        "_updated": true,
        "description": "Waitrose",
        "floor": 2,
        "goal": false
    },
    {
        "@id": "SHOP02",
        "@type": "Shop",
        "_datasetid": "5a05b34b-6772-4583-8d1a-427d2c72e330",
        "_id": "shop02/shop",
        "_timestamp": 1565974816,
        "_timestamp_day": 16,
        "_timestamp_hour": 18,
        "_timestamp_minute": 0,
        "_timestamp_month": 8,
        "_timestamp_second": 16,
        "_timestamp_year": 2019,
        "_updated": true,
        "description": "Marks and Spencer",
        "floor": 3,
        "goal": false
    },
    {
        "@id": "SHOP03",
        "@type": "Shop",
        "_datasetid": "5a05b34b-6772-4583-8d1a-427d2c72e330",
        "_id": "shop03/shop",
        "_timestamp": 1565974816,
        "_timestamp_day": 16,
        "_timestamp_hour": 18,
        "_timestamp_minute": 0,
        "_timestamp_month": 8,
        "_timestamp_second": 16,
        "_timestamp_year": 2019,
        "_updated": true,
        "description": "Costa",
        "floor": 4,
        "goal": true
    }
]

*/
