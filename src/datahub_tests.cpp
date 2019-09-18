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
#include <boost/algorithm/string.hpp>

using json = nlohmann::json;
using namespace std;

class DatahubTests
{
public:
	DatahubTests()
	{
    //team_id_= "gentlebots";
		//team_key_ = "MKRobot2019";
	}

	~DatahubTests()
	{
	}

	void step()
	{
		ROS_INFO("[%s] step", ros::this_node::getName().c_str());
	}

protected:

  ros::NodeHandle nh_;

  ros::Subscriber robot_location_sub_;

  //std::string team_id_;
  //std::string team_key_;

};



int main(int argc, char** argv)
{

  ros::init(argc, argv, "datahub_tests");

  DatahubTests datahub_tests;

  std::cout << "---------------GET MENU-------------------" << std::endl;
  menu m = gb_datahub::getMenu();
/*
	std::cout << m.id << std::endl;
	std::cout << m.type << std::endl;
	for(int i=0; i< m.products.size(); i++){
		std::cout << m.products[i].id << std::endl;
	}
	*/
  std::cout << "--------------END GET MENU---------------" << std::endl;

	std::cout << "---------------POST TABLE-------------------" << std::endl;
	std::string post_ = "  {\n    \"@id\": \"TABLE1\",\n    \"@type\": \"Table\",\n    \"customers\": 2,\n    \"status\": \"Ready\"\n  }";

	table t;
	t.id = "TABLE78";
	t.type = "Table";
	t.customers = 2;
	t.status = "Already served";

	int v = gb_datahub::postTable(t);
	//std::cout << v << std::endl;
	std::cout << v << std::endl;

  std::cout << "--------------END POST TABLE---------------" << std::endl;

/*
	std::cout << "---------------PUT TABLE-------------------" << std::endl;
	std::string put_ = "  {\n    \"@id\": \"TABLE0\",\n    \"@type\": \"Table\",\n    \"customers\": 2,\n    \"status\": \"Ready\"\n  }";
	gb_datahub::putTable(t);
	std::cout << "-------------json[0]["@id"]-END PUT TABLE---------------" << std::endl;

	std::cout << "---------------DELETE TABLE-------------------" << std::endl;
	gb_datahub::deleteTable(t.id);
	std::cout << "--------------END DELETE TABLE---------------" << std::endl;
*/


///test table
//std::string post_ = "  {\n    \"@id\": \"TABLE1\",\n    \"@type\": \"Table\",\n    \"customers\": 2,\n    \"status\": \"Ready\"\n  }";
//gb_datahub::tableJsonToObject(post_);


	std::cout <<"---------------POST ORDER-------------------" << std::endl;

	std::string or_ = "{\"@id\": \"ORDER0\",    \"@type\": \"Order\",    \"table\": \"Table1\",    \"timestamp\": \"2019-09-03T08:06:04.818Z\",  \"products\": [ \"coke\", \"fanta\" ],   \"status\": \"Pending\" }";

	order o = gb_datahub::orderJsonToObject(or_);
	/*
	std::vector<std::string> vec;
	vec.push_back("coke");
	vec.push_back("fanta");
	order o;
	o.id = "Order1";
	o.type = "Order";
	o.table = "table1";
	o.timestamp ="2019-09-03T08:06:04.818Z";
	o.products = vec;
	o.status = "pending";
	*/

	int a = gb_datahub::postOrder(o);

	std::cout << a << std::endl;

	json j2 = gb_datahub::orderToJson(o);

	//std::cout << j2.dump(4) << std::endl;
	//std::cout <<"______" << std::endl;

	std::cout <<"---------------END ORDER-------------------" << std::endl;
	gb_datahub::getOrder("ORDER0");
	//std::cout << gb_datahub::getOrder("ORDER0") << std::endl;

	std::cout <<"---------------END getORDER-------------------" << std::endl;


	std::cout << "---------------GET SHOP LIST-------------------" << std::endl;
	//std::vector<shop> sh = gb_datahub::getShopsList();
/*
	std::vector<shop> shops = gb_datahub::getShopsList();
	for (auto shop : shops)
	{
		if (shop.goal == true)
			//target_floor_ = car2ord(shop.floor);
			std::cout << shop.floor << std::endl;
	}
*/


	std::cout << "--------------END GET SHOP LIST---------------" << std::endl;

	robotLocation robotLocation_;
	robotLocation_.id = "ROBOTLOCATION00";
	robotLocation_.type = "RobotLocation";
	robotLocation_.episode = "EPISODE3";
	robotLocation_.team = "gentlebots";
	robotLocation_.timestamp = "2019-09-17T15:07:07.520Z";
														//2019-09-18T10:33:08.400779
	robotLocation_.x = 0.00;
	robotLocation_.y = 1.00;
	robotLocation_.z = 2.00;
/*
	std::string token = robotLocation_.timestamp.substr(11, robotLocation_.timestamp.find(":"));
	std::cout << token << std::endl;
	std::string token2 = token.substr(0, token.find(":"));
	std::cout << token2 << std::endl;

	std::string token3 = token.substr(13, token.find(":"));
	std::cout << token2 << std::endl;
*/

	std::vector<std::string> results;
	std::vector<std::string> results_hour;


	boost::split(results, robotLocation_.timestamp, [](char c){return c == ':';});

	for(int i=0; i< results.size(); i++){
		std::cout << results[i] << std::endl;

	}

	boost::split(results_hour, results[0], [](char c){return c == 'T';});

	std::cout << results_hour[1] << std::endl;

	std::cout << atoi(results_hour[1].c_str())+1 << std::endl;
	std::string gmt = results_hour[0] + "T"+ std::to_string(atoi(results_hour[1].c_str())+1) +":" + results[1] + ":" + results[2];
	std::cout << gmt << std::endl;
	//std::cout << gb_datahub::postRobotLocation(robotLocation_) << std::endl;


		 robotStatus robotstatus_;
		 robotstatus_.id = "ROBOTSTATUS00";
		 robotstatus_.type = "RobotStatus";
		 robotstatus_.message = "Serving...";
		 robotstatus_.episode = "EPISODE3";
		 robotstatus_.team = "gentlebots";
		 robotstatus_.timestamp = "2019-09-17T15:07:07.520Z";
		 													//2019-09-18T10:33:08.400779
		 robotstatus_.x = 0.00;
		 robotstatus_.y = 1.00;
		 robotstatus_.z = 2.00;
		 //cout << "RobotStatus init"<< dt << endl;
		// std::cout << gb_datahub::postRobotStatus(robotstatus_) << std::endl;
		 //cout << "RobotStatus end"<< dt << endl;

	return 0;

}

/*
Text [{"_id":"menu0\/menu","@id":"MENU0","@type":"Menu","products":[{"@id":"PR001","@type":"Product","label":"espresso","descriptions":"espresso in a small cup","price":"\u00a31.50"},{"@id":"PR002","@type":"Product","label":"cappuccino","descriptions":"cappuccino in a medium cup","price":"\u00a32.40"},{"@id":"PR003","@type":"Product","label":"flat white","descriptions":"flat white in a big cup","price":"\u00a32.80"},{"@id":"PR004","@type":"Product","label":"butter croissant","descriptions":"butter croissant with no packaging","price":"\u00a31.50"},{"@id":"PR005","@type":"Product","label":"biscotti","descriptions":"italian biscotti in a transparent packaging","price":"\u00a32.00"}],"_datasetid":"5a05b34b-6772-4583-8d1a-427d2c72e330","_timestamp":1565974108,"_timestamp_year":2019,"_timestamp_month":8,"_timestamp_day":16,"_timestamp_hour":17,"_timestamp_minute":48,"_timestamp_second":28,"_updated":true}]

[
    {
        "@id": "MENU0",
        "@type": "Menu",
        "_datasetid": "5a05b34b-6772-4583-8d1a-427d2c72e330",
        "_id": "menu0/menu",
        "_timestamp": 1565974108,
        "_timestamp_day": 16,
        "_timestamp_hour": 17,
        "_timestamp_minute": 48,
        "_timestamp_month": 8,
        "_timestamp_second": 28,
        "_timestamp_year": 2019,
        "_updated": true,
        "products": [
            {
                "@id": "PR001",
                "@type": "Product",
                "descriptions": "espresso in a small cup",
                "label": "espresso",
                "price": "£1.50"
            },
            {
                "@id": "PR002",
                "@type": "Product",
                "descriptions": "cappuccino in a medium cup",
                "label": "cappuccino",
                "price": "£2.40"
            },
            {
                "@id": "PR003",
                "@type": "Product",
                "descriptions": "flat white in a big cup",
                "label": "flat white",
                "price": "£2.80"
            },
            {
                "@id": "PR004",
                "@type": "Product",
                "descriptions": "butter croissant with no packaging",
                "label": "butter croissant",
                "price": "£1.50"
            },
            {
                "@id": "PR005",
                "@type": "Product",
                "descriptions": "italian biscotti in a transparent packaging",
                "label": "biscotti",
                "price": "£2.00"
            }
        ]
    }
]

*/


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
