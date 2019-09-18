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

#ifndef GB_DATAHUB_H
#define GB_DATAHUB_H

#include <ros/ros.h>
//#include <bica_graph/graph_client.h>


#include <cpr/cpr.h>

#include <nlohmann/json.hpp>
#include <string>
#include <list>
#include <set>

using json = nlohmann::json;

struct robotStatus {
  std::string id;
  std::string type;
  std::string message;
  std::string episode;
  std::string team;
  std::string timestamp;
  int x;
  int y;
  int z;
};

struct robotLocation {
  std::string id;
  std::string type;
  std::string episode;
  std::string team;
  std::string timestamp;
  int x;
  int y;
  int z;
};

struct table {
  std::string id;
  std::string type;
  int customers;
  std::string status;
};

struct product {
  std::string id;
  std::string type;
  std::string label;
  std::string descriptions;
  std::string price;
};

struct order {
  std::string id;
  std::string type;
  std::string table;
  std::string timestamp;
  std::vector<std::string> products;
  std::string status;
};

struct menu {
  std::string id;
  std::string type;
  std::vector<product> products;
};

struct shop {
  std::string id;
  std::string type;
  int floor;
  std::string description;
  bool goal;
};


namespace gb_datahub
{

  std::vector<robotStatus> getRobotStatusList();
  robotStatus getRobotStatus(std::string id);
  int putRobotStatus(robotStatus robotStatus_);
  int postRobotStatus(robotStatus robotStatus_);

  std::vector<robotLocation> getRobotLocationList();
  robotLocation getRobotLocation(std::string id);
  int putRobotLocation(robotLocation robotLocation_);
  int postRobotLocation(robotLocation robotLocation_);

// Episode3
  menu getMenu();
  std::vector<table> getTableList();
  table getTable(std::string id);
  int putTable(table table_);
  int postTable(table table_);
  int deleteTable(std::string id);
  std::vector<order> getOrderList();
  std::vector<order> getOrder(std::string id);
  int putOrder(order order_);
  int postOrder(order order_);
  int deleteOrder(std::string id);

// Episode4

  std::vector<shop> getShopsList();

// JSON
  std::vector<robotStatus> robotStatusListJsonToObject(std::string info);
  robotStatus robotStatusJsonToObject(std::string info);
  std::vector<robotLocation> robotLocationListJsonToObject(std::string info);
  robotLocation robotLocationJsonToObject(std::string info);
  std::vector<table> tableListJsonToObject(std::string info);
  table tableJsonToObject(std::string info);
  product productTextToObject(std::string info);
  product productJsonToObject(json json);
  order orderJsonToObject(std::string info);
  std::vector<order> orderListJsonToObject(std::string info);
  menu menuJsonToObject(std::string info);
  std::vector<shop> shopJsonToObject(std::string info);
  void prettyJson(std::string info);

  json robotStatusToJson(robotStatus robotStatus_);
  json robotLocationToJson(robotLocation robotLocation_);
  json tableToJson(table table_);
  json productToJson(product product_);
  json orderToJson(order order_);

  std::string magicHour(std::string text);


  //std::string team_id_= "gentlebots";
  //std::string team_key_ = "ea7bfa2e-77e3-4948-80b6-5b84af77a4b2";

}

#endif  // GB_DATAHUB_H
