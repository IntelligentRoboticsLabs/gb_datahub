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
#include <typeinfo>

using json = nlohmann::json;

namespace gb_datahub
{

  std::vector<robotStatus> getRobotStatusList()
  {
    std::string url_ = "https://api.mksmart.org/sciroc-competition/"+team_id_+"/sciroc-robot-status";

    cpr::Session session;
    session.SetVerifySsl(false);
    session.SetUrl(url_);
    session.SetAuth(cpr::Authentication{team_key_, ""});
    auto response_ = session.Get();

    //prettyJson(response_.text);

    return robotStatusListJsonToObject(response_.text);
  }

  robotStatus getRobotStatus(std::string id)
  {
    std::string url_ = "https://api.mksmart.org/sciroc-competition/"+team_id_+"/sciroc-robot-status/"+ id;

    cpr::Session session;
    session.SetVerifySsl(false);
    session.SetUrl(url_);
    session.SetAuth(cpr::Authentication{team_key_, ""});
    auto response_ = session.Get();

    //prettyJson(response_.text);

    return robotStatusJsonToObject(response_.text);
  }

  int putRobotStatus(robotStatus robotStatus_)
  {
    std::string url_ = "https://api.mksmart.org/sciroc-competition/"+team_id_+"/sciroc-robot-status/"+ robotStatus_.id;
    //std::string payload = "  {\n    \"@id\": \"xx\",\n    \"@type\": \"xx\",\n    \"message\": xx,\n    \"episode\": \"xx\"\n  \"team\": \"xx\"\n  \"timestamp\": \"xx\"\n  \"x\": \"xx\"\n  \"y\": \"xx\"\n  \"z\": \"xx\"\n  }";

    json j = robotStatusToJson(robotStatus_);
    std::string info = j.dump();

    cpr::Session session;
    session.SetVerifySsl(false);
    session.SetUrl(url_);
    session.SetBody(cpr::Body{info});
    session.SetAuth(cpr::Authentication{team_key_, ""});
    auto response_ = session.Put();

    return response_.status_code;
  }

  int postRobotStatus(robotStatus robotStatus_)
  {
    std::string url_ = "https://api.mksmart.org/sciroc-competition/"+team_id_+"/sciroc-robot-status/"+ robotStatus_.id;

    //std::string payload = "  {\n    \"@id\": \"xx\",\n    \"@type\": \"xx\",\n    \"message\": xx,\n    \"episode\": \"xx\"\n  \"team\": \"xx\"\n  \"timestamp\": \"xx\"\n  \"x\": \"xx\"\n  \"y\": \"xx\"\n  \"z\": \"xx\"\n  }";

    json j = robotStatusToJson(robotStatus_);
    std::string info = j.dump();

    cpr::Session session;
    session.SetVerifySsl(false);
    session.SetUrl(url_);
    session.SetBody(cpr::Body{info});
    session.SetAuth(cpr::Authentication{team_key_, ""});
    auto response_ = session.Post();

    return response_.status_code;
  }


  std::vector<robotLocation> getRobotLocationList()
  {
    std::string url_ = "https://api.mksmart.org/sciroc-competition/"+team_id_+"/sciroc-robot-location";

    cpr::Session session;
    session.SetVerifySsl(false);
    session.SetUrl(url_);
    session.SetAuth(cpr::Authentication{team_key_, ""});
    auto response_ = session.Get();

    //prettyJson(response_.text);

    return robotLocationListJsonToObject(response_.text);
  }

  robotLocation getRobotLocation(std::string id)
  {
    std::string url_ = "https://api.mksmart.org/sciroc-competition/"+team_id_+"/sciroc-robot-location/"+id;

    cpr::Session session;
    session.SetVerifySsl(false);
    session.SetUrl(url_);
    session.SetAuth(cpr::Authentication{team_key_, ""});
    auto response_ = session.Get();

    //prettyJson(response_.text);

    return robotLocationJsonToObject(response_.text);
  }

  int putRobotLocation(robotLocation robotLocation_)
  {
    std::string url_ = "https://api.mksmart.org/sciroc-competition/"+team_id_+"/sciroc-robot-location/"+robotLocation_.id;

    //std::string payload = "  {\n    \"@id\": \"xx\",\n    \"@type\": \"xx\",\n   \"episode\": \"xx\"\n  \"team\": \"xx\"\n  \"timestamp\": \"xx\"\n  \"x\": \"xx\"\n  \"y\": \"xx\"\n  \"z\": \"xx\"\n  }";

    json j = robotLocationToJson(robotLocation_);
    std::string info = j.dump();

    cpr::Session session;
    session.SetVerifySsl(false);
    session.SetUrl(url_);
    session.SetBody(cpr::Body{info});
    session.SetAuth(cpr::Authentication{team_key_, ""});
    auto response_ = session.Put();

    return response_.status_code;
  }

  int postRobotLocation(robotLocation robotLocation_)
  {
    std::string url_ = "https://api.mksmart.org/sciroc-competition/"+team_id_+"/sciroc-robot-location/"+robotLocation_.id;

    //std::string payload = "  {\n    \"@id\": \"xx\",\n    \"@type\": \"xx\",\n   \"episode\": \"xx\"\n  \"team\": \"xx\"\n  \"timestamp\": \"xx\"\n  \"x\": \"xx\"\n  \"y\": \"xx\"\n  \"z\": \"xx\"\n  }";

    json j = robotLocationToJson(robotLocation_);
    std::string info = j.dump();

    cpr::Session session;
    session.SetVerifySsl(false);
    session.SetUrl(url_);
    session.SetBody(cpr::Body{info});
    session.SetAuth(cpr::Authentication{team_key_, ""});
    auto response_ = session.Post();

    return response_.status_code;
  }

  // Episode3
  menu getMenu()
  {
    std::string url_ = "https://api.mksmart.org/sciroc-competition/master/sciroc-episode3-menu";

    cpr::Session session;
    session.SetVerifySsl(false);
    session.SetUrl(url_);
    session.SetAuth(cpr::Authentication{team_key_, ""});
    auto response_ = session.Get();

    //prettyJson(response_.text);

    return menuJsonToObject(response_.text);
  }

  std::vector<table> getTableList()
  {
    std::string url_ = "https://api.mksmart.org/sciroc-competition/"+team_id_+"/sciroc-episode3-table";

    cpr::Session session;
    session.SetVerifySsl(false);
    session.SetUrl(url_);
    session.SetAuth(cpr::Authentication{team_key_, ""});
    auto response_ = session.Get();

    //prettyJson(response_.text);

    return tableListJsonToObject(response_.text);

  }

  table getTable(std::string id)
  {
    std::string url_ = "https://api.mksmart.org/sciroc-competition/"+team_id_+"/sciroc-episode3-table/"+ id;

    cpr::Session session;
    session.SetVerifySsl(false);
    session.SetUrl(url_);
    session.SetAuth(cpr::Authentication{team_key_, ""});
    auto response_ = session.Get();

    //prettyJson(response_.text);

    return tableJsonToObject(response_.text);

  }

  int putTable(table table_)
  {
    std::string url_ = "https://api.mksmart.org/sciroc-competition/"+team_id_+"/sciroc-episode3-table/"+ table_.id;

    //std::string payload = "  {\n    \"@id\": \"xx\",\n    \"@type\": \"xx\",\n    \"customers\": xx,\n    \"status\": \"xx\"\n  }";

    json j = tableToJson(table_);
    std::string info = j.dump();

    cpr::Session session;
    session.SetVerifySsl(false);
    session.SetUrl(url_);
    session.SetBody(cpr::Body{info});
    session.SetAuth(cpr::Authentication{team_key_, ""});
    auto response_ = session.Put();

    return response_.status_code;
  }

  int postTable(table table_)
  {
    std::string url_ = "https://api.mksmart.org/sciroc-competition/"+team_id_+"/sciroc-episode3-table/"+ table_.id;

    json j = tableToJson(table_);
    std::string info = j.dump();

    cpr::Session session;
    session.SetVerifySsl(false);
    session.SetUrl(url_);
    session.SetBody(cpr::Body{info});
    session.SetAuth(cpr::Authentication{team_key_, ""});
    auto response_ = session.Post();

    return response_.status_code;
  }

  int deleteTable(std::string id)
  {
    std::string url_ = "https://api.mksmart.org/sciroc-competition/"+team_id_+"/sciroc-episode3-table/"+ id;

    cpr::Session session;
    session.SetVerifySsl(false);
    session.SetUrl(url_);
    session.SetAuth(cpr::Authentication{team_key_, ""});
    auto response_ = session.Delete();

    return response_.status_code;
  }

  std::vector<order> getOrderList()
  {
    std::string url_ = "https://api.mksmart.org/sciroc-competition/"+team_id_+"/sciroc-episode3-order";

    cpr::Session session;
    session.SetVerifySsl(false);
    session.SetUrl(url_);
    session.SetAuth(cpr::Authentication{team_key_, ""});
    auto response_ = session.Get();

    //prettyJson(response_.text);

    orderListJsonToObject(response_.text);

  }

  order getOrder(std::string id)
  {
    std::string url_ = "https://api.mksmart.org/sciroc-competition/"+team_id_+"/sciroc-episode3-order/"+ id;

    cpr::Session session;
    session.SetVerifySsl(false);
    session.SetUrl(url_);
    session.SetAuth(cpr::Authentication{team_key_, ""});
    auto response_ = session.Get();

    //prettyJson(response_.text);

    return orderJsonToObject(response_.text);
   }

  int putOrder(order order_)
  {
    std::string url_ = "https://api.mksmart.org/sciroc-competition/"+team_id_+"/sciroc-episode3-order/"+ order_.id;

    std::string payload = "  {\n    \"@id\": \"xx\",\n    \"@type\": \"xx\",\n    \"table\": xx,\n    \"timestamp\": \"xx\"\n  \"products\": [\n xx],\n   \"status\": xx,\n  }";

    json j = orderToJson(order_);
    std::string info = j.dump();

    cpr::Session session;
    session.SetVerifySsl(false);
    session.SetUrl(url_);
    session.SetBody(cpr::Body{info});
    session.SetAuth(cpr::Authentication{team_key_, ""});
    auto response_ = session.Put();

    return response_.status_code;
  }

  int postOrder(order order_)
  {
    std::string url_ = "https://api.mksmart.org/sciroc-competition/"+team_id_+"/sciroc-episode3-order/"+ order_.id;

    std::string payload = "  {\n    \"@id\": \"xx\",\n    \"@type\": \"xx\",\n    \"table\": xx,\n    \"timestamp\": \"xx\"\n  \"products\": [\n xx],\n   \"status\": xx,\n  }";

    json j = orderToJson(order_);
    std::string info = j.dump();

    cpr::Session session;
    session.SetVerifySsl(false);
    session.SetUrl(url_);
    session.SetBody(cpr::Body{info});
    session.SetAuth(cpr::Authentication{team_key_, ""});
    auto response_ = session.Post();

    return response_.status_code;
  }

  int deleteOrder(std::string id)
  {
    std::string url_ = "https://api.mksmart.org/sciroc-competition/"+team_id_+"/sciroc-episode3-order/"+ id;

    cpr::Session session;
    session.SetVerifySsl(false);
    session.SetUrl(url_);
    session.SetAuth(cpr::Authentication{team_key_, ""});
    auto response_ = session.Delete();

    return response_.status_code;
  }

  // Episode4
  std::vector<shop> getShopsList()
  {
    std::string url_ = "https://api.mksmart.org/sciroc-competition/master/sciroc-episode4-shop";

    cpr::Session session;
    session.SetVerifySsl(false);
    session.SetUrl(url_);
    session.SetAuth(cpr::Authentication{team_key_, ""});
    auto response_ = session.Get();

    //prettyJson(response_.text);

    return shopJsonToObject(response_.text);
  }

  /////////////////////////JSON/////////////////////////////

  robotStatus robotStatusJsonToObject(std::string info)
  {
    auto json = json::parse(info);

    robotStatus robotStatus_;

    robotStatus_.id = json["@id"];
    robotStatus_.type = json["@type"];
    robotStatus_.message = json["message"];
    robotStatus_.episode = json["episode"];
    robotStatus_.team = json["team"];
    robotStatus_.timestamp = json["timestamp"];
    robotStatus_.x = json["x"];
    robotStatus_.y = json["y"];
    robotStatus_.z = json["z"];

    return robotStatus_;
  }

  std::vector<robotStatus> robotStatusListJsonToObject(std::string info)
  {
    auto json = json::parse(info);

    std::vector<robotStatus> robotStatusList_;
    for(int i=0; i< json.size(); i++)
    {
      robotStatus robotStatus_;
      std::string str = json[i].dump();
      robotStatus_ = robotStatusJsonToObject(str);
      robotStatusList_.push_back(robotStatus_);
    }

    return robotStatusList_;
  }

  robotLocation robotLocationJsonToObject(std::string info)
  {
    auto json = json::parse(info);

    robotLocation robotLocation_;

    robotLocation_.id = json["@id"];
    robotLocation_.type = json["@type"];
    robotLocation_.episode = json["episode"];
    robotLocation_.team = json["team"];
    robotLocation_.timestamp = json["timestamp"];
    robotLocation_.x = json["x"];
    robotLocation_.y = json["y"];
    robotLocation_.z = json["z"];

    return robotLocation_;
  }

  std::vector<robotLocation> robotLocationListJsonToObject(std::string info)
  {
    auto json = json::parse(info);

    std::vector<robotLocation> robotLocations_;
    for(int i=0; i< json.size(); i++)
    {
      robotLocation robotLocation_;
      std::string str = json[i].dump();
      robotLocation_ = robotLocationJsonToObject(str);
      robotLocations_.push_back(robotLocation_);
    }
    return robotLocations_;
  }

  table tableJsonToObject(std::string info)
  {
    auto json = json::parse(info);

    table table_;

    table_.id = json["@id"];
    table_.type = json["@type"];
    table_.customers = json["customers"];
    table_.status = json["status"];

    return table_;
  }

  std::vector<table> tableListJsonToObject(std::string info)
  {
    auto json = json::parse(info);

    std::vector<table> tables_;

    for(int i=0; i< json.size(); i++)
    {
      table table_;
      std::string str = json[i].dump();
      table_ = tableJsonToObject(str);
      tables_.push_back(table_);
    }
    return tables_;
  }

  product productTextToObject(std::string info)
  {
    auto json = json::parse(info);
    product product_;

    product_.id = json["@id"];
    product_.type = json["@type"];
    product_.label = json["label"];
    product_.descriptions = json["descriptions"];
    product_.price = json["price"];

    return product_;
  }

  product productJsonToObject(json json)
  {
    product product_;

    product_.id = json["@id"];
    product_.type = json["@type"];
    product_.label = json["label"];
    product_.descriptions = json["descriptions"];
    product_.price = json["price"];

    return product_;
  }

  order orderJsonToObject(std::string info)
  {
    auto json = json::parse(info);

    order order_;

    order_.id = json["@id"];
    order_.type = json["@type"];
    order_.table = json["table"];
    order_.timestamp = json["timestamp"];

    std::vector<std::string> products_;
    for(int i=0; i< json["products"].size(); i++)
    {
      std::string p = json["products"][i];
      products_.push_back(p);
    }

    order_.products = products_;
    order_.status = json["status"];

    return order_;
  }

  std::vector<order> orderListJsonToObject(std::string info)
  {
    auto json = json::parse(info);

    std::vector<order> orders_;

    for(int i=0; i< json.size(); i++)
    {
      order order_;
      std::string str = json[i].dump();
      order_ = orderJsonToObject(str);
      orders_.push_back(order_);

    }
    return orders_;
  }

  menu menuJsonToObject(std::string info)
  {
    auto json = json::parse(info);

    menu menu_;

    menu_.id = json[0]["@id"];
    menu_.type = json[0]["@type"];

    std::vector<product> products_;

    for(int i=0; i< json[0]["products"].size(); i++)
    {

      product p = productJsonToObject(json[0]["products"][i]);
      products_.push_back(p);

    }
    menu_.products = products_;

    return menu_;
  }

  std::vector<shop> shopJsonToObject(std::string info)
  {
    auto json = json::parse(info);

    std::vector<shop> shopList;

    for(int i=0; i< json.size(); i++)
    {
      shop shop_;

      shop_.id = json[i]["@id"];
      shop_.type = json[i]["@type"];
      shop_.floor = json[i]["floor"];
      shop_.description = json[i]["description"];
      shop_.goal = json[i]["goal"];

      shopList.push_back(shop_);
    }

    return shopList;
  }

  json robotStatusToJson(robotStatus robotStatus_)
  {
    json json;

    json["@id"] = robotStatus_.id;
    json["@type"] = robotStatus_.type;
    json["message"] = robotStatus_.message;
    json["episode"] = robotStatus_.episode;
    json["team"] = robotStatus_.team;
    json["timestamp"] = robotStatus_.timestamp;
    json["x"] = robotStatus_.x;
    json["y"] = robotStatus_.y;
    json["z"] = robotStatus_.z;

    return json;
  }

  json robotLocationToJson(robotLocation robotLocation_)
  {
    json json;

    json["@id"] = robotLocation_.id;
    json["@type"] = robotLocation_.type;
    json["episode"] = robotLocation_.episode;
    json["team"] = robotLocation_.team;
    json["timestamp"] = robotLocation_.timestamp;
    json["x"] = robotLocation_.x;
    json["y"] = robotLocation_.y;
    json["z"] = robotLocation_.z;

    return json;
  }

  json tableToJson(table table_)
  {
    json json;

    json["@id"] = table_.id;
    json["@type"] = table_.type;
    json["customers"] = table_.customers;
    json["status"] = table_.status;

    return json;
  }

  json productToJson(product product_)
  {
    json json;

    json["@id"] = product_.id;
    json["@type"] = product_.type;
    json["label"] = product_.label;
    json["descriptions"] = product_.descriptions;
    json["price"] = product_.price;

    return json;
  }

  json orderToJson(order order_)
  {
    json json;

    json["@id"] = order_.id;
    json["@type"] = order_.type;
    json["table"] = order_.table;
    json["timestamp"] = order_.timestamp;
    json["products"] = order_.products;
    json["staus"] = order_.status;

    return json;
  }

  void prettyJson(std::string info)
  {
    auto json = json::parse(info);
    std::cout << json.dump(4) << std::endl;
  }


};  // namespace gb_attention
