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

  std::string getRobotStatusList()
  {
    std::string url_ = "https://api.mksmart.org/sciroc-competition/"+team_id_+"/sciroc-robot-status";

    cpr::Session session;
    session.SetVerifySsl(false);
    session.SetUrl(url_);
    session.SetAuth(cpr::Authentication{team_key_, ""});
    auto response_ = session.Get();
    //std::cout << response_.text << std::endl;

    if (response_.status_code == 200) {
      return response_.text;
    }
    return "";
  }

  std::string getRobotStatus(std::string id)
  {
    std::string url_ = "https://api.mksmart.org/sciroc-competition/"+team_id_+"/sciroc-robot-status/"+ id;

    cpr::Session session;
    session.SetVerifySsl(false);
    session.SetUrl(url_);
    session.SetAuth(cpr::Authentication{team_key_, ""});
    auto response_ = session.Get();
    //std::cout << response_.text << std::endl;

    if (response_.status_code == 200) {
      return response_.text;
    }
    return "";
  }

  void putRobotStatus(std::string id, robotStatus robotStatus_)
  {
    std::string url_ = "https://api.mksmart.org/sciroc-competition/"+team_id_+"/sciroc-robot-status/"+ id;
    std::string payload = "  {\n    \"@id\": \"xx\",\n    \"@type\": \"xx\",\n    \"message\": xx,\n    \"episode\": \"xx\"\n  \"team\": \"xx\"\n  \"timestamp\": \"xx\"\n  \"x\": \"xx\"\n  \"y\": \"xx\"\n  \"z\": \"xx\"\n  }";

    json j = robotStatusToJson(robotStatus_);
    std::string info = j.dump();

    cpr::Session session;
    session.SetVerifySsl(false);
    session.SetUrl(url_);
    session.SetBody(cpr::Body{info});
    session.SetAuth(cpr::Authentication{team_key_, ""});
    auto response_ = session.Put();
  }

  void postRobotStatus(std::string id, robotStatus robotStatus_)
  {
    std::string url_ = "https://api.mksmart.org/sciroc-competition/"+team_id_+"/sciroc-robot-status/"+ id;

    std::string payload = "  {\n    \"@id\": \"xx\",\n    \"@type\": \"xx\",\n    \"message\": xx,\n    \"episode\": \"xx\"\n  \"team\": \"xx\"\n  \"timestamp\": \"xx\"\n  \"x\": \"xx\"\n  \"y\": \"xx\"\n  \"z\": \"xx\"\n  }";

    json j = robotStatusToJson(robotStatus_);
    std::string info = j.dump();

    cpr::Session session;
    session.SetVerifySsl(false);
    session.SetUrl(url_);
    session.SetBody(cpr::Body{info});
    session.SetAuth(cpr::Authentication{team_key_, ""});
    auto response_ = session.Post();
  }


  std::string getRobotLocationList()
  {
    std::string url_ = "https://api.mksmart.org/sciroc-competition/"+team_id_+"/sciroc-robot-location";

    cpr::Session session;
    session.SetVerifySsl(false);
    session.SetUrl(url_);
    session.SetAuth(cpr::Authentication{team_key_, ""});
    auto response_ = session.Get();
    //std::cout << response_.text << std::endl;

    if (response_.status_code == 200) {
      return response_.text;
    }
    return "";
  }

  std::string getRobotLocation(std::string id)
  {
    std::string url_ = "https://api.mksmart.org/sciroc-competition/"+team_id_+"/sciroc-robot-location/"+id;

    cpr::Session session;
    session.SetVerifySsl(false);
    session.SetUrl(url_);
    session.SetAuth(cpr::Authentication{team_key_, ""});
    auto response_ = session.Get();
    //std::cout << response_.text << std::endl;

    if (response_.status_code == 200) {
      return response_.text;
    }
    return "";
  }

  void putRobotLocation(std::string id, robotLocation robotLocation_)
  {
    std::string url_ = "https://api.mksmart.org/sciroc-competition/"+team_id_+"/sciroc-robot-location/"+id;

    std::string payload = "  {\n    \"@id\": \"xx\",\n    \"@type\": \"xx\",\n   \"episode\": \"xx\"\n  \"team\": \"xx\"\n  \"timestamp\": \"xx\"\n  \"x\": \"xx\"\n  \"y\": \"xx\"\n  \"z\": \"xx\"\n  }";

    json j = robotLocationToJson(robotLocation_);
    std::string info = j.dump();

    cpr::Session session;
    session.SetVerifySsl(false);
    session.SetUrl(url_);
    session.SetBody(cpr::Body{info});
    session.SetAuth(cpr::Authentication{team_key_, ""});
    auto response_ = session.Put();

  }

  void postRobotLocation(std::string id, robotLocation robotLocation_)
  {
    std::string url_ = "https://api.mksmart.org/sciroc-competition/"+team_id_+"/sciroc-robot-location/"+id;

    std::string payload = "  {\n    \"@id\": \"xx\",\n    \"@type\": \"xx\",\n   \"episode\": \"xx\"\n  \"team\": \"xx\"\n  \"timestamp\": \"xx\"\n  \"x\": \"xx\"\n  \"y\": \"xx\"\n  \"z\": \"xx\"\n  }";

    json j = robotLocationToJson(robotLocation_);
    std::string info = j.dump();

    cpr::Session session;
    session.SetVerifySsl(false);
    session.SetUrl(url_);
    session.SetBody(cpr::Body{info});
    session.SetAuth(cpr::Authentication{team_key_, ""});
    auto response_ = session.Post();

  }

  // Episode3
  std::string getMenu()
  {
    std::string url_ = "https://api.mksmart.org/sciroc-competition/master/sciroc-episode3-menu";

    cpr::Session session;
    session.SetVerifySsl(false);
    session.SetUrl(url_);
    session.SetAuth(cpr::Authentication{team_key_, ""});
    auto response_ = session.Get();

    std::cout << "Text " << response_.text << std::endl;
    std::cout << std::endl;
    std::cout << "Status " << response_.status_code << std::endl;

    auto json = json::parse(response_.text);

    std::cout << json[0]["@id"] << std::endl;

    if (response_.status_code == 200) {
      return response_.text;
    }
    return "";
  }

  std::string getTable()
  {
    std::string url_ = "https://api.mksmart.org/sciroc-competition/"+team_id_+"/sciroc-episode3-table";

    cpr::Session session;
    session.SetVerifySsl(false);
    session.SetUrl(url_);
    session.SetAuth(cpr::Authentication{team_key_, ""});
    auto response_ = session.Get();

    if (response_.status_code == 200) {
      return response_.text;
    }
    return "";
  }

  std::string getTable(std::string id)
  {
    std::string url_ = "https://api.mksmart.org/sciroc-competition/"+team_id_+"/sciroc-episode3-table/"+ id;

    cpr::Session session;
    session.SetVerifySsl(false);
    session.SetUrl(url_);
    session.SetAuth(cpr::Authentication{team_key_, ""});
    auto response_ = session.Get();

    //std::cout << response_.text << std::endl;

    if (response_.status_code == 200) {
      return response_.text;
    }
    return "";
  }

  void putTable(std::string id, table table_)
  {
    std::string url_ = "https://api.mksmart.org/sciroc-competition/"+team_id_+"/sciroc-episode3-table/"+ id;

    std::string payload = "  {\n    \"@id\": \"xx\",\n    \"@type\": \"xx\",\n    \"customers\": xx,\n    \"status\": \"xx\"\n  }";

    json j = tableToJson(table_);
    std::string info = j.dump();

    cpr::Session session;
    session.SetVerifySsl(false);
    session.SetUrl(url_);
    session.SetBody(cpr::Body{info});
    session.SetAuth(cpr::Authentication{team_key_, ""});
    auto response_ = session.Put();

    std::cout << response_.status_code << std::endl;
  }

  void postTable(std::string id, table table_)
  {
    std::string url_ = "https://api.mksmart.org/sciroc-competition/"+team_id_+"/sciroc-episode3-table/"+ id;

    json j = tableToJson(table_);
    std::string info = j.dump();

    cpr::Session session;
    session.SetVerifySsl(false);
    session.SetUrl(url_);
    session.SetBody(cpr::Body{info});
    session.SetAuth(cpr::Authentication{team_key_, ""});
    auto response_ = session.Post();

    std::cout << response_.status_code << std::endl;
  }

  void deleteTable(std::string id)
  {
    std::string url_ = "https://api.mksmart.org/sciroc-competition/"+team_id_+"/sciroc-episode3-table/"+ id;

    cpr::Session session;
    session.SetVerifySsl(false);
    session.SetUrl(url_);
    session.SetAuth(cpr::Authentication{team_key_, ""});
    auto response_ = session.Delete();

    std::cout << response_.status_code << std::endl;
  }

  std::string getOrder()
  {
    std::string url_ = "https://api.mksmart.org/sciroc-competition/"+team_id_+"/sciroc-episode3-order";

    cpr::Session session;
    session.SetVerifySsl(false);
    session.SetUrl(url_);
    session.SetAuth(cpr::Authentication{team_key_, ""});
    auto response_ = session.Get();
    //std::cout << response_.text << std::endl;

    if (response_.status_code == 200) {
      return response_.text;
    }
    return "";
  }

  std::string getOrder(std::string id)
  {
    std::string url_ = "https://api.mksmart.org/sciroc-competition/"+team_id_+"/sciroc-episode3-order/"+ id;

    cpr::Session session;
    session.SetVerifySsl(false);
    session.SetUrl(url_);
    session.SetAuth(cpr::Authentication{team_key_, ""});
    auto response_ = session.Get();
    //std::cout << response_.text << std::endl;

    if (response_.status_code == 200) {
      return response_.text;
    }
    return "";
   }

  void putOrder(std::string id, order order_)
  {
    std::string url_ = "https://api.mksmart.org/sciroc-competition/"+team_id_+"/sciroc-episode3-order/"+ id;

    std::string payload = "  {\n    \"@id\": \"xx\",\n    \"@type\": \"xx\",\n    \"table\": xx,\n    \"timestamp\": \"xx\"\n  \"products\": [\n xx],\n   \"status\": xx,\n  }";

    json j = orderToJson(order_);
    std::string info = j.dump();

    cpr::Session session;
    session.SetVerifySsl(false);
    session.SetUrl(url_);
    session.SetBody(cpr::Body{info});
    session.SetAuth(cpr::Authentication{team_key_, ""});
    auto response_ = session.Put();
  }

  void postOrder(std::string id, order order_)
  {
    std::string url_ = "https://api.mksmart.org/sciroc-competition/"+team_id_+"/sciroc-episode3-order/"+ id;

    std::string payload = "  {\n    \"@id\": \"xx\",\n    \"@type\": \"xx\",\n    \"table\": xx,\n    \"timestamp\": \"xx\"\n  \"products\": [\n xx],\n   \"status\": xx,\n  }";

    json j = orderToJson(order_);
    std::string info = j.dump();

    cpr::Session session;
    session.SetVerifySsl(false);
    session.SetUrl(url_);
    session.SetBody(cpr::Body{info});
    session.SetAuth(cpr::Authentication{team_key_, ""});
    auto response_ = session.Post();

  }

  void deleteOrder(std::string id)
  {
    std::string url_ = "https://api.mksmart.org/sciroc-competition/"+team_id_+"/sciroc-episode3-order/"+ id;

    cpr::Session session;
    session.SetVerifySsl(false);
    session.SetUrl(url_);
    session.SetAuth(cpr::Authentication{team_key_, ""});
    auto response_ = session.Delete();

    std::cout << response_.status_code << std::endl;

  }

  // Episode4
  std::string getShopList()
  {
    std::string url_ = "https://api.mksmart.org/sciroc-competition/master/sciroc-episode4-shop";

    cpr::Session session;
    session.SetVerifySsl(false);
    session.SetUrl(url_);
    session.SetAuth(cpr::Authentication{team_key_, ""});
    auto response_ = session.Get();

    auto json = json::parse(response_.text);

    //std::cout << response_.text << std::endl;
    //std::cout << json << std::endl;

    /*
    auto a = json.dump(4);
    std::cout << json[0]["@id"] << std::endl;
    std::cout << json["@id"] << std::endl;
*/

    //std::cout << json[0] << std::endl;

    if (response_.status_code == 200) {
      return response_.text;
    }

    return "";
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
    order_.table = json["tavle"];
    order_.timestamp = json["timestamp"];

    std::vector<product> products_;
    for(int i=0; i< json["products"].size(); i++)
    {
      /*
      products_[i].id = json[i]["products"]["@id"];
      products_[i].type = json[i]["products"]["@type"];
      products_[i].label = json[i]["products"]["label"];
      products_[i].descriptions = json[i]["descriptions"];
      products_[i].price = json[i]["products"]["price"];
*/
      product p = productJsonToObject(json["products"][i]);
      products_.push_back(p);
    }

    order_.products =products_;
    order_.status = json["status"];

    return order_;
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
    std::cout << json.size() << std::endl;

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
    //json["products"] = order_.products;
    //rellenar esto
    json["staus"] = order_.status;

    return json;
  }

  void prettyJson(std::string info)
  {
    auto json = json::parse(info);
    std::cout << json.dump(4) << std::endl;
  }


};  // namespace gb_attention
