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


namespace gb_datahub
{

  std::string getRobotStatusList()
  {
    std::string url_ = "https://api.mksmart.org/sciroc-competition/"+team_id_+"/sciroc-robot-status";

    auto response_ = cpr::Get(cpr::Url{url_}, cpr::Authentication{team_key_, team_id_});
    //std::cout << response_.text << std::endl;

    if (response_.status_code == 200) {
      return response_.text;
    }
    return "";
  }

  std::string getRobotStatus(std::string id)
  {
    std::string url_ = "https://api.mksmart.org/sciroc-competition/"+team_id_+"/sciroc-robot-status/"+ id;

    auto response_ = cpr::Get(cpr::Url{url_}, cpr::Authentication{team_key_, team_id_});
    //std::cout << response_.text << std::endl;

    if (response_.status_code == 200) {
      return response_.text;
    }
    return "";
  }

  void putRobotStatus(std::string id, std::string info)
  {
    std::string url_ = "https://api.mksmart.org/sciroc-competition/"+team_id_+"/sciroc-robot-status/"+ id;
//payload example
    std::string payload = "  {\n    \"@id\": \"xx\",\n    \"@type\": \"xx\",\n    \"message\": xx,\n    \"episode\": \"xx\"\n  \"team\": \"xx\"\n  \"timestamp\": \"xx\"\n  \"x\": \"xx\"\n  \"y\": \"xx\"\n  \"z\": \"xx\"\n  }";
    payload = info;

    auto response_ = cpr::Put(cpr::Url{url_}, cpr::Payload{{"data",payload}}, cpr::Authentication{team_key_, team_id_});

  }

  void postRobotStatus(std::string id, std::string info)
  {
    std::string url_ = "https://api.mksmart.org/sciroc-competition/"+team_id_+"/sciroc-robot-status/"+ id;

    std::string payload = "  {\n    \"@id\": \"xx\",\n    \"@type\": \"xx\",\n    \"message\": xx,\n    \"episode\": \"xx\"\n  \"team\": \"xx\"\n  \"timestamp\": \"xx\"\n  \"x\": \"xx\"\n  \"y\": \"xx\"\n  \"z\": \"xx\"\n  }";
    payload = info;

    auto response_ = cpr::Post(cpr::Url{url_}, cpr::Payload{{"data",payload}}, cpr::Authentication{team_key_, team_id_});
  }


  std::string getRobotLocationList()
  {
    std::string url_ = "https://api.mksmart.org/sciroc-competition/"+team_id_+"/sciroc-robot-location";

    auto response_ = cpr::Get(cpr::Url{url_}, cpr::Authentication{team_key_, team_id_});
    //std::cout << response_.text << std::endl;

    if (response_.status_code == 200) {
      return response_.text;
    }
    return "";
  }

  std::string getRobotLocation(std::string id)
  {
    std::string url_ = "https://api.mksmart.org/sciroc-competition/"+team_id_+"/sciroc-robot-location/"+id;

    auto response_ = cpr::Get(cpr::Url{url_}, cpr::Authentication{team_key_, team_id_});
    //std::cout << response_.text << std::endl;

    if (response_.status_code == 200) {
      return response_.text;
    }
    return "";
  }

  void putRobotLocation(std::string id, std::string info)
  {
    std::string url_ = "https://api.mksmart.org/sciroc-competition/"+team_id_+"/sciroc-robot-location/"+id;

    std::string payload = "  {\n    \"@id\": \"xx\",\n    \"@type\": \"xx\",\n   \"episode\": \"xx\"\n  \"team\": \"xx\"\n  \"timestamp\": \"xx\"\n  \"x\": \"xx\"\n  \"y\": \"xx\"\n  \"z\": \"xx\"\n  }";
    payload = info;

    auto response_ = cpr::Put(cpr::Url{url_}, cpr::Payload{{"data",payload}}, cpr::Authentication{team_key_, team_id_});

  }

  void postRobotLocation(std::string id, std::string info)
  {
    std::string url_ = "https://api.mksmart.org/sciroc-competition/"+team_id_+"/sciroc-robot-location/"+id;

    std::string payload = "  {\n    \"@id\": \"xx\",\n    \"@type\": \"xx\",\n   \"episode\": \"xx\"\n  \"team\": \"xx\"\n  \"timestamp\": \"xx\"\n  \"x\": \"xx\"\n  \"y\": \"xx\"\n  \"z\": \"xx\"\n  }";
    payload = info;

    auto response_ = cpr::Post(cpr::Url{url_}, cpr::Payload{{"data",payload}}, cpr::Authentication{team_key_, team_id_});

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


    if (response_.status_code == 200) {
      return response_.text;
    }
    return "";
  }

  std::string getTable()
  {
    std::string url_ = "https://api.mksmart.org/sciroc-competition/"+team_id_+"/sciroc-episode3-table";

    auto response_ = cpr::Get(cpr::Url{url_}, cpr::Authentication{team_key_, team_id_});
    //std::cout << response_.text << std::endl;

    if (response_.status_code == 200) {
      return response_.text;
    }
    return "";
  }

  std::string getTable(std::string id)
  {
    std::string url_ = "https://api.mksmart.org/sciroc-competition/"+team_id_+"/sciroc-episode3-table/"+ id;

    auto response_ = cpr::Get(cpr::Url{url_}, cpr::Authentication{team_key_, team_id_});
    //std::cout << response_.text << std::endl;

    if (response_.status_code == 200) {
      return response_.text;
    }
    return "";
  }

  void putTable(std::string id, std::string info)
  {
    std::string url_ = "https://api.mksmart.org/sciroc-competition/"+team_id_+"/sciroc-episode3-table/"+ id;

    std::string payload = "  {\n    \"@id\": \"xx\",\n    \"@type\": \"xx\",\n    \"customers\": xx,\n    \"status\": \"xx\"\n  }";
    payload = info;

    auto response_ = cpr::Put(cpr::Url{url_}, cpr::Payload{{"data",payload}}, cpr::Authentication{team_key_, team_id_});
  }

  void postTable(std::string id, std::string info)
  {
    std::string url_ = "https://api.mksmart.org/sciroc-competition/"+team_id_+"/sciroc-episode3-table/"+ id;

    std::string payload = "  {\n    \"@id\": \"xx\",\n    \"@type\": \"xx\",\n    \"customers\": xx,\n    \"status\": \"xx\"\n  }";
    payload = info;

    auto response_ = cpr::Post(cpr::Url{url_}, cpr::Payload{{"data",payload}}, cpr::Authentication{team_key_, team_id_});

  }

  void deleteTable(std::string id)
  {
    std::string url_ = "https://api.mksmart.org/sciroc-competition/"+team_id_+"/sciroc-episode3-table/"+ id;

    auto response_ = cpr::Delete(cpr::Url{url_}, cpr::Authentication{team_key_, team_id_});
    //std::cout << response_.text << std::endl;
/*
    if (response_.status_code == 200) {
      return response_.text;
    }
    return "";
    */
  }

  std::string getOrder()
  {
    std::string url_ = "https://api.mksmart.org/sciroc-competition/"+team_id_+"/sciroc-episode3-order";

    auto response_ = cpr::Get(cpr::Url{url_}, cpr::Authentication{team_key_, team_id_});
    //std::cout << response_.text << std::endl;

    if (response_.status_code == 200) {
      return response_.text;
    }
    return "";
  }

  std::string getOrder(std::string id)
  {
    std::string url_ = "https://api.mksmart.org/sciroc-competition/"+team_id_+"/sciroc-episode3-order/"+ id;

    auto response_ = cpr::Get(cpr::Url{url_}, cpr::Authentication{team_key_, team_id_});
    //std::cout << response_.text << std::endl;

    if (response_.status_code == 200) {
      return response_.text;
    }
    return "";
   }

  void putOrder(std::string id, std::string info)
  {
    std::string url_ = "https://api.mksmart.org/sciroc-competition/"+team_id_+"/sciroc-episode3-order/"+ id;

    std::string payload = "  {\n    \"@id\": \"xx\",\n    \"@type\": \"xx\",\n    \"table\": xx,\n    \"timestamp\": \"xx\"\n  \"products\": [\n xx],\n   \"status\": xx,\n  }";
    payload = info;

    auto response_ = cpr::Put(cpr::Url{url_}, cpr::Payload{{"data",payload}}, cpr::Authentication{team_key_, team_id_});
  }

  void postOrder(std::string id, std::string info)
  {
    std::string url_ = "https://api.mksmart.org/sciroc-competition/"+team_id_+"/sciroc-episode3-order/"+ id;

    std::string payload = "  {\n    \"@id\": \"xx\",\n    \"@type\": \"xx\",\n    \"table\": xx,\n    \"timestamp\": \"xx\"\n  \"products\": [\n xx],\n   \"status\": xx,\n  }";
    payload = info;

    auto response_ = cpr::Post(cpr::Url{url_}, cpr::Payload{{"data",payload}}, cpr::Authentication{team_key_, team_id_});

  }

  void deleteOrder(std::string id)
  {
    std::string url_ = "https://api.mksmart.org/sciroc-competition/"+team_id_+"/sciroc-episode3-order/"+ id;

    auto response_ = cpr::Delete(cpr::Url{url_}, cpr::Authentication{team_key_, team_id_});

  }

  // Episode4
  std::string getShopList()
  {
    std::string url_ = "https://api.mksmart.org/sciroc-competition/master/sciroc-episode4-shop";

    auto response_ = cpr::Get(cpr::Url{url_}, cpr::Authentication{team_key_, team_id_});
    //std::cout << response_.text << std::endl;

    if (response_.status_code == 200) {
      return response_.text;
    }
    return "";
  }


};  // namespace gb_attention
