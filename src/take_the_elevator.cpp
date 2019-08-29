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

using namespace web::http;
using namespace web::http::client;

namespace gb_datahub
{

TakeTheElevator::TakeTheElevator(): nh_(), team_id_("gentlebots"), team_key_("ea7bfa2e-77e3-4948-80b6-5b84af77a4b2")
{

}


void TakeTheElevator::robotStatus() // get, post y put
{
  std::string url_ = "https://api.mksmart.org/sciroc-competition/"+team_id_+"/sciroc-robot-status/"+ id;
  http_client client_(url_);

}

void TakeTheElevator::robotLocation() // get, post, put
{
  std::string url_ = "https://api.mksmart.org/sciroc-competition/"+team_id_+"/sciroc-robot-location/"+ id;
  http_client client_(url_);

}

void TakeTheElevator::shopList() //get
{
  std::string url_ = "https://api.mksmart.org/sciroc-competition/master/sciroc-episode4-shop"
  http_client client_(url_);
/*
  http_response response_;
  response_ = client.request(methods::GET, "/get").get();
  std::cout << response_.extract_string().get() << "\n";

*/
}
/*
void TakeTheElevator::identifyGoal()
{


}
*/
}

};  // namespace gb_attention


int main(int argc, char** argv){
	ros::init(argc, argv, "take_the_elevator");
	TakeTheElevator take_the_elevator;
	ros::spin();
	return 0;
}
