/**
 * @file    main.cpp
 * @author  Royneal Rayess
 * @copyright MIT License (c) 2018 Royneal Rayess
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 *
 *
 * @brief DESCRIPTION
 * Test node to demo usage of rostest, 
 *
 */
// Bring in gtest
#include <gtest/gtest.h>
#include "ros/ros.h"
#include "kitting_cell/kuka.hpp"

// Declare a test
TEST(ServiceTests, Change_Publisher_Rate) {
  ros::NodeHandle nh;
  // Wait on Service to start before begining with testing
  if ( ros::service::waitForService ( "message_rate", 1000))
    ROS_INFO("Service message_rate started successfully");
    else
    ROS_INFO("Service Wait timed out ");

  ros::ServiceClient client = nh.serviceClient<beginner_tutorials::message_rate>
  ("message_rate");  // register service client
  beginner_tutorials::message_rate srv;  // create a service object
  srv.request.rate = 50;  // set service request value
  client.call(srv);
  EXPECT_EQ(1, srv.response.oldrate);  // service responds with old rate = 1 hz
}

// Run all declared tests
int main(int argc, char **argv) {
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "mypackagetest");
  return RUN_ALL_TESTS();
}