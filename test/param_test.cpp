#include "commons/parameter.h"
#include <chrono>
#include <gtest/gtest.h>
#include <ros/ros.h>
#include <thread>

#include <iostream> // std::cout
#include <sstream> // std::stringstream

class ParameterTest : public ::testing::Test
{
public:
  std::string param_name_{"param"};
  std::string param_type_{"type"};
  bool param_typing_ = true;

  std::string param_equival_name_{"param_equival"};

  rtask::commons::Parameter param_, param_equal_, param_equival_;

  ros::NodeHandle m_nh;

  ParameterTest()
    : m_nh("~")
  {
    param_ = {param_name_, param_type_};
  }

  ~ParameterTest() {}
};

TEST_F(ParameterTest, isEqual)
{
  param_equal_ = {param_name_, param_type_};
  ASSERT_TRUE(param_.isEqual(param_equal_));
}

TEST_F(ParameterTest, isEquivalent)
{
  param_equival_ = {param_equival_name_, param_type_};
  ASSERT_TRUE(param_.isEquivalent(param_equival_));
}

TEST_F(ParameterTest, fromXmlRpc)
{
  XmlRpc::XmlRpcValue param_descr_;
  rtask::commons::Parameter param_check_;

  m_nh.getParam("param", param_descr_);

  param_check_ = rtask::commons::Parameter(param_descr_);

  ASSERT_TRUE(param_check_.isEqual(param_descr_));
}

TEST_F(ParameterTest, toMsg)
{
  rtask_msgs::Parameter param_msg_;
  rtask::commons::Parameter param_check_;

  param_msg_ = param_.toMsg();
  param_check_ = {param_msg_};

  ASSERT_TRUE(param_check_.isEqual(param_));
}

TEST_F(ParameterTest, set)
{
  rtask::commons::Parameter param_check_;

  param_check_.set(param_name_, param_type_);

  ASSERT_TRUE(param_.isEqual(param_check_));
}

TEST_F(ParameterTest, setType)
{
  rtask::commons::Parameter param_check_;

  param_check_.setType(param_type_);

  ASSERT_TRUE(param_.isEquivalent(param_check_));
}

TEST_F(ParameterTest, getName)
{
  ASSERT_EQ(param_.getName(), param_name_);
}

TEST_F(ParameterTest, getType)
{
  ASSERT_EQ(param_.getType(), param_type_);
}

TEST_F(ParameterTest, isValid)
{
  ASSERT_EQ(param_.isValid(), true);
}

TEST_F(ParameterTest, toPddl)
{
  std::string param_pddl_check_ = "?" + param_name_ + " - " + param_type_;

  ASSERT_EQ(param_.toPddl(param_typing_), param_pddl_check_);
}

int main(int argc, char* argv[])
{
  testing::InitGoogleTest(&argc, argv);

  ros::init(argc, argv, "param_test_node");

  std::thread t([] {
    while (ros::ok())
      ros::spin();
  });
  auto res = RUN_ALL_TESTS();
  ros::shutdown();
  return res;
}
