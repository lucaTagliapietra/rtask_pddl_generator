#include "commons/condition.h"
#include <chrono>
#include <gtest/gtest.h>
#include <ros/ros.h>
#include <thread>

#include <iostream> // std::cout
#include <sstream> // std::stringstream

class ConditionTest : public ::testing::Test
{
public:
  std::string cond_name_{"condition"};

  std::string arg_0_{"arg_0"};
  std::string arg_1_{"arg_1"};

  std::string arg_2_{"arg_2"};
  std::string arg_3_{"arg_3"};

  std::vector<std::string> cond_args_{arg_0_, arg_1_};

  std::vector<std::string> cond_equiv_args_{arg_2_, arg_3_};

  bool cond_neg_ = true;
  bool cond_typing_ = true;

  rtask::commons::Condition cond_, cond_equiv_;

  ros::NodeHandle m_nh;

  ConditionTest()
    : m_nh("~")
  {
    cond_ = {cond_name_, cond_args_, cond_neg_};
    cond_equiv_ = {cond_name_, cond_equiv_args_, cond_neg_};
  }

  ~ConditionTest() {}
};

TEST_F(ConditionTest, isEqual)
{
  ASSERT_TRUE(cond_.isEqual(cond_));
}

TEST_F(ConditionTest, isEquivalent)
{
  ASSERT_TRUE(cond_.isEquivalent(cond_equiv_));
}

TEST_F(ConditionTest, fromXmlRpc)
{
  XmlRpc::XmlRpcValue cond_descr_;
  rtask::commons::Condition cond_check_;

  m_nh.getParam("condition", cond_descr_);

  cond_check_ = rtask::commons::Condition(cond_descr_);

  ASSERT_TRUE(cond_check_.isEqual(cond_descr_));
}

TEST_F(ConditionTest, toMsg)
{
  rtask_msgs::Condition cond_msg_;
  rtask::commons::Condition cond_check_;

  cond_msg_ = cond_.toMsg();
  cond_check_ = {cond_msg_};

  ASSERT_TRUE(cond_.isEqual(cond_check_));
}

TEST_F(ConditionTest, set)
{
  rtask::commons::Condition cond_check_;

  cond_check_.set(cond_name_, cond_args_, cond_neg_);

  ASSERT_TRUE(cond_.isEqual(cond_check_));
}

TEST_F(ConditionTest, getName)
{
  ASSERT_EQ(cond_.getName(), cond_name_);
}

TEST_F(ConditionTest, getNegated)
{
  ASSERT_EQ(cond_.getNegated(), cond_neg_);
}

TEST_F(ConditionTest, getArgs)
{
  std::vector<std::string> cond_args_check_ = cond_.getArgs();

  ASSERT_TRUE(cond_args_check_ == cond_args_);
}

TEST_F(ConditionTest, isValid)
{
  ASSERT_EQ(cond_.isValid(), true);
}

TEST_F(ConditionTest, toPddl)
{
  std::string cond_pddl_check_;

  if (cond_neg_)
    cond_pddl_check_ += "(not ";
  cond_pddl_check_ += "(" + cond_name_;
  for (auto& a : cond_.getArgs())
    cond_pddl_check_ += " ?" + a;
  cond_pddl_check_ += ")";
  if (cond_neg_)
    cond_pddl_check_ += ")";

  ASSERT_EQ(cond_.toPddl(cond_typing_), cond_pddl_check_);
}

int main(int argc, char* argv[])
{
  testing::InitGoogleTest(&argc, argv);

  ros::init(argc, argv, "condition_test_node");

  std::thread t([] {
    while (ros::ok())
      ros::spin();
  });
  auto res = RUN_ALL_TESTS();
  ros::shutdown();
  return res;
}
