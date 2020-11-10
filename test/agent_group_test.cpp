#include "commons/agentGroup.h"
#include <chrono>
#include <gtest/gtest.h>
#include <ros/ros.h>
#include <thread>

#include <iostream> // std::cout
#include <sstream> // std::stringstream

class AgentGroupTest : public ::testing::Test
{
public:
  std::string ag_name_{"agent_group"};
  std::string ag_descr_{"agent_group_description"};

  std::string a_0_name_{"agent_0"};
  std::string a_0_descr_{"agent_0_description"};
  rtask::commons::AgentStatus a_0_status_{rtask::commons::AgentStatus::UNKNOWN};
  rtask::commons::Agent a_0_{a_0_name_, a_0_descr_, a_0_status_};

  std::string a_1_name_{"agent_1"};
  std::string a_1_descr_{"agent_1_description"};
  rtask::commons::AgentStatus a_1_status_{rtask::commons::AgentStatus::READY};
  rtask::commons::Agent a_1_{a_1_name_, a_1_descr_, a_1_status_};

  std::string a_2_name_{"agent_2"};
  std::string a_2_descr_{"agent_2_description"};
  rtask::commons::AgentStatus a_2_status_{rtask::commons::AgentStatus::BUSY};
  rtask::commons::Agent a_2_{a_2_name_, a_2_descr_, a_2_status_};

  std::string a_3_name_{"agent_3"};
  std::string a_3_descr_{"agent_3_description"};
  rtask::commons::AgentStatus a_3_status_{rtask::commons::AgentStatus::DISCONNECTED};
  rtask::commons::Agent a_3_{a_3_name_, a_3_descr_, a_3_status_};

  std::string a_in_name_{"agent_in"};
  std::string a_in_descr_{"agent_in_description"};
  rtask::commons::AgentStatus a_in_status_{rtask::commons::AgentStatus::DISCONNECTED};
  rtask::commons::Agent a_in_{a_in_name_, a_in_descr_, a_in_status_};

  std::vector<rtask::commons::Agent> ag_agents_{a_0_, a_1_, a_2_, a_3_};

  rtask::commons::AgentGroup ag_;

  ros::NodeHandle m_nh;

  AgentGroupTest()
    : m_nh("~")
  {
    ag_ = {ag_name_, ag_descr_, ag_agents_};
  }

  ~AgentGroupTest() {}
};

TEST_F(AgentGroupTest, equalityOperator)
{
  rtask::commons::AgentGroup dual_ag_ = ag_;
  ASSERT_TRUE(dual_ag_ == ag_);
}

TEST_F(AgentGroupTest, fromXmlRpc)
{
  XmlRpc::XmlRpcValue ag_descr_;
  rtask::commons::AgentGroup ag_check_;

  m_nh.getParam("agent_group", ag_descr_);

  ag_check_ = rtask::commons::AgentGroup(ag_descr_);

  ASSERT_TRUE(ag_check_ == ag_descr_);
}

TEST_F(AgentGroupTest, toMsg)
{
  rtask_msgs::AgentGroup ag_msg_;
  rtask::commons::AgentGroup ag_check_;

  ag_msg_ = ag_.toMsg();
  ag_check_ = {ag_msg_};

  ASSERT_TRUE(ag_check_ == ag_);
}

TEST_F(AgentGroupTest, set)
{
  rtask::commons::AgentGroup ag_check_;

  ag_check_.set(ag_name_, ag_descr_, ag_agents_);

  ASSERT_TRUE(ag_check_ == ag_);
}

TEST_F(AgentGroupTest, isValid)
{
  ASSERT_EQ(ag_.isValid(), true);
}

TEST_F(AgentGroupTest, getName)
{
  ASSERT_EQ(ag_.getName(), ag_name_);
}

TEST_F(AgentGroupTest, getDescription)
{
  ASSERT_EQ(ag_.getDescription(), ag_descr_);
}

TEST_F(AgentGroupTest, getAgents)
{
  ASSERT_EQ(ag_.getAgents(), ag_agents_);
}

TEST_F(AgentGroupTest, getAgentList)
{
  std::vector<std::string> a_list_check_;

  for (auto& a_ : ag_agents_) {
    a_list_check_.push_back(a_.getName());
  }

  ASSERT_EQ(ag_.getAgentList(), a_list_check_);
}

TEST_F(AgentGroupTest, hasAgent)
{
  std::string a_name_check_ = ag_agents_.at(0).getName();
  ASSERT_EQ(ag_.hasAgent(a_name_check_), true);
}

TEST_F(AgentGroupTest, deleteAgent)
{
  std::vector<rtask::commons::Agent> a_check_ = ag_agents_;
  a_check_.pop_back();

  ag_.deleteAgent(ag_agents_.back().getName());

  ASSERT_EQ(ag_.getAgents(), a_check_);
}

TEST_F(AgentGroupTest, isAgentValid)
{
  bool valid_ = true;

  for (auto& a_ : ag_.getAgents())
    valid_ &= ag_.isAgentValid(a_.getName());

  ASSERT_EQ(valid_, true);
}

TEST_F(AgentGroupTest, getAgent)
{
  std::vector<rtask::commons::Agent> a_check_{};
  std::vector<std::string> a_names_check_ = ag_.getAgentList();

  for (auto& n_ : a_names_check_)
    a_check_.push_back(ag_.getAgent(n_).second);

  ASSERT_EQ(a_check_, ag_.getAgents());
}

TEST_F(AgentGroupTest, setAgentAllInputs)
{

  ag_.setAgent(a_in_.getName(), a_in_.getDescription(), a_in_.getStatus());

  rtask::commons::Agent a_check_ = ag_.getAgent(a_in_.getName()).second;

  ASSERT_EQ(a_in_, a_check_);
}

TEST_F(AgentGroupTest, setAgent)
{

  ag_.setAgent(a_in_.getName(), a_in_);

  rtask::commons::Agent a_check_ = ag_.getAgent(a_in_.getName()).second;

  ASSERT_EQ(a_in_, a_check_);
}

int main(int argc, char* argv[])
{
  testing::InitGoogleTest(&argc, argv);

  ros::init(argc, argv, "agent_group_test_node");

  std::thread t([] {
    while (ros::ok())
      ros::spin();
  });
  auto res = RUN_ALL_TESTS();
  ros::shutdown();
  return res;
}
