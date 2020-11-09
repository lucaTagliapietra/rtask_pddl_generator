#include "commons/agent.h"
#include <chrono>
#include <gtest/gtest.h>
#include <ros/ros.h>
#include <thread>

#include <iostream> // std::cout
#include <sstream> // std::stringstream

class AgentTest : public ::testing::Test
{
public:
  std::string agent_name_{"agent"};
  std::string agent_description_{"agent_description"};
  rtask::commons::AgentStatus agent_status_{rtask::commons::AgentStatus::READY};

  rtask::commons::Agent agent_;

  ros::NodeHandle m_nh;

  AgentTest()
    : m_nh("~")
  {
    agent_ = {agent_name_, agent_description_, agent_status_};
  }

  ~AgentTest() {}
};

TEST_F(AgentTest, equalityOperator)
{
  rtask::commons::Agent dual_agent_ = agent_;
  ASSERT_TRUE(dual_agent_ == agent_);
}

TEST_F(AgentTest, fromXmlRpc)
{
  XmlRpc::XmlRpcValue agent_descr_;
  rtask::commons::Agent agent_check_;

  m_nh.getParam("agent", agent_descr_);

  agent_check_ = rtask::commons::Agent(agent_descr_);

  ASSERT_TRUE(agent_check_ == agent_descr_);
}

TEST_F(AgentTest, toMsg)
{
  rtask_msgs::Agent agent_msg_;
  rtask::commons::Agent agent_check_;

  agent_msg_ = agent_.toMsg();
  agent_check_ = {agent_msg_};

  ASSERT_TRUE(agent_check_ == agent_);
}

TEST_F(AgentTest, set)
{
  rtask::commons::Agent agent_check_;

  agent_check_.set(agent_name_, agent_description_, agent_status_);

  ASSERT_TRUE(agent_check_ == agent_);
}

TEST_F(AgentTest, getName)
{
  ASSERT_EQ(agent_.getName(), agent_name_);
}

TEST_F(AgentTest, getDescription)
{
  ASSERT_EQ(agent_.getDescription(), agent_description_);
}

TEST_F(AgentTest, getStatus)
{
  ASSERT_EQ(agent_.getStatus(), agent_status_);
}

TEST_F(AgentTest, isValid)
{
  ASSERT_EQ(agent_.isValid(), true);
}

TEST_F(AgentTest, isReady)
{
  ASSERT_EQ(agent_.isReady(), true);
}

TEST_F(AgentTest, isBusy)
{
  ASSERT_EQ(agent_.isBusy(), false);
}

TEST_F(AgentTest, isConnected)
{
  ASSERT_EQ(agent_.isConnected(), true);
}

TEST_F(AgentTest, setDescription)
{
  rtask::commons::Agent agent_check_(agent_name_, {}, agent_status_);

  agent_check_.setDescription(agent_description_);

  ASSERT_TRUE(agent_check_ == agent_);
}

TEST_F(AgentTest, setStatus)
{
  rtask::commons::Agent agent_check_(agent_name_, agent_description_, {});

  agent_check_.setStatus(agent_status_);

  ASSERT_TRUE(agent_check_ == agent_);
}

int main(int argc, char* argv[])
{
  testing::InitGoogleTest(&argc, argv);

  ros::init(argc, argv, "agent_test_node");

  std::thread t([] {
    while (ros::ok())
      ros::spin();
  });
  auto res = RUN_ALL_TESTS();
  ros::shutdown();
  return res;
}
