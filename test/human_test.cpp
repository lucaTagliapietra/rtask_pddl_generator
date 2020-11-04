#include "commons/human.h"
#include <chrono>
#include <gtest/gtest.h>
#include <ros/ros.h>
#include <thread>

#include <iostream> // std::cout
#include <sstream> // std::stringstream

class HumanTest : public ::testing::Test
{
public:
  std::string human_name_{"human"};

  std::string tool_name_0_{"tool_0"};
  std::string tool_name_1_{"tool_1"};

  rtask::commons::DeviceClass tool_class_ = rtask::commons::DeviceClass::TOOL;

  std::vector<rtask::commons::Property> props_ = {{"bool_prop_", false},
                                                  {"int_prop_", 1},
                                                  {"double_prop_", 0.1},
                                                  {"string_prop_", std::string("string")}};

  std::vector<rtask::commons::Property> tool_unique_props_ = props_;

  std::vector<rtask::commons::Property> tool_extra_props_ = props_;

  std::string cap_name_{"capability"};
  std::vector<rtask::commons::Capability> tool_cap_ = {{cap_name_, props_}};

  rtask::commons::Device tool_0_{tool_name_0_, tool_class_, {}, tool_unique_props_, tool_extra_props_, tool_cap_};
  rtask::commons::Device tool_1_{tool_name_1_, tool_class_, {}, tool_unique_props_, tool_extra_props_, tool_cap_};

  std::vector<rtask::commons::Device> human_tools_{tool_0_, tool_1_};

  std::vector<rtask::commons::Property> human_extra_props_ = props_;

  rtask::commons::Human human_;

  ros::NodeHandle m_nh;

  HumanTest()
    : m_nh("~")
  {
    human_ = {human_name_, human_tools_, human_extra_props_};
  }

  ~HumanTest() {}
};

TEST_F(HumanTest, equalityOperator)
{
  rtask::commons::Human dual_human_ = human_;
  ASSERT_TRUE(dual_human_ == human_);
}

TEST_F(HumanTest, fromXmlRpc)
{
  XmlRpc::XmlRpcValue human_descr_;
  rtask::commons::Human human_check_;

  m_nh.getParam("human", human_descr_);

  human_check_ = rtask::commons::Human(human_descr_);

  ASSERT_TRUE(human_check_ == human_descr_);
}

TEST_F(HumanTest, toMsg)
{
  rtask_msgs::Human human_msg_;
  rtask::commons::Human human_check_;

  human_msg_ = human_.toMsg();
  human_check_ = {human_msg_};

  ASSERT_TRUE(human_check_ == human_);
}

TEST_F(HumanTest, set)
{
  rtask::commons::Human human_check_;

  human_check_.set(human_name_, human_tools_, human_extra_props_);

  ASSERT_TRUE(human_check_ == human_);
}

TEST_F(HumanTest, isValid)
{
  ASSERT_EQ(human_.isValid(), true);
}

TEST_F(HumanTest, getName)
{
  ASSERT_EQ(human_.getName(), human_name_);
}

TEST_F(HumanTest, getTools)
{
  ASSERT_EQ(human_.getTools(), human_tools_);
}

TEST_F(HumanTest, getExtraProperties)
{
  ASSERT_EQ(human_.getExtraProperties(), human_extra_props_);
}

TEST_F(HumanTest, getExtraPropertyList)
{
  std::vector<std::string> extra_prop_list_check_;

  for (auto& ep_ : human_extra_props_) {
    extra_prop_list_check_.push_back(ep_.getName());
  }

  ASSERT_EQ(human_.getExtraPropertyList(), extra_prop_list_check_);
}

TEST_F(HumanTest, getToolList)
{
  std::vector<std::string> tool_list_check_;

  for (auto& t_ : human_tools_) {
    tool_list_check_.push_back(t_.getName());
  }

  ASSERT_EQ(human_.getToolList(), tool_list_check_);
}

TEST_F(HumanTest, hasExtraProperty)
{
  std::string extra_prop_name_check_ = human_extra_props_.at(0).getName();
  ASSERT_EQ(human_.hasExtraProperty(extra_prop_name_check_), true);
}

TEST_F(HumanTest, deleteExtraProperty)
{
  std::vector<rtask::commons::Property> extra_prop_check_ = human_extra_props_;
  extra_prop_check_.pop_back();

  human_.deleteExtraProperty(human_extra_props_.back().getName());

  ASSERT_EQ(human_.getExtraProperties(), extra_prop_check_);
}

TEST_F(HumanTest, isExtraPropertyValid)
{
  bool valid_ = true;

  for (auto& p_ : human_.getExtraProperties())
    valid_ &= human_.isExtraPropertyValid(p_.getName());

  ASSERT_EQ(valid_, true);
}

TEST_F(HumanTest, getExtraProperty)
{
  std::vector<rtask::commons::Property> extra_prop_check_{};
  std::vector<std::string> extra_prop_names_check_ = human_.getExtraPropertyList();

  for (auto& n_ : extra_prop_names_check_)
    extra_prop_check_.push_back(human_.getExtraProperty(n_).second);

  ASSERT_EQ(extra_prop_check_, human_.getExtraProperties());
}

TEST_F(HumanTest, setExtraProperty)
{
  rtask::commons::Property extra_prop_in_{"bool_prop_", true};

  human_.setExtraProperty(extra_prop_in_.getName(), extra_prop_in_.getValue().second);

  rtask::commons::Property extra_prop_check_ = human_.getExtraProperty(extra_prop_in_.getName()).second;

  ASSERT_EQ(extra_prop_in_, extra_prop_check_);
}

TEST_F(HumanTest, hasTool)
{
  std::string tool_name_check_ = human_tools_.at(0).getName();
  ASSERT_EQ(human_.hasTool(tool_name_check_), true);
}

TEST_F(HumanTest, deleteTool)
{
  std::vector<rtask::commons::Device> tools_check_ = human_tools_;
  tools_check_.pop_back();

  human_.deleteTool(human_tools_.back().getName());

  ASSERT_EQ(human_.getTools(), tools_check_);
}

TEST_F(HumanTest, isToolValid)
{
  bool valid_ = true;

  for (auto& t_ : human_.getTools())
    valid_ &= human_.isToolValid(t_.getName());

  ASSERT_EQ(valid_, true);
}

TEST_F(HumanTest, getTool)
{
  std::vector<rtask::commons::Device> tools_check_{};
  std::vector<std::string> tool_names_check_ = human_.getToolList();

  for (auto& n_ : tool_names_check_)
    tools_check_.push_back(human_.getTool(n_).second);

  ASSERT_EQ(tools_check_, human_.getTools());
}

TEST_F(HumanTest, setTool)
{
  rtask::commons::Device tool_in_{tool_name_0_, tool_class_, {}, tool_unique_props_, tool_extra_props_, tool_cap_};

  human_.setTool(tool_name_0_, tool_0_);

  rtask::commons::Device tool_check_ = human_.getTool(tool_in_.getName()).second;

  ASSERT_EQ(tool_in_, tool_check_);
}

int main(int argc, char* argv[])
{
  testing::InitGoogleTest(&argc, argv);

  ros::init(argc, argv, "human_test_node");

  std::thread t([] {
    while (ros::ok())
      ros::spin();
  });
  auto res = RUN_ALL_TESTS();
  ros::shutdown();
  return res;
}
