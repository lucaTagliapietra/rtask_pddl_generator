#include "commons/capability.h"
#include <chrono>
#include <gtest/gtest.h>
#include <ros/ros.h>
#include <thread>

#include <iostream> // std::cout
#include <sstream> // std::stringstream

class CapabilityTest : public ::testing::Test
{
public:
  std::string cap_name_{"capability"};
  rtask::commons::Property cap_prop_{"bool_prop_", false};

  std::vector<rtask::commons::Property> cap_props_1_{{"int_prop_", 1},
                                                     {"double_prop_", 0.1},
                                                     {"string_prop_", std::string("string")}};

  std::vector<rtask::commons::Property> cap_props_all_{{"bool_prop_", false},
                                                       {"int_prop_", 1},
                                                       {"double_prop_", 0.1},
                                                       {"string_prop_", std::string("string")}};

  std::vector<rtask::commons::Property> cap_props_;

  rtask::commons::Capability cap_;

  ros::NodeHandle m_nh;

  CapabilityTest()
    : m_nh("~")
  {
    cap_props_.reserve(cap_props_1_.size());
    cap_props_.insert(cap_props_.end(), cap_props_1_.begin(), cap_props_1_.end());

    cap_ = {cap_name_, cap_props_};
    cap_.setProperty("bool_prop_", false);
  }

  ~CapabilityTest() {}
};

TEST_F(CapabilityTest, equalityOperator)
{
  rtask::commons::Capability dual_cap_ = cap_;
  ASSERT_TRUE(dual_cap_ == cap_);
}

TEST_F(CapabilityTest, fromXmlRpc)
{
  XmlRpc::XmlRpcValue cap_descr_;
  rtask::commons::Capability cap_check_;

  m_nh.getParam("capability", cap_descr_);

  cap_check_ = rtask::commons::Capability(cap_descr_);

  ASSERT_TRUE(cap_check_ == cap_descr_);
}

TEST_F(CapabilityTest, toMsg)
{
  rtask_msgs::Capability cap_msg_;
  rtask::commons::Capability cap_check_;

  cap_msg_ = cap_.toMsg();
  cap_check_ = {cap_msg_};

  ASSERT_TRUE(cap_check_ == cap_);
}

TEST_F(CapabilityTest, set)
{
  rtask::commons::Capability cap_check_;

  cap_check_.set(cap_name_, cap_props_all_);

  ASSERT_TRUE(cap_check_ == cap_);
}

TEST_F(CapabilityTest, getName)
{
  ASSERT_EQ(cap_.getName(), cap_name_);
}

TEST_F(CapabilityTest, isValid)
{
  ASSERT_EQ(cap_.isValid(), true);
}

TEST_F(CapabilityTest, getProperties)
{
  bool same_ = true;
  for (auto& p : cap_.getProperties()) {
    auto it = std::find_if(cap_props_all_.begin(), cap_props_all_.end(), [p](auto& op) { return p == op; });
    if (it == cap_props_all_.end()) {
      same_ &= false;
    }
  }

  ASSERT_EQ(same_, true);
}

TEST_F(CapabilityTest, getPropertyList)
{
  std::vector<std::string> prop_list_;

  for (auto& prop_ : cap_props_all_) {
    prop_list_.push_back(prop_.getName());
  }

  bool same_ = true;

  for (auto& p_name : cap_.getPropertyList()) {
    auto it = std::find_if(prop_list_.begin(), prop_list_.end(), [p_name](auto& name) { return name == p_name; });
    if (it == prop_list_.end()) {
      same_ &= false;
    }
  }

  ASSERT_EQ(same_, true);
}

TEST_F(CapabilityTest, hasProperty)
{
  ASSERT_EQ(cap_.hasProperty(cap_prop_.getName()), true);
}

TEST_F(CapabilityTest, deleteProperty)
{
  cap_.deleteProperty(cap_prop_.getName());

  ASSERT_EQ(cap_.getProperties(), cap_props_1_);
}

TEST_F(CapabilityTest, isPropertyValid)
{
  bool valid_ = true;
  for (auto& p : cap_.getProperties())
    valid_ &= cap_.isPropertyValid(p.getName());

  ASSERT_EQ(valid_, true);
}

TEST_F(CapabilityTest, getProperty)
{
  bool equal_ = (cap_.getProperty(cap_prop_.getName()).second == cap_prop_);

  ASSERT_EQ(equal_, true);
}

int main(int argc, char* argv[])
{
  testing::InitGoogleTest(&argc, argv);

  ros::init(argc, argv, "capability_test_node");

  std::thread t([] {
    while (ros::ok())
      ros::spin();
  });
  auto res = RUN_ALL_TESTS();
  ros::shutdown();
  return res;
}
