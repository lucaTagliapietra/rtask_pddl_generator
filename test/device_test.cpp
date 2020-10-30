#include "commons/device.h"
#include <chrono>
#include <gtest/gtest.h>
#include <ros/ros.h>
#include <thread>

#include <iostream> // std::cout
#include <sstream> // std::stringstream

class DeviceTest : public ::testing::Test
{
public:
  std::string dev_name_{"device"};
  rtask::commons::DeviceClass dev_class_ = rtask::commons::DeviceClass::ROBOT;
  std::string dev_subclass_{"Subclass"};

  std::vector<rtask::commons::Property> props_ = {{"bool_prop_", false},
                                                  {"int_prop_", 1},
                                                  {"double_prop_", 0.1},
                                                  {"string_prop_", std::string("string")}};
  std::vector<rtask::commons::Property> dev_unique_props_ = props_;

  std::vector<rtask::commons::Property> dev_extra_props_ = props_;

  std::string cap_name_{"capability"};
  std::vector<rtask::commons::Capability> dev_cap_ = {{cap_name_, props_}};

  rtask::commons::Device dev_;

  ros::NodeHandle m_nh;

  DeviceTest()
    : m_nh("~")
  {
    dev_ = {dev_name_, dev_class_, dev_subclass_, dev_unique_props_, dev_extra_props_, dev_cap_};
  }

  ~DeviceTest() {}
};

TEST_F(DeviceTest, equalityOperator)
{
  rtask::commons::Device dual_dev_ = dev_;
  ASSERT_TRUE(dual_dev_ == dev_);
}

TEST_F(DeviceTest, fromXmlRpc)
{
  XmlRpc::XmlRpcValue dev_descr_;
  rtask::commons::Device dev_check_;

  m_nh.getParam("device", dev_descr_);

  dev_check_ = rtask::commons::Device(dev_descr_);

  ASSERT_TRUE(dev_check_ == dev_descr_);
}

TEST_F(DeviceTest, toMsg)
{
  rtask_msgs::Device dev_msg_;
  rtask::commons::Device dev_check_;

  dev_msg_ = dev_.toMsg();
  dev_check_ = {dev_msg_};

  ASSERT_TRUE(dev_check_ == dev_);
}

TEST_F(DeviceTest, set)
{
  rtask::commons::Device dev_check_;

  dev_check_.set(dev_name_, dev_class_, dev_subclass_, dev_unique_props_, dev_extra_props_, dev_cap_);

  ASSERT_TRUE(dev_check_ == dev_);
}

TEST_F(DeviceTest, isValid)
{
  ASSERT_EQ(dev_.isValid(), true);
}

TEST_F(DeviceTest, getName)
{
  ASSERT_EQ(dev_.getName(), dev_name_);
}

TEST_F(DeviceTest, getClass)
{
  ASSERT_EQ(dev_.getClass(), dev_class_);
}

TEST_F(DeviceTest, getSubclass)
{
  ASSERT_EQ(dev_.getSubclass(), dev_subclass_);
}

TEST_F(DeviceTest, getUniqueProperties)
{
  ASSERT_EQ(dev_.getUniqueProperties(), dev_unique_props_);
}

TEST_F(DeviceTest, getExtraProperties)
{
  ASSERT_EQ(dev_.getExtraProperties(), dev_extra_props_);
}

TEST_F(DeviceTest, getCapabilities)
{
  ASSERT_EQ(dev_.getCapabilities(), dev_cap_);
}

TEST_F(DeviceTest, getUniquePropertyList)
{
  std::vector<std::string> unique_prop_list_;

  for (auto& up_ : dev_unique_props_) {
    unique_prop_list_.push_back(up_.getName());
  }

  ASSERT_EQ(dev_.getUniquePropertyList(), unique_prop_list_);
}

TEST_F(DeviceTest, getExtraPropertyList)
{
  std::vector<std::string> extra_prop_list_;

  for (auto& ep_ : dev_extra_props_) {
    extra_prop_list_.push_back(ep_.getName());
  }

  ASSERT_EQ(dev_.getExtraPropertyList(), extra_prop_list_);
}

TEST_F(DeviceTest, getCapabilityList)
{
  std::vector<std::string> cap_list_;

  for (auto& c_ : dev_cap_) {
    cap_list_.push_back(c_.getName());
  }

  ASSERT_EQ(dev_.getCapabilityList(), cap_list_);
}

// TEST_F(CapabilityTest, deleteProperty)
//{
//  cap_.deleteProperty(cap_prop_.getName());

//  ASSERT_EQ(cap_.getProperties(), cap_props_1_);
//}

// TEST_F(CapabilityTest, isPropertyValid)
//{
//  bool valid_ = true;
//  for (auto& p : cap_.getProperties())
//    valid_ &= p.isValid();

//  ASSERT_EQ(valid_, true);
//}

// TEST_F(CapabilityTest, getProperty)
//{
//  bool equal_ = (cap_.getProperty(cap_prop_.getName()).second == cap_prop_);

//  ASSERT_EQ(equal_, true);
//}

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
