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

  rtask::commons::Property prop_in_{"in_prop_", true};

  std::vector<rtask::commons::Property> props_ = {{"bool_prop_", false},
                                                  {"int_prop_", 1},
                                                  {"double_prop_", 0.1},
                                                  {"string_prop_", std::string("string")}};
  std::vector<rtask::commons::Property> dev_unique_props_ = props_;

  std::vector<rtask::commons::Property> dev_extra_props_ = props_;

  std::string cap_name_{"capability"};
  std::string cap_in_name_{"capability_in"};
  std::vector<rtask::commons::Capability> dev_cap_ = {{cap_name_, props_}};

  rtask::commons::Capability cap_in_{cap_in_name_, props_};

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
  std::vector<std::string> unique_prop_list_check_;

  for (auto& up_ : dev_unique_props_) {
    unique_prop_list_check_.push_back(up_.getName());
  }

  ASSERT_EQ(dev_.getUniquePropertyList(), unique_prop_list_check_);
}

TEST_F(DeviceTest, getExtraPropertyList)
{
  std::vector<std::string> extra_prop_list_check_;

  for (auto& ep_ : dev_extra_props_) {
    extra_prop_list_check_.push_back(ep_.getName());
  }

  ASSERT_EQ(dev_.getExtraPropertyList(), extra_prop_list_check_);
}

TEST_F(DeviceTest, getCapabilityList)
{
  std::vector<std::string> cap_list_check_;

  for (auto& c_ : dev_cap_) {
    cap_list_check_.push_back(c_.getName());
  }

  ASSERT_EQ(dev_.getCapabilityList(), cap_list_check_);
}

TEST_F(DeviceTest, hasUniqueProperty)
{
  std::string unique_prop_name_check_ = dev_unique_props_.at(0).getName();
  ASSERT_EQ(dev_.hasUniqueProperty(unique_prop_name_check_), true);
}
TEST_F(DeviceTest, deleteUniqueProperty)
{
  std::vector<rtask::commons::Property> unique_prop_check_ = dev_unique_props_;
  unique_prop_check_.pop_back();

  dev_.deleteUniqueProperty(dev_unique_props_.back().getName());

  ASSERT_EQ(dev_.getUniqueProperties(), unique_prop_check_);
}
TEST_F(DeviceTest, isUniquePropertyValid)
{
  bool valid_ = true;

  for (auto& p_ : dev_.getUniqueProperties())
    valid_ &= dev_.isUniquePropertyValid(p_.getName());

  ASSERT_EQ(valid_, true);
}

TEST_F(DeviceTest, getUniqueProperty)
{
  std::vector<rtask::commons::Property> unique_prop_check_{};
  std::vector<std::string> unique_prop_names_check_ = dev_.getUniquePropertyList();

  for (auto& n_ : unique_prop_names_check_)
    unique_prop_check_.push_back(dev_.getUniqueProperty(n_).second);

  ASSERT_EQ(dev_.getUniqueProperties(), unique_prop_check_);
}

TEST_F(DeviceTest, setUniqueProperty)
{
  dev_.setUniqueProperty(prop_in_.getName(), prop_in_.getValue().second);

  rtask::commons::Property unique_prop_check_ = dev_.getUniqueProperty(prop_in_.getName()).second;

  ASSERT_EQ(prop_in_, unique_prop_check_);
}

TEST_F(DeviceTest, hasExtraProperty)
{
  std::string extra_prop_name_check_ = dev_extra_props_.at(0).getName();
  ASSERT_EQ(dev_.hasExtraProperty(extra_prop_name_check_), true);
}
TEST_F(DeviceTest, deleteExtraProperty)
{
  std::vector<rtask::commons::Property> extra_prop_check_ = dev_extra_props_;
  extra_prop_check_.pop_back();

  dev_.deleteExtraProperty(dev_extra_props_.back().getName());

  ASSERT_EQ(dev_.getExtraProperties(), extra_prop_check_);
}
TEST_F(DeviceTest, isExtraPropertyValid)
{
  bool valid_ = true;

  for (auto& p_ : dev_.getExtraProperties())
    valid_ &= dev_.isExtraPropertyValid(p_.getName());

  ASSERT_EQ(valid_, true);
}
TEST_F(DeviceTest, getExtraProperty)
{
  std::vector<rtask::commons::Property> extra_prop_check_{};
  std::vector<std::string> extra_prop_names_check_ = dev_.getExtraPropertyList();

  for (auto& n_ : extra_prop_names_check_)
    extra_prop_check_.push_back(dev_.getExtraProperty(n_).second);

  ASSERT_EQ(extra_prop_check_, dev_.getExtraProperties());
}

TEST_F(DeviceTest, setExtraProperty)
{

  dev_.setExtraProperty(prop_in_.getName(), prop_in_.getValue().second);

  rtask::commons::Property extra_prop_check_ = dev_.getExtraProperty(prop_in_.getName()).second;

  ASSERT_EQ(prop_in_, extra_prop_check_);
}

TEST_F(DeviceTest, hasCapability)
{
  std::string cap_name_check_ = dev_cap_.at(0).getName();
  ASSERT_EQ(dev_.hasCapability(cap_name_check_), true);
}

TEST_F(DeviceTest, deleteCapability)
{
  std::vector<rtask::commons::Capability> cap_check_ = dev_cap_;
  cap_check_.pop_back();

  dev_.deleteCapability(dev_cap_.back().getName());

  ASSERT_EQ(dev_.getCapabilities(), cap_check_);
}

TEST_F(DeviceTest, isCapabilityValid)
{
  bool valid_ = true;

  for (auto& c_ : dev_.getCapabilities())
    valid_ &= dev_.isCapabilityValid(c_.getName());

  ASSERT_EQ(valid_, true);
}

TEST_F(DeviceTest, getCapability)
{
  std::vector<rtask::commons::Capability> cap_check_{};
  std::vector<std::string> cap_names_check_ = dev_.getCapabilityList();

  for (auto& n_ : cap_names_check_)
    cap_check_.push_back(dev_.getCapability(n_).second);

  ASSERT_EQ(cap_check_, dev_.getCapabilities());
}

TEST_F(DeviceTest, setCapability)
{

  dev_.setCapability(cap_in_.getName(), cap_in_.getProperties());

  rtask::commons::Capability cap_check_ = dev_.getCapability(cap_in_.getName()).second;

  ASSERT_EQ(cap_in_, cap_check_);
}

int main(int argc, char* argv[])
{
  testing::InitGoogleTest(&argc, argv);

  ros::init(argc, argv, "device_test_node");

  std::thread t([] {
    while (ros::ok())
      ros::spin();
  });
  auto res = RUN_ALL_TESTS();
  ros::shutdown();
  return res;
}
