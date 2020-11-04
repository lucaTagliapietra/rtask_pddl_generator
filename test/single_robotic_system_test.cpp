#include "commons/singleRoboticSystem.h"
#include <chrono>
#include <gtest/gtest.h>
#include <ros/ros.h>
#include <thread>

#include <iostream> // std::cout
#include <sstream> // std::stringstream

class SingleRoboticSystemTest : public ::testing::Test
{
public:
  std::string srs_name_{"single_robotic_system"};

  std::string dev_name_0_{"device_0"};
  std::string dev_name_1_{"device_1"};

  rtask::commons::DeviceClass dev_class_0_ = rtask::commons::DeviceClass::ROBOT;
  rtask::commons::DeviceClass dev_class_1_ = rtask::commons::DeviceClass::GRIPPER;

  std::string dev_subclass_{"Subclass"};

  std::vector<rtask::commons::Property> props_ = {{"bool_prop_", false},
                                                  {"int_prop_", 1},
                                                  {"double_prop_", 0.1},
                                                  {"string_prop_", std::string("string")}};

  std::vector<rtask::commons::Property> dev_unique_props_ = props_;

  std::vector<rtask::commons::Property> dev_extra_props_ = props_;

  std::string cap_name_{"capability"};
  std::vector<rtask::commons::Capability> dev_cap_ = {{cap_name_, props_}};

  rtask::commons::Device dev_0_{dev_name_0_,
                                dev_class_0_,
                                dev_subclass_,
                                dev_unique_props_,
                                dev_extra_props_,
                                dev_cap_};
  rtask::commons::Device dev_1_{dev_name_1_,
                                dev_class_1_,
                                dev_subclass_,
                                dev_unique_props_,
                                dev_extra_props_,
                                dev_cap_};

  std::vector<rtask::commons::Device> srs_devs_{dev_0_, dev_1_};

  std::vector<rtask::commons::Property> srs_extra_props_ = props_;

  rtask::commons::SingleRoboticSystem srs_;

  ros::NodeHandle m_nh;

  SingleRoboticSystemTest()
    : m_nh("~")
  {
    srs_ = {srs_name_, srs_devs_, srs_extra_props_};
  }

  ~SingleRoboticSystemTest() {}
};

TEST_F(SingleRoboticSystemTest, equalityOperator)
{
  rtask::commons::SingleRoboticSystem dual_srs_ = srs_;
  ASSERT_TRUE(dual_srs_ == srs_);
}

TEST_F(SingleRoboticSystemTest, fromXmlRpc)
{
  XmlRpc::XmlRpcValue srs_descr_;
  rtask::commons::SingleRoboticSystem srs_check_;

  m_nh.getParam("single_robotic_system", srs_descr_);

  srs_check_ = rtask::commons::SingleRoboticSystem(srs_descr_);

  ASSERT_TRUE(srs_check_ == srs_descr_);
}

TEST_F(SingleRoboticSystemTest, toMsg)
{
  rtask_msgs::SingleRoboticSystem srs_msg_;
  rtask::commons::SingleRoboticSystem srs_check_;

  srs_msg_ = srs_.toMsg();
  srs_check_ = {srs_msg_};

  ASSERT_TRUE(srs_check_ == srs_);
}

TEST_F(SingleRoboticSystemTest, set)
{
  rtask::commons::SingleRoboticSystem srs_check_;

  srs_check_.set(srs_name_, srs_devs_, srs_extra_props_);

  ASSERT_TRUE(srs_check_ == srs_);
}

TEST_F(SingleRoboticSystemTest, isValid)
{
  ASSERT_EQ(srs_.isValid(), true);
}

TEST_F(SingleRoboticSystemTest, getName)
{
  ASSERT_EQ(srs_.getName(), srs_name_);
}

TEST_F(SingleRoboticSystemTest, getDevices)
{
  ASSERT_EQ(srs_.getDevices(), srs_devs_);
}

TEST_F(SingleRoboticSystemTest, getExtraProperties)
{
  ASSERT_EQ(srs_.getExtraProperties(), srs_extra_props_);
}

TEST_F(SingleRoboticSystemTest, getExtraPropertyList)
{
  std::vector<std::string> extra_prop_list_check_;

  for (auto& ep_ : srs_extra_props_) {
    extra_prop_list_check_.push_back(ep_.getName());
  }

  ASSERT_EQ(srs_.getExtraPropertyList(), extra_prop_list_check_);
}

TEST_F(SingleRoboticSystemTest, getDeviceList)
{
  std::vector<std::string> dev_list_check_;

  for (auto& d_ : srs_devs_) {
    dev_list_check_.push_back(d_.getName());
  }

  ASSERT_EQ(srs_.getDeviceList(), dev_list_check_);
}

TEST_F(SingleRoboticSystemTest, hasExtraProperty)
{
  std::string extra_prop_name_check_ = srs_extra_props_.at(0).getName();
  ASSERT_EQ(srs_.hasExtraProperty(extra_prop_name_check_), true);
}

TEST_F(SingleRoboticSystemTest, deleteExtraProperty)
{
  std::vector<rtask::commons::Property> extra_prop_check_ = srs_extra_props_;
  extra_prop_check_.pop_back();

  srs_.deleteExtraProperty(srs_extra_props_.back().getName());

  ASSERT_EQ(srs_.getExtraProperties(), extra_prop_check_);
}

TEST_F(SingleRoboticSystemTest, isExtraPropertyValid)
{
  bool valid_ = true;

  for (auto& p_ : srs_.getExtraProperties())
    valid_ &= srs_.isExtraPropertyValid(p_.getName());

  ASSERT_EQ(valid_, true);
}

TEST_F(SingleRoboticSystemTest, getExtraProperty)
{
  std::vector<rtask::commons::Property> extra_prop_check_{};
  std::vector<std::string> extra_prop_names_check_ = srs_.getExtraPropertyList();

  for (auto& n_ : extra_prop_names_check_)
    extra_prop_check_.push_back(srs_.getExtraProperty(n_).second);

  ASSERT_EQ(extra_prop_check_, srs_.getExtraProperties());
}

TEST_F(SingleRoboticSystemTest, setExtraProperty)
{
  rtask::commons::Property extra_prop_in_{"bool_prop_", true};

  srs_.setExtraProperty(extra_prop_in_.getName(), extra_prop_in_.getValue().second);

  rtask::commons::Property extra_prop_check_ = srs_.getExtraProperty(extra_prop_in_.getName()).second;

  ASSERT_EQ(extra_prop_in_, extra_prop_check_);
}

TEST_F(SingleRoboticSystemTest, hasDevice)
{
  std::string dev_name_check_ = srs_devs_.at(0).getName();
  ASSERT_EQ(srs_.hasDevice(dev_name_check_), true);
}

TEST_F(SingleRoboticSystemTest, deleteDevice)
{
  std::vector<rtask::commons::Device> devs_check_ = srs_devs_;
  devs_check_.pop_back();

  srs_.deleteDevice(srs_devs_.back().getName());

  ASSERT_EQ(srs_.getDevices(), devs_check_);
}

TEST_F(SingleRoboticSystemTest, isDeviceValid)
{
  bool valid_ = true;

  for (auto& d_ : srs_.getDevices())
    valid_ &= srs_.isDeviceValid(d_.getName());

  ASSERT_EQ(valid_, true);
}

TEST_F(SingleRoboticSystemTest, getDevice)
{
  std::vector<rtask::commons::Device> devs_check_{};
  std::vector<std::string> dev_names_check_ = srs_.getDeviceList();

  for (auto& n_ : dev_names_check_)
    devs_check_.push_back(srs_.getDevice(n_).second);

  ASSERT_EQ(devs_check_, srs_.getDevices());
}

TEST_F(SingleRoboticSystemTest, setDeviceAllInputs)
{
  rtask::commons::Device dev_in_{
    dev_name_0_, dev_class_0_, dev_subclass_, dev_unique_props_, dev_extra_props_, dev_cap_};

  srs_.setDevice(dev_name_0_, dev_class_0_, dev_subclass_, dev_unique_props_, dev_extra_props_, dev_cap_);

  rtask::commons::Device dev_check_ = srs_.getDevice(dev_in_.getName()).second;

  ASSERT_EQ(dev_in_, dev_check_);
}

TEST_F(SingleRoboticSystemTest, setDevice)
{

  rtask::commons::Device dev_in_{
    dev_name_0_, dev_class_0_, dev_subclass_, dev_unique_props_, dev_extra_props_, dev_cap_};

  srs_.setDevice(dev_in_.getName(),
                 dev_in_.getClass(),
                 dev_in_.getSubclass(),
                 dev_in_.getUniqueProperties(),
                 dev_in_.getExtraProperties(),
                 dev_in_.getCapabilities());

  rtask::commons::Device dev_check_ = srs_.getDevice(dev_in_.getName()).second;

  ASSERT_EQ(dev_in_, dev_check_);
}

int main(int argc, char* argv[])
{
  testing::InitGoogleTest(&argc, argv);

  ros::init(argc, argv, "single_robotic_system_test_node");

  std::thread t([] {
    while (ros::ok())
      ros::spin();
  });
  auto res = RUN_ALL_TESTS();
  ros::shutdown();
  return res;
}
