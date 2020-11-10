#include "commons/collectiveRoboticSystem.h"
#include <chrono>
#include <gtest/gtest.h>
#include <ros/ros.h>
#include <thread>

#include <iostream> // std::cout
#include <sstream> // std::stringstream

class CollectiveRoboticSystemTest : public ::testing::Test
{
public:
  std::string crs_name_{"collective_robotic_system"};

  std::string srs_0_name_{"single_robotic_system_0"};
  std::string srs_1_name_{"single_robotic_system_1"};
  std::string srs_in_name_{"single_robotic_system_in"};

  std::string dev_0_name_{"device_0"};
  std::string dev_1_name_{"device_1"};

  rtask::commons::DeviceClass dev_0_class_ = rtask::commons::DeviceClass::ROBOT;
  rtask::commons::DeviceClass dev_1_class_ = rtask::commons::DeviceClass::GRIPPER;

  std::string dev_subclass_{"Subclass"};

  std::vector<rtask::commons::Property> props_ = {{"bool_prop_", false},
                                                  {"int_prop_", 1},
                                                  {"double_prop_", 0.1},
                                                  {"string_prop_", std::string("string")}};

  rtask::commons::Property extra_prop_in_{"in_prop_", true};

  std::vector<rtask::commons::Property> dev_unique_props_ = props_;

  std::vector<rtask::commons::Property> dev_extra_props_ = props_;

  std::string cap_name_{"capability"};
  std::vector<rtask::commons::Capability> dev_cap_ = {{cap_name_, props_}};

  rtask::commons::Device dev_0_{dev_0_name_,
                                dev_0_class_,
                                dev_subclass_,
                                dev_unique_props_,
                                dev_extra_props_,
                                dev_cap_};
  rtask::commons::Device dev_1_{dev_1_name_,
                                dev_1_class_,
                                dev_subclass_,
                                dev_unique_props_,
                                dev_extra_props_,
                                dev_cap_};

  std::vector<rtask::commons::Device> srs_devs_{dev_0_, dev_1_};

  std::vector<rtask::commons::Property> srs_extra_props_ = props_;

  rtask::commons::SingleRoboticSystem srs_0_{srs_0_name_, srs_devs_, srs_extra_props_};
  rtask::commons::SingleRoboticSystem srs_1_{srs_1_name_, srs_devs_, srs_extra_props_};
  rtask::commons::SingleRoboticSystem srs_in_{srs_in_name_, srs_devs_, srs_extra_props_};

  std::vector<rtask::commons::SingleRoboticSystem> crs_srs_{srs_0_, srs_1_};

  std::vector<rtask::commons::Property> crs_extra_props_ = props_;

  rtask::commons::CollectiveRoboticSystem crs_;

  ros::NodeHandle m_nh;

  CollectiveRoboticSystemTest()
    : m_nh("~")
  {
    crs_ = {crs_name_, crs_srs_, crs_extra_props_};
  }

  ~CollectiveRoboticSystemTest() {}
};

TEST_F(CollectiveRoboticSystemTest, equalityOperator)
{
  rtask::commons::CollectiveRoboticSystem dual_crs_ = crs_;
  ASSERT_TRUE(dual_crs_ == crs_);
}

TEST_F(CollectiveRoboticSystemTest, fromXmlRpc)
{
  XmlRpc::XmlRpcValue crs_descr_;
  rtask::commons::CollectiveRoboticSystem crs_check_;

  m_nh.getParam("collective_robotic_system", crs_descr_);

  crs_check_ = rtask::commons::CollectiveRoboticSystem(crs_descr_);

  ASSERT_TRUE(crs_check_ == crs_descr_);
}

TEST_F(CollectiveRoboticSystemTest, toMsg)
{
  rtask_msgs::CollectiveRoboticSystem crs_msg_;
  rtask::commons::CollectiveRoboticSystem crs_check_;

  crs_msg_ = crs_.toMsg();
  crs_check_ = {crs_msg_};

  ASSERT_TRUE(crs_check_ == crs_);
}

TEST_F(CollectiveRoboticSystemTest, set)
{
  rtask::commons::CollectiveRoboticSystem crs_check_;

  crs_check_.set(crs_name_, crs_srs_, crs_extra_props_);

  ASSERT_TRUE(crs_check_ == crs_);
}

TEST_F(CollectiveRoboticSystemTest, isValid)
{
  ASSERT_EQ(crs_.isValid(), true);
}

TEST_F(CollectiveRoboticSystemTest, getName)
{
  ASSERT_EQ(crs_.getName(), crs_name_);
}

TEST_F(CollectiveRoboticSystemTest, getSingleRoboticSystems)
{
  ASSERT_EQ(crs_.getSingleRoboticSystems(), crs_srs_);
}

TEST_F(CollectiveRoboticSystemTest, getExtraProperties)
{
  ASSERT_EQ(crs_.getExtraProperties(), crs_extra_props_);
}

TEST_F(CollectiveRoboticSystemTest, getExtraPropertyList)
{
  std::vector<std::string> extra_prop_list_check_;

  for (auto& ep_ : crs_extra_props_) {
    extra_prop_list_check_.push_back(ep_.getName());
  }

  ASSERT_EQ(crs_.getExtraPropertyList(), extra_prop_list_check_);
}

TEST_F(CollectiveRoboticSystemTest, getSingleRoboticSystemList)
{
  std::vector<std::string> srs_list_check_;

  for (auto& srs_ : crs_srs_) {
    srs_list_check_.push_back(srs_.getName());
  }

  ASSERT_EQ(crs_.getSingleRoboticSystemList(), srs_list_check_);
}

TEST_F(CollectiveRoboticSystemTest, hasExtraProperty)
{
  std::string extra_prop_name_check_ = crs_extra_props_.at(0).getName();
  ASSERT_EQ(crs_.hasExtraProperty(extra_prop_name_check_), true);
}

TEST_F(CollectiveRoboticSystemTest, deleteExtraProperty)
{
  std::vector<rtask::commons::Property> extra_prop_check_ = crs_extra_props_;
  extra_prop_check_.pop_back();

  crs_.deleteExtraProperty(crs_extra_props_.back().getName());

  ASSERT_EQ(crs_.getExtraProperties(), extra_prop_check_);
}

TEST_F(CollectiveRoboticSystemTest, isExtraPropertyValid)
{
  bool valid_ = true;

  for (auto& p_ : crs_.getExtraProperties())
    valid_ &= crs_.isExtraPropertyValid(p_.getName());

  ASSERT_EQ(valid_, true);
}

TEST_F(CollectiveRoboticSystemTest, getExtraProperty)
{
  std::vector<rtask::commons::Property> extra_prop_check_{};
  std::vector<std::string> extra_prop_names_check_ = crs_.getExtraPropertyList();

  for (auto& n_ : extra_prop_names_check_)
    extra_prop_check_.push_back(crs_.getExtraProperty(n_).second);

  ASSERT_EQ(extra_prop_check_, crs_.getExtraProperties());
}

TEST_F(CollectiveRoboticSystemTest, setExtraProperty)
{

  crs_.setExtraProperty(extra_prop_in_.getName(), extra_prop_in_.getValue().second);

  rtask::commons::Property extra_prop_check_ = crs_.getExtraProperty(extra_prop_in_.getName()).second;

  ASSERT_EQ(extra_prop_in_, extra_prop_check_);
}

TEST_F(CollectiveRoboticSystemTest, hasSingleRoboticSystem)
{
  std::string srs_name_check_ = crs_srs_.at(0).getName();
  ASSERT_EQ(crs_.hasSingleRoboticSystem(srs_name_check_), true);
}

TEST_F(CollectiveRoboticSystemTest, deleteSingleRoboticSystem)
{
  std::vector<rtask::commons::SingleRoboticSystem> srs_check_ = crs_srs_;
  srs_check_.pop_back();

  crs_.deleteSingleRoboticSystem(crs_srs_.back().getName());

  ASSERT_EQ(crs_.getSingleRoboticSystems(), srs_check_);
}

TEST_F(CollectiveRoboticSystemTest, isSingleRoboticSystemValid)
{
  bool valid_ = true;

  for (auto& srs_ : crs_.getSingleRoboticSystems())
    valid_ &= crs_.isSingleRoboticSystemValid(srs_.getName());

  ASSERT_EQ(valid_, true);
}

TEST_F(CollectiveRoboticSystemTest, getSingleRoboticSystem)
{
  std::vector<rtask::commons::SingleRoboticSystem> srs_check_{};
  std::vector<std::string> srs_names_check_ = crs_.getSingleRoboticSystemList();

  for (auto& n_ : srs_names_check_)
    srs_check_.push_back(crs_.getSingleRoboticSystem(n_).second);

  ASSERT_EQ(srs_check_, crs_.getSingleRoboticSystems());
}

TEST_F(CollectiveRoboticSystemTest, setSingleRoboticSystemAllInputs)
{
  crs_.setSingleRoboticSystem(srs_in_.getName(), srs_in_.getDevices(), srs_in_.getExtraProperties());

  rtask::commons::SingleRoboticSystem srs_check_ = crs_.getSingleRoboticSystem(srs_in_.getName()).second;

  ASSERT_EQ(srs_in_, srs_check_);
}

TEST_F(CollectiveRoboticSystemTest, setSingleRoboticSystem)
{

  crs_.setSingleRoboticSystem(srs_in_.getName(), srs_in_);

  rtask::commons::SingleRoboticSystem srs_check_ = crs_.getSingleRoboticSystem(srs_in_.getName()).second;

  ASSERT_EQ(srs_in_, srs_check_);
}

int main(int argc, char* argv[])
{
  testing::InitGoogleTest(&argc, argv);

  ros::init(argc, argv, "collective_robotic_system_test_node");

  std::thread t([] {
    while (ros::ok())
      ros::spin();
  });
  auto res = RUN_ALL_TESTS();
  ros::shutdown();
  return res;
}
