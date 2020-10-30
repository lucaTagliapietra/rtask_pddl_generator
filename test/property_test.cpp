#include "commons/property.h"
#include <chrono>
#include <gtest/gtest.h>
#include <ros/ros.h>
#include <thread>

#include <iostream> // std::cout
#include <sstream> // std::stringstream

class PropertyTest : public ::testing::Test
{
public:
  std::string bool_name_ = "bool_property";
  std::string int_name_ = "int_property";
  std::string double_name_ = "double_property";
  std::string string_name_ = "string_property";

  rtask::commons::PropertyVariant bool_value_ = false;
  rtask::commons::PropertyVariant int_value_ = 1;
  rtask::commons::PropertyVariant double_value_ = 0.5;
  rtask::commons::PropertyVariant string_value_ = std::string("string");

  rtask::commons::Property bool_prop_, int_prop_, double_prop_, string_prop_;

  ros::NodeHandle m_nh;

  PropertyTest()
    : m_nh("~")
  {

    bool_prop_ = {bool_name_, bool_value_};
    int_prop_ = {int_name_, int_value_};
    double_prop_ = {double_name_, double_value_};
    string_prop_ = {string_name_, string_value_};
  }

  ~PropertyTest() {}
};

TEST_F(PropertyTest, equalityOperator)
{
  rtask::commons::Property dual_property = bool_prop_;
  ASSERT_TRUE(dual_property == bool_prop_);
}

TEST_F(PropertyTest, getName)
{
  ASSERT_EQ(bool_prop_.getName(), bool_name_);
  ASSERT_EQ(int_prop_.getName(), int_name_);
  ASSERT_EQ(double_prop_.getName(), double_name_);
  ASSERT_EQ(string_prop_.getName(), string_name_);
}

TEST_F(PropertyTest, getType)
{
  ASSERT_EQ(bool_prop_.getType(), rtask::commons::PropertyType::BOOL);
  ASSERT_EQ(int_prop_.getType(), rtask::commons::PropertyType::INT);
  ASSERT_EQ(double_prop_.getType(), rtask::commons::PropertyType::DOUBLE);
  ASSERT_EQ(string_prop_.getType(), rtask::commons::PropertyType::STRING);
}

TEST_F(PropertyTest, getValue)
{
  std::pair<bool, rtask::commons::PropertyVariant> prop_value_out_;
  prop_value_out_ = bool_prop_.getValue();
  ASSERT_EQ(prop_value_out_.second, bool_value_);

  prop_value_out_ = int_prop_.getValue();
  ASSERT_EQ(prop_value_out_.second, int_value_);

  prop_value_out_ = double_prop_.getValue();
  ASSERT_EQ(prop_value_out_.second, double_value_);

  prop_value_out_ = string_prop_.getValue();
  ASSERT_EQ(prop_value_out_.second, string_value_);
}

TEST_F(PropertyTest, isValid)
{
  ASSERT_EQ(bool_prop_.isValid(), true);
  ASSERT_EQ(int_prop_.isValid(), true);
  ASSERT_EQ(double_prop_.isValid(), true);
  ASSERT_EQ(string_prop_.isValid(), true);
}

TEST_F(PropertyTest, toMsg)
{
  rtask_msgs::Property bool_prop_msg_, int_prop_msg_, double_prop_msg_, string_prop_msg_;
  rtask::commons::Property bool_prop_check_, int_prop_check_, double_prop_check_, string_prop_check_;

  bool_prop_msg_ = bool_prop_.toMsg();
  bool_prop_check_ = {bool_prop_msg_};

  int_prop_msg_ = int_prop_.toMsg();
  int_prop_check_ = {int_prop_msg_};

  double_prop_msg_ = double_prop_.toMsg();
  double_prop_check_ = {double_prop_msg_};

  string_prop_msg_ = string_prop_.toMsg();
  string_prop_check_ = {string_prop_msg_};

  ASSERT_TRUE(bool_prop_check_ == bool_prop_);
  ASSERT_TRUE(int_prop_check_ == int_prop_);
  ASSERT_TRUE(double_prop_check_ == double_prop_);
  ASSERT_TRUE(string_prop_check_ == string_prop_);
}

TEST_F(PropertyTest, set)
{
  rtask::commons::Property bool_prop_check_, int_prop_check_, double_prop_check_, string_prop_check_;

  bool_prop_check_.set(bool_name_, bool_value_);
  int_prop_check_.set(int_name_, int_value_);
  double_prop_check_.set(double_name_, double_value_);
  string_prop_check_.set(string_name_, string_value_);

  ASSERT_TRUE(bool_prop_check_ == bool_prop_);
  ASSERT_TRUE(int_prop_check_ == int_prop_);
  ASSERT_TRUE(double_prop_check_ == double_prop_);
  ASSERT_TRUE(string_prop_check_ == string_prop_);
}

TEST_F(PropertyTest, setValue)
{
  rtask::commons::Property bool_prop_check_, int_prop_check_, double_prop_check_, string_prop_check_;
  bool_prop_check_ = {bool_name_};
  int_prop_check_ = {int_name_};
  double_prop_check_ = {double_name_};
  string_prop_check_ = {string_name_};

  bool_prop_check_.setValue(bool_value_);
  int_prop_check_.setValue(int_value_);
  double_prop_check_.setValue(double_value_);
  string_prop_check_.setValue(string_value_);

  ASSERT_TRUE(bool_prop_check_ == bool_prop_);
  ASSERT_TRUE(int_prop_check_ == int_prop_);
  ASSERT_TRUE(double_prop_check_ == double_prop_);
  ASSERT_TRUE(string_prop_check_ == string_prop_);
}

TEST_F(PropertyTest, fromXmlRpc)
{
  XmlRpc::XmlRpcValue bool_prop_descr_, int_prop_descr_, double_prop_descr_, string_prop_descr_;
  rtask::commons::Property bool_prop_check_, int_prop_check_, double_prop_check_, string_prop_check_;

  m_nh.getParam("bool_property", bool_prop_descr_);
  m_nh.getParam("int_property", int_prop_descr_);
  m_nh.getParam("double_property", double_prop_descr_);
  m_nh.getParam("string_property", string_prop_descr_);

  bool_prop_check_ = rtask::commons::Property(bool_prop_descr_);
  int_prop_check_ = rtask::commons::Property(int_prop_descr_);
  double_prop_check_ = rtask::commons::Property(double_prop_descr_);
  string_prop_check_ = rtask::commons::Property(string_prop_descr_);

  // std::cerr << "bool_prop_check_: " << bool_prop_check_ << std::endl;
  // std::cerr << "bool_prop_: " << bool_prop_ << std::endl;

  ASSERT_TRUE(bool_prop_check_ == bool_prop_);
  ASSERT_TRUE(int_prop_check_ == int_prop_);
  ASSERT_TRUE(double_prop_check_ == double_prop_);
  ASSERT_TRUE(string_prop_check_ == string_prop_);
}

int main(int argc, char* argv[])
{
  testing::InitGoogleTest(&argc, argv);

  ros::init(argc, argv, "property_test_node");

  std::thread t([] {
    while (ros::ok())
      ros::spin();
  });
  auto res = RUN_ALL_TESTS();
  ros::shutdown();
  return res;
}
