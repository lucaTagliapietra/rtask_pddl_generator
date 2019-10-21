#include "commons/property.h"
#include <chrono>
#include <gtest/gtest.h>
#include <ros/ros.h>
#include <thread>

class PropertyTest : public ::testing::Test
{
public:
  std::string property_name;
  double property_value;

  rtask_msgs::Property property_msg;
  rtask::commons::Property property;

  PropertyTest()
  {

    property_name = "gripper_max_opening";
    property_value = 0.10;

    property.setName(property_name);
    property.setValue(property_value);
  }

  ~PropertyTest() {}
};

TEST_F(PropertyTest, sameProperty)
{
  rtask::commons::Property dual_property = property;
  ASSERT_TRUE(dual_property == property);
}

TEST_F(PropertyTest, getName)
{
  ASSERT_EQ(property.getName(), property_name);
}

TEST_F(PropertyTest, getType)
{
  ASSERT_EQ(property.getType(), rtask::commons::Property::Type::Double);
}

TEST_F(PropertyTest, getValue)
{
  double property_value_output;
  property.getValue(property_value_output);
  ASSERT_EQ(property_value_output, property_value);
}

TEST_F(PropertyTest, toAndFromPropertyMsg)
{
  rtask_msgs::PropertyPtr property_msg_ptr = property.toPropertyMsg();

  rtask::commons::Property property_check;
  property_check.setFromPropertyMsg(property_msg_ptr);

  ASSERT_TRUE(property_check == property);
}

int main(int argc, char* argv[])
{
  testing::InitGoogleTest(&argc, argv);

  std::thread t([] {
    while (ros::ok())
      ros::spin();
  });
  auto res = RUN_ALL_TESTS();
  ros::shutdown();
  return res;
}
