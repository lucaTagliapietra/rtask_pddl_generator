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
  std::string property_name;
  double property_value;

  rtask_msgs::Property property_msg;
  rtask::commons::Property property;

  ros::NodeHandle m_nh;

  PropertyTest()
    : m_nh("~")
  {

    property_name = "robot_weight";
    property_value = 18.4;

    property.setProperty(property_name, property_value);
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

TEST_F(PropertyTest, fromXml)
{

  XmlRpc::XmlRpcValue property_description;
  rtask::commons::Property property_check;

  m_nh.getParam("property_description", property_description);

  property_check.setPropertyFromXmlRpc(property_description);

  ASSERT_TRUE(property_check == property);
}

TEST_F(PropertyTest, isValid)
{
  ASSERT_EQ(property.isValid(), true);
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
