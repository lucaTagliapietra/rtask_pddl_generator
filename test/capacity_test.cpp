#include "commons/capacity.h"
#include <chrono>
#include <gtest/gtest.h>
#include <ros/ros.h>
#include <thread>

class CapacityTest : public ::testing::Test
{
public:
  std::string capability_name = "";

  std::vector<rtask::commons::Property> properties{};

  rtask::commons::Property prop1, prop2, prop3;
  std::string prop_name1, prop_name2, prop_name3;
  double value1;
  int value2, value3;

  rtask_msgs::Capacity capacity_msg;
  rtask_msgs::Property prop1_msg, prop2_msg, prop3_msg;

  rtask::commons::Capacity cap;

  ros::NodeHandle m_nh;

  CapacityTest()
    : m_nh("~")
  {

    prop_name1 = "robot_weight";
    value1 = 18.4;

    prop1_msg.name = prop_name1;
    prop1_msg.type = rtask_msgs::Property::DOUBLE;
    prop1_msg.double_value = value1;

    prop1.setFromPropertyMsg(prop1_msg);

    prop_name2 = "robot_payload";
    value2 = 5;

    prop2_msg.name = prop_name2;
    prop2_msg.type = rtask_msgs::Property::INTEGER;
    prop2_msg.int_value = value2;

    prop2.setFromPropertyMsg(prop2_msg);

    prop_name3 = "robot_reach";
    value3 = 850;

    prop3_msg.name = prop_name3;
    prop3_msg.type = rtask_msgs::Property::INTEGER;
    prop3_msg.double_value = value3;

    prop3.setFromPropertyMsg(prop3_msg);

    capability_name = "manipulation";

    properties.push_back(prop1);
    properties.push_back(prop2);

    capacity_msg.capability = capability_name;
    capacity_msg.properties.push_back(prop1_msg);
    capacity_msg.properties.push_back(prop2_msg);

    cap.setCapacity(capability_name, properties);
  }

  ~CapacityTest() {}

  inline bool arePropertiesVectorsEqual(const std::vector<rtask::commons::Property>& property1,
                                        const std::vector<rtask::commons::Property>& property2)
  {
    return ((property1.size() == property2.size())
            && std::is_permutation(property1.begin(), property1.end(), property2.begin()));
  }
};

TEST_F(CapacityTest, sameCapacity)
{
  rtask::commons::Capacity dual_capacity = cap;
  ASSERT_TRUE(dual_capacity == cap);
}

TEST_F(CapacityTest, fromXml)
{

  XmlRpc::XmlRpcValue capacity_description;
  rtask::commons::Capacity capacity_check;

  m_nh.getParam("capacity_description", capacity_description);

  capacity_check.setCapacityFromXmlRpc(capacity_description);

  ASSERT_TRUE(capacity_check == cap);
}

TEST_F(CapacityTest, getCapabilityName)
{
  ASSERT_EQ(cap.getCapabilityName(), capability_name);
}

TEST_F(CapacityTest, hasProperty)
{
  ASSERT_TRUE(cap.hasProperty(prop_name1));
}

TEST_F(CapacityTest, getProperty)
{
  rtask::commons::Property property_output;
  double value;
  cap.getProperty(prop_name1, property_output);
  property_output.getValue(value);

  ASSERT_EQ(property_output.getType(), prop1.getType());
  ASSERT_EQ(value, value1);
}

TEST_F(CapacityTest, addPropertyAndList)
{
  std::vector<rtask::commons::Property> properties_output;

  properties.push_back(prop3);

  cap.addProperty(prop3);

  cap.getProperties(properties_output);

  ASSERT_TRUE(arePropertiesVectorsEqual(properties_output, properties));
}

TEST_F(CapacityTest, removePropertyAndList)
{
  std::vector<rtask::commons::Property> properties_output;

  properties.erase(std::find(properties.begin(), properties.end(), prop1));

  cap.removeProperty(prop1.getName());

  cap.getProperties(properties_output);

  ASSERT_TRUE(arePropertiesVectorsEqual(properties_output, properties));
}

TEST_F(CapacityTest, toAndFromCapacityMsg)
{
  rtask_msgs::CapacityPtr capacity_msg_ptr = cap.toCapacityMsg();

  rtask::commons::Capacity capacity_check;
  capacity_check.setFromCapacityMsg(capacity_msg_ptr);

  ASSERT_TRUE(capacity_check == cap);
}

TEST_F(CapacityTest, isValid)
{
  ASSERT_EQ(cap.isValid(), true);
}

TEST_F(CapacityTest, clear)
{

  cap.clear();

  ASSERT_TRUE(cap.getCapabilityName() == "");
}

int main(int argc, char* argv[])
{
  testing::InitGoogleTest(&argc, argv);

  ros::init(argc, argv, "capacity_test_node");

  std::thread t([] {
    while (ros::ok())
      ros::spin();
  });
  auto res = RUN_ALL_TESTS();
  ros::shutdown();
  return res;
}
