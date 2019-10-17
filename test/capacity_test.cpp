#include "commons/capacity.h"
#include <chrono>
#include <gtest/gtest.h>
#include <ros/ros.h>
#include <thread>

class CapacityTest : public ::testing::Test
{
public:
  struct Sort
  {
    // overload the function call operator
    bool operator()(const rtask::commons::Property& property1, const rtask::commons::Property& property2) const
    {
      return property1.getName() < property2.getName();
    }
  };

  std::string capability_name = "";

  std::vector<rtask::commons::Property> properties{};
  std::vector<rtask::commons::Property> properties_output;

  rtask::commons::Property prop1, prop2, prop3;
  std::string prop_name1, prop_name2, prop_name3;
  int value1, value2;
  double value3;

  rtask_msgs::Capacity capacity_msg;
  rtask_msgs::Property prop1_msg, prop2_msg, prop3_msg;

  rtask::commons::Capacity cap;

  CapacityTest()
  {

    prop_name1 = "robot_payload";
    value1 = 10;

    prop1_msg.name = prop_name1;
    prop1_msg.type = rtask_msgs::Property::INTEGER;
    prop1_msg.int_value = value1;

    prop1.setFromPropertyMsg(prop1_msg);

    prop_name2 = "gripper_payload";
    value2 = 5;

    prop2_msg.name = prop_name2;
    prop2_msg.type = rtask_msgs::Property::INTEGER;
    prop2_msg.int_value = value2;

    prop2.setFromPropertyMsg(prop2_msg);

    prop_name3 = "gripper_max_opening";
    value3 = 0.1;

    prop3_msg.name = prop_name3;
    prop3_msg.type = rtask_msgs::Property::DOUBLE;
    prop3_msg.double_value = value3;

    prop3.setFromPropertyMsg(prop3_msg);

    capability_name = "Manipulation";

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
    bool result = false;
    for (auto it1 = property1.begin(); it1 != property1.end(); ++it1)
      for (auto it2 = property2.begin(); it2 != property2.end(); ++it2)
        if (it1->getName() == it2->getName())
          result = true;
    return result;
  }
};

TEST_F(CapacityTest, sameCapacity)
{
  ASSERT_TRUE(cap == cap);
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
  rtask::commons::Property prop;
  int value;
  cap.getProperty(prop_name1, prop);
  prop.getValue(value);

  ASSERT_EQ(prop.getType(), prop1.getType());
  ASSERT_EQ(value, value1);
}

TEST_F(CapacityTest, addPropertyAndList)
{
  properties.push_back(prop3);
  std::sort(properties.begin(), properties.end(), Sort());

  cap.addProperty(prop3);

  cap.getProperties(properties_output);

  ASSERT_TRUE(arePropertiesVectorsEqual(properties_output, properties));
}

TEST_F(CapacityTest, removePropertyAndList)
{
  properties.erase(std::find(properties.begin(), properties.end(), prop1));
  std::sort(properties.begin(), properties.end(), Sort());

  cap.removeProperty(prop1.getName());

  cap.getProperties(properties_output);

  ASSERT_TRUE(arePropertiesVectorsEqual(properties_output, properties));
}

TEST_F(CapacityTest, toCapacityMsg)
{

  // boost::shared_ptr<rtask::commons::Capacity> c = boost::make_shared<rtask::commons::Capacity>();
  // rtask_msgs::CapacityPtr capacity = cap.toCapacityMsg();
  //  //  ROS_ERROR_STREAM(capa.capability << " " << cap.getCapabilityName());
  //  ASSERT_TRUE(true);
  //  // ASSERT_EQ(capa->capability, capability_name);
  //  // capacity = *capacity_msg_output;
  // ASSERT_EQ(capacity_msg.capability, capability_name);
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
