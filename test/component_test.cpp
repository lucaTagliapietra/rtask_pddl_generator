#include "commons/component.h"
#include <chrono>
#include <gtest/gtest.h>
#include <ros/ros.h>
#include <thread>

class ComponentTest : public ::testing::Test
{
public:
  struct CapacitySort
  {
    // overload the function call operator
    bool operator()(const rtask::commons::Capacity& capacity1, const rtask::commons::Capacity& capacity2) const
    {
      return capacity1.getCapabilityName() < capacity2.getCapabilityName();
    }
  };

  struct PropertySort
  {
    // overload the function call operator
    bool operator()(const rtask::commons::Property& property1, const rtask::commons::Property& property2) const
    {
      return property1.getName() < property2.getName();
    }
  };

  unsigned int id;
  std::string name, type, description, urdf_link, moveit_group_name, reference_frame, parent_link;

  std::vector<rtask::commons::Capacity> capacities;
  std::vector<rtask::commons::Property> properties1, properties2;

  rtask::commons::Property property1, property2, property3;
  rtask::commons::Capacity capacity1, capacity2;

  std::string property1_name, property2_name, property3_name;
  std::string capacity1_name, capacity2_name;

  rtask_msgs::Component component_msg;
  rtask::commons::Component component;

  ComponentTest()
  {

    id = 0;
    name = "ur5";
    type = "Universal Robots UR5";
    reference_frame = "world";
    description = "URDF description and properties for Universal UR5 robot arm";
    urdf_link = "https://raw.githubusercontent.com/ros-industrial/universal_robot/kinetic-devel/ur_description/urdf/"
                "ur5_robot.urdf.xacro";
    moveit_group_name = "manipulator";
    parent_link = "world";

    property1_name = "weight";
    property2_name = "payload";
    property3_name = "reach";

    property1.setName(property1_name);
    property1.setValue(18.4);

    property2.setName(property2_name);
    property2.setValue(5);

    property3.setName(property3_name);
    property3.setValue(850);

    properties1.push_back(property1);
    properties1.push_back(property2);

    properties2.push_back(property3);

    capacity1_name = "Manipulation";
    capacity2_name = "Move";

    capacity1.setCapacity(capacity1_name, properties1);
    capacity2.setCapacity(capacity2_name, properties2);

    capacities.push_back(capacity1);
    capacities.push_back(capacity2);

    component.setComponent(id, name, type, reference_frame, capacities);

    component.setDescription(description);
    component.setUrdfLink(urdf_link);
    component.setMoveitGroupName(moveit_group_name);
    component.setParentLink(parent_link);
  }

  ~ComponentTest() {}

  inline bool areCapacitiesVectorsEqual(const std::vector<rtask::commons::Capacity>& capacity1,
                                        const std::vector<rtask::commons::Capacity>& capacity2)
  {
    bool result = false;
    for (auto c1 : capacity1)
      for (auto c2 : capacity2)
        if (c1.operator==(c2))
          result = true;
    return result;
  }

  inline bool arePropertiesVectorsEqual(const std::vector<rtask::commons::Property>& property1,
                                        const std::vector<rtask::commons::Property>& property2)
  {
    bool result = false;
    for (auto p1 : property1)
      for (auto p2 : property2)
        if (p1.operator==(p2))
          result = true;
    return result;
  }
};

// ---------------
// Component level
// ---------------

TEST_F(ComponentTest, getDescriptionAndProperties)
{
  std::vector<rtask::commons::Capacity> capacities_output;
  std::vector<std::string> capabilities_check, capabilities_output;

  component.getCapacities(capacities_output);
  component.getCapabilities(capabilities_output);

  for (auto cap : capacities)
    capabilities_check.push_back(cap.getCapabilityName());

  ASSERT_EQ(component.getId(), id);
  ASSERT_EQ(component.getName(), name);
  ASSERT_EQ(component.getType(), type);
  ASSERT_EQ(component.getReferenceFrame(), reference_frame);
  ASSERT_EQ(component.getDescription(), description);
  ASSERT_EQ(component.getUrdfLink(), urdf_link);
  ASSERT_EQ(component.getMoveitGroupName(), moveit_group_name);
  ASSERT_EQ(component.getParentLink(), parent_link);
  ASSERT_TRUE(areCapacitiesVectorsEqual(capacities_output, capacities));
  ASSERT_EQ(capabilities_output, capabilities_check);
}

TEST_F(ComponentTest, toAndFromComponentMsg)
{

  rtask_msgs::ComponentPtr component_msg_ptr = component.toComponentMsg();

  rtask::commons::Component component_output;
  component_output.setFromComponentMsg(component_msg_ptr);

  ASSERT_TRUE(component_output == component); // TRUE if same name and capacities
}

TEST_F(ComponentTest, clear)
{

  component.clear();

  ASSERT_TRUE(component.getName() == "");
}

// --------------
// Capacity Level
// --------------

TEST_F(ComponentTest, capacityTest)
{
  std::vector<rtask::commons::Capacity> capacities_check, capacities_output;
  rtask::commons::Capacity capacity_output;
  std::vector<rtask::commons::Property> properties_output;

  component.getCapacities(capacities_check);

  ASSERT_TRUE(component.hasCapacity(capacity1_name));

  capacities_check.erase(std::find(capacities_check.begin(), capacities_check.end(), capacity1));
  std::sort(capacities_check.begin(), capacities_check.end(), CapacitySort());
  component.removeCapacity(capacity1.getCapabilityName());
  component.getCapacities(capacities_output);
  ASSERT_TRUE(areCapacitiesVectorsEqual(capacities_check, capacities_output));

  component.getCapacity(capacity2_name, capacity_output);
  ASSERT_TRUE(capacity_output == capacity2);

  component.getCapacityProperties(capacity2_name, properties_output);
  ASSERT_TRUE(arePropertiesVectorsEqual(properties_output, properties2));

  capacities_check.push_back(capacity1);
  std::sort(capacities_check.begin(), capacities_check.end(), CapacitySort());
  component.addCapacity(capacity1);
  component.getCapacities(capacities_output);
  ASSERT_TRUE(areCapacitiesVectorsEqual(capacities_check, capacities_output));
}

// --------------
// Property level
// --------------

TEST_F(ComponentTest, propertyTest)
{

  std::vector<rtask::commons::Property> properties_check, properties_output;
  rtask::commons::Property property_output;

  component.getCapacityProperties(capacity1_name, properties_check);

  ASSERT_TRUE(component.hasCapacityProperty(capacity1_name, property1_name));

  properties_check.erase(std::find(properties_check.begin(), properties_check.end(), property1));
  std::sort(properties_check.begin(), properties_check.end(), PropertySort());
  component.removeCapacityProperty(capacity1_name, property1_name);
  component.getCapacityProperties(capacity1_name, properties_output);
  ASSERT_TRUE(arePropertiesVectorsEqual(properties_check, properties_output));

  component.getCapacityProperty(capacity1_name, property2_name, property_output);
  ASSERT_TRUE(property_output == property2);

  properties_check.push_back(property1);
  std::sort(properties_check.begin(), properties_check.end(), PropertySort());
  component.addCapacityProperty(capacity1_name, property1);
  component.getCapacityProperties(capacity1_name, properties_output);
  ASSERT_TRUE(arePropertiesVectorsEqual(properties_check, properties_output));

  // setCapacityProperty
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
