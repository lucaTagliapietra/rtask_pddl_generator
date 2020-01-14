#include "commons/component.h"
#include <chrono>
#include <gtest/gtest.h>
#include <ros/ros.h>
#include <thread>

class ComponentTest : public ::testing::Test
{
public:
  unsigned int id;
  std::string name, model, manufacturer, description, urdf_link, moveit_group_name, reference_frame, parent_link;

  std::vector<rtask::commons::Capacity> capacities;
  std::vector<rtask::commons::Property> properties1, properties2;

  rtask::commons::Property property1, property2, property3, property4;
  rtask::commons::Capacity capacity1, capacity2;

  std::string property1_name, property2_name, property3_name, property4_name;
  std::string capacity1_name, capacity2_name;

  rtask_msgs::Component component_msg;
  rtask::commons::Component component;

  ros::NodeHandle m_nh;

  ComponentTest()
    : m_nh("~")
  {

    id = 0;
    name = "component_0";
    model = "ur5";
    manufacturer = "universal_robot";
    reference_frame = "world";
    description = "URDF description and properties of the Universal Robot UR5 robot arm";
    urdf_link = "https://raw.githubusercontent.com/ros-industrial/universal_robot/kinetic-devel/ur_description/urdf/"
                "ur5_robot.urdf.xacro";
    moveit_group_name = "manipulator";
    parent_link = "world";

    property1_name = "robot_weight";
    property2_name = "robot_payload";
    property3_name = "robot_ip_classification";
    property4_name = "robot_force_sensor";

    property1.setProperty(property1_name, 18.4);

    property2.setProperty(property2_name, 5);

    property3.setProperty(property3_name, std::string{"ip54"});

    property4.setProperty(property4_name, true);

    properties1.push_back(property1);
    properties1.push_back(property2);
    properties1.push_back(property3);

    properties2.push_back(property4);

    capacity1_name = "manipulation";
    capacity2_name = "human_collaboration";

    capacity1.setCapacity(capacity1_name, properties1);
    capacity2.setCapacity(capacity2_name, properties2);

    capacities.push_back(capacity1);

    component.setComponent(id, name, model, manufacturer, reference_frame, capacities);
    component.setCapacityProperty(capacity2_name, property4);

    capacities.push_back(capacity2);

    component.setDescription(description);
    component.setUrdfLink(urdf_link);
    component.setMoveitGroupName(moveit_group_name);
    component.setParentLink(parent_link);
  }

  ~ComponentTest() {}

  inline bool areCapacitiesVectorsEqual(const std::vector<rtask::commons::Capacity>& capacity1,
                                        const std::vector<rtask::commons::Capacity>& capacity2)
  {

    return ((capacity1.size() == capacity2.size())
            && std::is_permutation(capacity1.begin(), capacity1.end(), capacity2.begin()));
  }

  inline bool arePropertiesVectorsEqual(const std::vector<rtask::commons::Property>& property1,
                                        const std::vector<rtask::commons::Property>& property2)
  {
    return ((property1.size() == property2.size())
            && std::is_permutation(property1.begin(), property1.end(), property2.begin()));
  }
};

// ---------------
// Component level
// ---------------

TEST_F(ComponentTest, componentTest)
{
  std::vector<rtask::commons::Capacity> capacities_output;

  component.getCapacities(capacities_output);

  ASSERT_EQ(component.isValid(), true);

  ASSERT_EQ(component.getId(), id);
  ASSERT_EQ(component.getName(), name);
  ASSERT_EQ(component.getModel(), model);
  ASSERT_EQ(component.getManufacturer(), manufacturer);
  ASSERT_EQ(component.getReferenceFrame(), reference_frame);
  ASSERT_EQ(component.getDescription(), description);
  ASSERT_EQ(component.getUrdfLink(), urdf_link);
  ASSERT_EQ(component.getMoveitGroupName(), moveit_group_name);
  ASSERT_EQ(component.getParentLink(), parent_link);

  ASSERT_TRUE(areCapacitiesVectorsEqual(capacities, capacities_output));

  XmlRpc::XmlRpcValue component_description;
  rtask::commons::Component component_check;
  m_nh.getParam("component_description", component_description);
  component_check.setComponentFromXmlRpc(component_description);
  ASSERT_TRUE(component_check == component);

  component_check.clear();
  rtask_msgs::ComponentPtr component_msg_ptr = component.toComponentMsg();
  component_check.setFromComponentMsg(component_msg_ptr);
  ASSERT_TRUE(component_check == component);

  std::vector<std::string> capabilities_check, capabilities_output;
  component_check.getCapabilities(capabilities_check);
  component.getCapabilities(capabilities_output);

  ASSERT_EQ(capabilities_check, capabilities_output);

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
  component.removeCapacity(capacity1.getCapabilityName());
  component.getCapacities(capacities_output);
  ASSERT_TRUE(areCapacitiesVectorsEqual(capacities_check, capacities_output));

  component.getCapacity(capacity2_name, capacity_output);
  ASSERT_TRUE(capacity_output == capacity2);

  component.getCapacityProperties(capacity2_name, properties_output);
  ASSERT_TRUE(arePropertiesVectorsEqual(properties_output, properties2));

  capacities_output.clear();
  capacities_check.push_back(capacity1);
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
  rtask::commons::Property property_output, property_check;

  component.getCapacityProperties(capacity1_name, properties_check);

  ASSERT_TRUE(component.hasCapacityProperty(capacity1_name, property1_name));

  properties_check.erase(std::find(properties_check.begin(), properties_check.end(), property1));
  component.removeCapacityProperty(capacity1_name, property1_name);
  component.getCapacityProperties(capacity1_name, properties_output);
  ASSERT_TRUE(arePropertiesVectorsEqual(properties_check, properties_output));

  component.getCapacityProperty(capacity1_name, property2_name, property_output);
  ASSERT_TRUE(property_output == property2);

  properties_output.clear();
  properties_check.push_back(property1);
  component.addCapacityProperty(capacity1_name, property1);
  component.getCapacityProperties(capacity1_name, properties_output);
  ASSERT_TRUE(arePropertiesVectorsEqual(properties_check, properties_output));
}

int main(int argc, char* argv[])
{
  testing::InitGoogleTest(&argc, argv);

  ros::init(argc, argv, "component_test_node");

  std::thread t([] {
    while (ros::ok())
      ros::spin();
  });
  auto res = RUN_ALL_TESTS();
  ros::shutdown();
  return res;
}
