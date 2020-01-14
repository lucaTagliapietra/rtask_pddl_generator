#include "commons/agent.h"
#include <chrono>
#include <gtest/gtest.h>
#include <ros/ros.h>
#include <thread>

class AgentTest : public ::testing::Test
{
public:
  struct ComponentSort
  {
    // overload the function call operator
    bool operator()(const rtask::commons::Component& component1, const rtask::commons::Component& component2) const
    {
      return component1.getName() < component2.getName();
    }
  };

  struct CapacitySort
  {
    // overload the function call operator
    bool operator()(const rtask::commons::Capacity& capacity1, const rtask::commons::Capacity& capacity2) const
    {
      return capacity1.getCapabilityName() < capacity2.getCapabilityName();
    }
  };

  unsigned int id;
  std::string name, description, urdf_link, base_link;

  rtask::commons::Component component1, component2, component3;
  std::vector<rtask::commons::Component> components;

  rtask::commons::Property property1, property2, property3;
  rtask::commons::Capacity capacity1, capacity2, capacity3, capacity4;
  std::vector<rtask::commons::Property> properties_component1, properties_component2, properties_component3;
  std::vector<rtask::commons::Capacity> capacities_component1, capacities_component2, capacities_component3;

  rtask::commons::Agent agent;

  AgentTest()
  {
    id = 0;
    name = "rur53";
    description = "URDF description and properties for a UGV composed of a Robotnik Summit XL HL mobile base, "
                  "a Universal Robot UR5 manipulator, and a Robotiq 3-Finger adaptive gripper ";
    base_link = "world";

    property1.setName("arm_payload");
    property1.setValue(10);
    properties_component1.push_back(property1);
    capacity1.setCapacity("Manipulation", properties_component1);
    capacity2.setCapacity("Move", properties_component1);

    capacities_component1.push_back(capacity1);
    capacities_component1.push_back(capacity2);

    component1.setComponent(0, "arm", "Universal Robot UR5", "base_link", capacities_component1);

    property2.setName("gripper_payload");
    property2.setValue(10);
    properties_component2.push_back(property2);
    capacity3.setCapacity("Manipulation", properties_component2);
    capacities_component2.push_back(capacity3);

    component2.setComponent(1, "gripper", "Robotiq 3-Finger Adaptive Gripper", "base_link", capacities_component2);

    property3.setName("base_payload");
    property3.setValue(65);
    properties_component3.push_back(property3);
    capacity4.setCapacity("Move", properties_component3);
    capacities_component3.push_back(capacity4);

    component3.setComponent(2, "base", "Robotnik Summit XL HL", "base_link", capacities_component3);

    components.push_back(component1);
    components.push_back(component2);
    components.push_back(component3);

    agent.setAgent(id, name, components);

    agent.setDescription(description);
    agent.setBaseLink(base_link);
  }

  ~AgentTest() {}

  inline bool areComponentsVectorsEqual(const std::vector<rtask::commons::Component>& comp1,
                                        const std::vector<rtask::commons::Component>& comp2)
  {
    bool result = false;
    for (auto c1 : comp1)
      for (auto c2 : comp2)
        if (c1.operator==(c2))
          result = true;
    return result;
  }

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
// Agent level
// ---------------

TEST_F(AgentTest, getDescriptionAndProperties)
{
  std::vector<rtask::commons::Component> components_output;
  std::vector<std::string> capabilities_output, capabilities_check, component_names_output, component_names_check;

  agent.getComponents(components_output);
  ASSERT_TRUE(areComponentsVectorsEqual(components_output, components));

  for (auto cap : capacities_component1)
    capabilities_check.push_back(cap.getCapabilityName());

  for (auto cap : capacities_component2)
    capabilities_check.push_back(cap.getCapabilityName());

  for (auto cap : capacities_component3)
    capabilities_check.push_back(cap.getCapabilityName());

  std::sort(capabilities_check.begin(), capabilities_check.end());
  capabilities_check.erase(std::unique(capabilities_check.begin(), capabilities_check.end()), capabilities_check.end());

  agent.getCapabilities(capabilities_output);

  ASSERT_EQ(capabilities_output, capabilities_check);

  ASSERT_EQ(agent.getId(), id);
  ASSERT_EQ(agent.getName(), name);
  ASSERT_EQ(agent.getDescription(), description);
  ASSERT_EQ(agent.getBaseLink(), base_link);

  for (auto comp : components)
    component_names_check.push_back(comp.getName());

  std::sort(component_names_check.begin(), component_names_check.end());

  agent.getComponentNames(component_names_output);

  ASSERT_EQ(component_names_output, component_names_check);

  ASSERT_TRUE(agent.hasCapability(capacity1.getCapabilityName()));
}

TEST_F(AgentTest, toAndFromAgentMsg)
{
  rtask::commons::Agent agent_output;

  auto agent_msg_ptr = agent.toAgentMsg();

  agent_output.setFromAgentMsg(agent_msg_ptr);

  ASSERT_TRUE(agent_output == agent); // TRUE if same name and components
}

TEST_F(AgentTest, clear)
{

  agent.clear();

  ASSERT_TRUE(agent.getName() == "");
}

// ---------------
// Component level
// ---------------

TEST_F(AgentTest, componentTest)
{
  std::vector<rtask::commons::Component> components_check, components_output;
  rtask::commons::Component component_output;
  unsigned int id_output;
  std::string type_output, reference_frame_output;
  std::vector<rtask::commons::Capacity> capacities_check, capacities_output;
  std::vector<std::string> capabilities_check, capabilities_output;

  ASSERT_TRUE(agent.hasComponent(component1.getName()));

  agent.getComponents(components_check);
  components_check.erase(std::find(components_check.begin(), components_check.end(), component1));
  std::sort(components_check.begin(), components_check.end(), ComponentSort());
  agent.removeComponent(component1.getName());
  agent.getComponents(components_output);
  ASSERT_TRUE(areComponentsVectorsEqual(components_output, components_check));

  agent.getComponent(component2.getName(), component_output);
  ASSERT_TRUE(component_output == component2);

  component_output.clear();
  components_check.push_back(component1);
  std::sort(components_check.begin(), components_check.end(), ComponentSort());
  agent.addComponent(component1);
  agent.getComponents(components_output);
  ASSERT_TRUE(areComponentsVectorsEqual(components_output, components_check));

  agent.getComponentId(component1.getName(), id_output);
  ASSERT_EQ(id_output, component1.getId());

  agent.getComponentType(component1.getName(), type_output);
  ASSERT_EQ(type_output, component1.getType());

  agent.getComponentReferenceFrame(component1.getName(), reference_frame_output);
  ASSERT_EQ(reference_frame_output, component1.getReferenceFrame());

  agent.getComponentCapacities(component1.getName(), capacities_output);
  component1.getCapacities(capacities_check);
  ASSERT_TRUE(areCapacitiesVectorsEqual(capacities_output, capacities_check));

  agent.getComponentCapabilities(component1.getName(), capabilities_output);
  component1.getCapabilities(capabilities_check);
  ASSERT_TRUE(capabilities_output == capabilities_check);
}

// ---------------
// Capacity level
// ---------------

TEST_F(AgentTest, capacityTest)
{
  std::vector<rtask::commons::Capacity> capacities_output, capacities_check;
  rtask::commons::Capacity capacity_output;
  std::vector<rtask::commons::Property> properties_output, properties_check;

  ASSERT_TRUE(agent.hasComponentCapacity(component1.getName(), capacity1.getCapabilityName()));

  agent.getComponentCapacities(component1.getName(), capacities_check);
  capacities_check.erase(std::find(capacities_check.begin(), capacities_check.end(), capacity1));
  std::sort(capacities_check.begin(), capacities_check.end(), CapacitySort());
  agent.removeComponentCapacity(component1.getName(), capacity1.getCapabilityName());
  agent.getComponentCapacities(component1.getName(), capacities_output);
  ASSERT_TRUE(areCapacitiesVectorsEqual(capacities_output, capacities_check));

  agent.getComponentCapacity(component1.getName(), capacity2.getCapabilityName(), capacity_output);
  ASSERT_TRUE(capacity_output == capacity2);

  agent.getComponentCapacityProperties(component1.getName(), capacity2.getCapabilityName(), properties_output);
  capacity2.getProperties(properties_check);
  ASSERT_TRUE(arePropertiesVectorsEqual(properties_output, properties_check));

  capacities_output.clear();
  capacities_check.push_back(capacity1);
  std::sort(capacities_check.begin(), capacities_check.end(), CapacitySort());
  agent.addComponentCapacity(component1.getName(), capacity1);
  agent.getComponentCapacities(component1.getName(), capacities_output);
  ASSERT_TRUE(areCapacitiesVectorsEqual(capacities_output, capacities_check));
}

// ---------------
// Property level
// ---------------

TEST_F(AgentTest, propertyTest) {}

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
