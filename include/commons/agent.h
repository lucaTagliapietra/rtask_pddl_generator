#ifndef rtask_commons_agent_h
#define rtask_commons_agent_h

#include <limits>
#include <map>
#include <string>
#include <vector>

#include "rtask_msgs/Agent.h"

#include "component.h"

namespace rtask {
  namespace commons {
    class Agent
    {
    public:
      Agent() {}
      Agent(const unsigned int t_id,
            const std::string& t_name,
            const std::vector<Component> t_comps = {},
            const std::string& t_description = "",
            const std::string& t_urdf_link = "",
            const std::string& t_base_link = "");
      Agent(const rtask_msgs::Agent& t_msg);
      Agent(const rtask_msgs::AgentConstPtr& t_msg_ptr);
      Agent(XmlRpc::XmlRpcValue& t_node);

      ~Agent() {}

      // -----------
      // Agent Level
      // -----------
      rtask_msgs::AgentPtr toAgentMsg() const;

      bool setAgentFromXmlRpc(XmlRpc::XmlRpcValue& t_node);
      void setFromAgentMsg(const rtask_msgs::Agent& t_msg);
      void setFromAgentMsg(const rtask_msgs::AgentConstPtr t_msg_ptr);
      void setAgent(const unsigned int t_id,
                    const std::string& t_name,
                    const std::vector<Component> t_comps = {},
                    const std::string& t_description = "",
                    const std::string& t_urdf_link = "",
                    const std::string& t_base_link = "");
      void setDescription(const std::string& t_value) { m_params.description = t_value; }
      void setUrdfLink(const std::string& t_value) { m_params.urdf_link = t_value; }
      void setBaseLink(const std::string& t_value) { m_params.base_link = t_value; }
      void clear();

      bool isValid() const { return m_params.valid; }
      unsigned int getId() const { return m_params.id; }
      std::string getName() const { return m_params.name; }
      std::string getDescription() const { return m_params.description; }
      std::string getUrdfLink() const { return m_params.urdf_link; }
      std::string getBaseLink() const { return m_params.base_link; }
      void getComponents(std::vector<Component>& t_components) const;
      void getComponentNames(std::vector<std::string>& t_component_names) const;
      void getCapabilities(std::vector<std::string>& t_capabilities) const;
      bool hasCapability(const std::string& t_capability) const;

      // ---------------
      // Component Level
      // ---------------
      bool hasComponent(const std::string& t_component_name) const;
      bool removeComponent(const std::string& t_component_name);
      bool getComponent(const std::string& t_component_name, Component& t_component) const;
      bool addComponent(const Component& t_component);
      void setComponent(const Component& t_component);

      bool getComponentId(const std::string& t_component_name, unsigned int& t_id) const;
      bool getComponentModel(const std::string& t_component_name, std::string& t_model) const;
      bool getComponentManufacturer(const std::string& t_component_name, std::string& t_manufacturer) const;
      bool getComponentDescription(const std::string& t_component_name, std::string& t_description) const;
      bool getComponentUrdfLink(const std::string& t_component_name, std::string& t_urdf_link) const;
      bool getComponentMoveitGroupName(const std::string& t_component_name, std::string& t_moveit_group) const;
      bool getComponentReferenceFrame(const std::string& t_component_name, std::string& t_ref_frame) const;
      bool getComponentParentLink(const std::string& t_component_name, std::string& t_parent_link) const;
      bool getComponentCapacities(const std::string& t_component_name, std::vector<Capacity>& t_capacities) const;
      bool getComponentCapabilities(const std::string& t_component_name,
                                    std::vector<std::string>& t_capacity_names) const;

      // --------------
      // Capacity Level
      // --------------
      bool hasComponentCapacity(const std::string& t_component_name, const std::string& t_capacity_name) const;
      bool removeComponentCapacity(const std::string& t_component_name, const std::string& t_capacity_name);
      bool getComponentCapacity(const std::string& t_component_name,
                                const std::string& t_capacity_name,
                                Capacity& t_capacity) const;
      bool getComponentCapacityProperties(const std::string& t_component_name,
                                          const std::string& t_capacity_name,
                                          std::vector<Property>& t_props) const;
      bool addComponentCapacity(const std::string& t_component_name, const Capacity& t_capacity);
      bool setComponentCapacity(const std::string& t_component_name, const Capacity& t_capacity);

      // --------------
      // Property level
      // --------------
      bool hasComponentCapacityProperty(const std::string& t_component_name,
                                        const std::string& t_capacity_name,
                                        const std::string& t_prop_name) const;
      bool removeComponentCapacityProperty(const std::string& t_component_name,
                                           const std::string& t_capacity_name,
                                           const std::string& t_prop_name);
      bool getComponentCapacityProperty(const std::string& t_component_name,
                                        const std::string& t_capacity_name,
                                        const std::string& t_prop_name,
                                        Property& t_prop) const;
      bool addComponentCapacityProperty(const std::string& t_component_name,
                                        const std::string& t_capacity_name,
                                        const Property& t_prop);
      bool setComponentCapacityProperty(const std::string& t_component_name,
                                        const std::string& t_capacity_name,
                                        const Property& t_prop);

      // ---------
      // Operators
      // ---------

      bool operator==(const Agent& t_agent);

    private:
      struct AgentParameters
      {
        unsigned int id = std::numeric_limits<unsigned int>::quiet_NaN();
        std::string name = "";
        std::string description = "";
        std::string urdf_link = "";
        std::string base_link = "";
        bool valid = false;

        std::map<std::string, Component> components{};
      };

      AgentParameters m_params;
      std::map<std::string, std::vector<std::string>> m_capability_to_component{};

      void clearComponents();

      void updCapabilityMap();
      void updCapabilityMap(const Component& t_comp);
      bool updValidity();
    };
  } // namespace commons
} // namespace rtask

#endif // rtask_commons_agent_h
