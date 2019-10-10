#ifndef rtask_commons_component_h
#define rtask_commons_component_h

#include <limits>
#include <map>
#include <string>
#include <vector>

#include "rtask_msgs/Component.h"

#include "commons/capacity.h"

namespace rtask {
  namespace commons {
    class Component
    {
    public:
      Component() {}
      Component(const unsigned int t_id,
                const std::string& t_name,
                const std::string& t_type,
                const std::string& t_ref_frame,
                const std::vector<Capacity> t_capacities = {},
                const std::string& t_description = "",
                const std::string& t_urdf_link = "",
                const std::string& t_moveit_group_name = "",
                const std::string& t_parent_link = "");
      Component(const rtask_msgs::Component& t_msg);
      Component(const rtask_msgs::ComponentConstPtr t_msg_ptr);

      ~Component() {}

      // ---------------
      // Component level
      // ---------------
      rtask_msgs::ComponentPtr toComponentMsg() const;

      void setFromComponentMsg(const rtask_msgs::Component& t_msg);
      void setFromComponentMsg(const rtask_msgs::ComponentConstPtr t_msg_ptr);
      void setComponent(const unsigned int t_id,
                        const std::string& t_name,
                        const std::string& t_type,
                        const std::string& t_ref_frame,
                        const std::vector<Capacity> t_capacities = {},
                        const std::string& t_description = "",
                        const std::string& t_urdf_link = "",
                        const std::string& t_moveit_group_name = "",
                        const std::string& t_parent_link = "");
      void setDescription(const std::string& t_value) { m_params.description = t_value; }
      void setUrdfLink(const std::string& t_value) { m_params.urdf_link = t_value; }
      void setMoveitGroupName(const std::string& t_value) { m_params.moveit_group_name = t_value; }
      void setParentLink(const std::string& t_value) { m_params.parent_link = t_value; }
      void clear();

      unsigned int getId() const { return m_params.id; }
      std::string getName() const { return m_params.name; }
      std::string getType() const { return m_params.type; }
      std::string getDescription() const { return m_params.description; }
      std::string getUrdfLink() const { return m_params.urdf_link; }
      std::string getMoveitGroupName() const { return m_params.moveit_group_name; }
      std::string getReferenceFrame() const { return m_params.reference_frame; }
      std::string getParentLink() const { return m_params.parent_link; }
      void getCapacities(std::vector<Capacity>& t_capacities) const;
      void getCapabilities(std::vector<std::string>& t_capacity_names) const; // Vector of capacity names

      // --------------
      // Capacity Level
      // --------------
      bool hasCapacity(const std::string& t_capacity_name);
      bool removeCapacity(const std::string& t_capacity_name);
      bool getCapacity(const std::string& t_capacity_name, Capacity& t_capacity);
      bool getCapacityProperties(const std::string& t_capacity_name, std::vector<Property>& t_props);
      bool addCapacity(const Capacity& t_capacity);
      void setCapacity(const Capacity& t_capacity);

      // --------------
      // Property level
      // --------------
      bool hasCapacityProperty(const std::string& t_capacity_name, const std::string& t_prop_name);
      bool removeCapacityProperty(const std::string& t_capacity_name, const std::string& t_prop_name);
      bool getCapacityProperty(const std::string& t_capacity_name, const std::string& t_prop_name, Property& t_prop);
      bool addCapacityProperty(const std::string& t_capacity_name, const Property& t_prop);
      void setCapacityProperty(const std::string& t_capacity_name, const Property& t_prop);

    private:
      struct ComponentParameters
      {
        unsigned int id = std::numeric_limits<unsigned int>::quiet_NaN();
        std::string name = "";
        std::string type = "";
        std::string description = "";
        std::string urdf_link = "";
        std::string moveit_group_name = "";
        std::string reference_frame = "";
        std::string parent_link = "";
        std::map<std::string, Capacity> capacities{};
      };

      ComponentParameters m_params{};

      void setId(const unsigned int t_value) { m_params.id = t_value; }
      void setName(const std::string& t_value) { m_params.name = t_value; }
      void setType(const std::string& t_value) { m_params.type = t_value; }
      void setReferenceFrame(const std::string& t_value) { m_params.reference_frame = t_value; }

      void clearCapacities();
      void setCapacities(const std::vector<Capacity>& t_capacities);

      // void clearCapacityProperties(const std::string& t_capacity_name);
      // bool setCapacityProperties(const std::string& t_capacity_name, const std::vector<Property>& t_props);
    };

  } // namespace commons
} // namespace rtask

#endif
