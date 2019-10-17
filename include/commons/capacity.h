#ifndef rtask_commons_capacity_h
#define rtask_commons_capacity_h

#include <map>
#include <string>
#include <vector>

#include "rtask_msgs/Capacity.h"

#include "commons/property.h"

namespace rtask {
  namespace commons {

    class Capacity
    {

    public:
      Capacity() {}
      Capacity(const std::string t_capability, const std::vector<Property>& t_properties = {});
      Capacity(const rtask_msgs::Capacity& t_msg);
      Capacity(const rtask_msgs::CapacityConstPtr t_msg_ptr);

      ~Capacity() {}

      // --------------
      // Capacity Level
      // --------------
      rtask_msgs::CapacityPtr toCapacityMsg() const;

      void setFromCapacityMsg(const rtask_msgs::Capacity& t_msg);
      void setFromCapacityMsg(const rtask_msgs::CapacityConstPtr t_msg_ptr);
      void setCapacity(const std::string& t_capability, const std::vector<Property>& t_properties = {});

      void clear();

      std::string getCapabilityName() const;
      void getProperties(std::vector<Property>& t_properties) const;

      // --------------
      // Property Level
      // --------------
      bool hasProperty(const std::string& t_property_name) const;
      bool getProperty(const std::string& t_property_name, Property& t_property) const;
      void setProperty(const Property& t_property); // if not in, add it; if in, overwrite
      bool addProperty(const Property& t_property); // if already in, leave it unchanged
      bool removeProperty(const std::string& t_property_name);

      // ---------
      // Operators
      // ---------

      bool operator==(const Capacity& t_capacity);

    private:
      std::string m_capability = "";
      std::map<std::string, Property> m_properties{};

      void setCapabilityName(const std::string& t_capability);
      void setProperties(const std::vector<Property>& t_properties);
      void clearProperties();
    };
  } // namespace commons
} // namespace rtask

#endif
