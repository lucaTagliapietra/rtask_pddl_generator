#ifndef rtask_commons_capability_h
#define rtask_commons_capability_h

#include <string>
#include <vector>

#include "xmlrpcpp/XmlRpcValue.h"

#include "commons/property.h"
#include "rtask_msgs/Capability.h"

namespace rtask {
  namespace commons {

    class Capability
    {

    public:
      Capability() = default;
      ~Capability() = default;

      Capability(const std::string& t_name, const std::vector<Property>& t_props = {});

      Capability(const rtask_msgs::Capability& t_msg);
      Capability(const rtask_msgs::CapabilityConstPtr t_msg_ptr);
      Capability(XmlRpc::XmlRpcValue& t_rpc_val);

      // --------------
      // Capability Level
      // --------------
      rtask_msgs::Capability toMsg() const;

      void clear();
      void set(const std::string& t_name, const std::vector<Property>& t_props);

      inline std::string getName() const { return name_; }
      inline bool isValid() const { return valid_; }
      inline std::vector<Property> getProperties() const { return properties_; }

      std::vector<std::string> getPropertyList() const;

      // --------------
      // Property Level
      // --------------
      bool hasProperty(const std::string& t_name) const;
      bool deleteProperty(const std::string& t_name);

      bool getProperty(const std::string& t_name, PropertyVariant& t_val) const;
      void setProperty(const std::string& t_name, const PropertyVariant& t_val);
      bool isPropertyValid(const std::string& t_name) const;

      // ---------
      // Operators
      // ---------

      bool operator==(const Capability& t_capability) const;
      Capability& operator=(const Capability& t_capability);

    private:
      std::string name_{};
      bool valid_{false};
      std::vector<Property> properties_{};

      void updValidity();
      void fromMsg(const rtask_msgs::Capability& t_msg);
    };

    static std::ostream& operator<<(std::ostream& out, const Capability& c)
    {
      out << "Capability name: " << c.getName() << std::endl << "Capability valid: " << c.isValid() << std::endl;
      unsigned int i = 0;
      for (const auto& p : c.getProperties()) {
        out << "\t"
            << "p[" << i << "]: " << p;
        ++i;
      }
      return out << std::endl;
    }
  } // namespace commons
} // namespace rtask

#endif
