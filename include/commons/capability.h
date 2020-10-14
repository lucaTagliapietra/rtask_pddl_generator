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

      // bool setCapacityFromXmlRpc(XmlRpc::XmlRpcValue& t_value);
      // void setFromCapacityMsg(const rtask_msgs::Capacity& t_msg);
      // void setFromCapacityMsg(const rtask_msgs::CapacityConstPtr t_msg_ptr);
      // void setCapacity(const std::string& t_capability, const std::vector<Property>& t_properties = {});

      void clear();

      void set(const std::string t_name, const std::vector<Property>& t_props);

      inline std::string getName() const { return name_; }
      inline bool isValid() const { return valid_; }

      std::vector<std::string> getPropertyList() const;
      void getProperties(std::vector<Property>& t_props) const;

      // --------------
      // Property Level
      // --------------
      bool hasProperty(const std::string& t_name) const;
      bool deleteProperty(const std::string& t_property_name);

      bool getProperty(const std::string& t_name, bool& t_val) const;
      bool getProperty(const std::string& t_name, int& t_val) const;
      bool getProperty(const std::string& t_name, double& t_val) const;
      bool getProperty(const std::string& t_name, std::string& t_val) const;

      bool setProperty(const std::string& t_name, bool t_val) const;
      bool setProperty(const std::string& t_name, int t_val) const;
      bool setProperty(const std::string& t_name, double t_val) const;
      bool setProperty(const std::string& t_name, std::string t_val) const;

      // ---------
      // Operators
      // ---------

      bool operator==(const Capability& t_capability) const;

    private:
      std::string name_{};
      bool valid_{false};
      std::vector<Property> properties_{};

      void updValidity();
      void fromMsg(const rtask_msgs::Capability& t_msg);
    };

    //    static std::ostream& operator<<(std::ostream& out, const Capability& p)
    //    {
    //      out << "is_valid: " << p.isValid() << std::endl << "name: " << p.getCapabilityName() << std::endl <<
    //      std::endl; std::vector<Property> props; p.getProperties(props); unsigned int i = 0; for (const auto& p :
    //      props) {
    //        out << "property " << i << ": " << std::endl << std::endl << p;
    //        ++i;
    //      }
    //      return out << std::endl;
    //    }
  } // namespace commons
} // namespace rtask

#endif
