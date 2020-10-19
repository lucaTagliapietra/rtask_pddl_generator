#ifndef rtask_commons_device_h
#define rtask_commons_device_h

#include <string>
#include <vector>

#include "rtask_msgs/Device.h"

#include "commons/capability.h"
#include "commons/property.h"

namespace rtask {
  namespace commons {

    enum DeviceClass
    {
      INVALID = -1,
      ROBOT = 0,
      GRIPPER = 1,
      SENSOR = 2,
      TOOL = 3
    };

    class Device
    {
    public:
      Device() = default;
      ~Device() = default;

      Device(const std::string& t_name,
             const DeviceClass& t_class,
             const std::string& t_subclass = {},
             const std::vector<Property> t_unique_props = {},
             const std::vector<Property> t_extra_properties = {},
             const std::vector<Capability> t_capabilities = {});
      Device(const rtask_msgs::Device& t_msg);
      Device(const rtask_msgs::DeviceConstPtr t_msg_ptr);
      Device(XmlRpc::XmlRpcValue& t_rpc_val);

      // ---------------
      // Device level
      // ---------------
      rtask_msgs::Device toMsg() const;

      void clear();
      void set(const std::string& t_name,
               const DeviceClass& t_class,
               const std::string& t_subclass,
               const std::vector<Property> t_unique_props,
               const std::vector<Property> t_extra_properties,
               const std::vector<Capability> t_capabilities);

      inline bool isValid() const { return valid_; }

      inline std::string getName() const { return name_; }
      inline DeviceClass getClass() const { return class_; }
      inline std::string getSubclass() const { return subclass_; }

      inline std::vector<Property> getUniqueProperties() const { return unique_properties_; }
      inline std::vector<Property> getExtraProperties() const { return extra_properties_; }
      inline std::vector<Capability> getCapabilities() const { return capabilities_; }

      std::vector<std::string> getUniquePropertyList() const;
      std::vector<std::string> getExtraPropertyList() const;
      std::vector<std::string> getCapabilityList() const;

      // --------------
      // Property Level
      // --------------
      bool hasUniqueProperty(const std::string& t_name) const;
      bool deleteUniqueProperty(const std::string& t_name);
      bool isUniquePropertyValid(const std::string& t_name) const;
      std::pair<bool, Property> getUniqueProperty(const std::string& t_name) const;
      void setUniqueProperty(const std::string& t_name, const PropertyVariant& t_val);

      bool hasExtraProperty(const std::string& t_name) const;
      bool deleteExtraProperty(const std::string& t_name);
      bool isExtraPropertyValid(const std::string& t_name) const;
      std::pair<bool, Property> getExtraProperty(const std::string& t_name) const;
      void setExtraProperty(const std::string& t_name, const PropertyVariant& t_val);

      // ----------------
      // Capability Level
      // ----------------
      bool hasCapability(const std::string& t_name) const;
      bool deleteCapability(const std::string& t_name);
      bool isCapabilityValid(const std::string& t_name) const;
      std::pair<bool, Capability> getCapability(const std::string& t_name) const;
      void setCapability(const std::string& t_name, const std::vector<Property>& t_val);

      // ---------
      // Operators
      // ---------
      bool operator==(const Device& t_dev) const;
      Device& operator=(const Device& t_dev);

    private:
      bool valid_{false};
      std::string name_{};
      DeviceClass class_{DeviceClass::INVALID};
      std::string subclass_{};
      std::vector<Property> unique_properties_{};
      std::vector<Property> extra_properties_{};
      std::vector<Capability> capabilities_{};

      void updValidity();
      void fromMsg(const rtask_msgs::Device& t_msg);
    };

    static std::ostream& operator<<(std::ostream& out, const Device& d)
    {
      out << "Device name: " << d.getName() << std::endl
          << "Device class: " << d.getClass() << std::endl
          << "Device subclass: " << d.getSubclass() << std::endl
          << "Device valid: " << d.isValid() << std::endl
          << "Unique properties: " << std::endl;
      unsigned int i = 0;
      for (const auto& p : d.getUniqueProperties()) {
        out << "\t"
            << " - p[" << i << "]: " << p;
        ++i;
      }
      out << "Extra properties: " << std::endl;
      i = 0;
      for (const auto& p : d.getExtraProperties()) {
        out << "\t"
            << " - p[" << i << "]: " << p;
        ++i;
      }
      out << "Capabilities: " << std::endl;
      i = 0;
      for (const auto& c : d.getCapabilities()) {
        out << "\t"
            << " - c[" << i << "]: " << c;
        ++i;
      }
      return out << std::endl;
    }

    static std::ostream& operator<<(std::ostream& out, const DeviceClass& dt)
    {
      switch (dt) {
        case DeviceClass::INVALID: {
          out << "Invalid";
          break;
        };
        case DeviceClass::ROBOT: {
          out << "Robot";
          break;
        };
        case DeviceClass::GRIPPER: {
          out << "Gripper";
          break;
        };
        case DeviceClass::SENSOR: {
          out << "Sensor";
          break;
        };
        case DeviceClass::TOOL: {
          out << "Tool";
          break;
        };
      }
      return out << std::endl;
    }
  } // namespace commons
} // namespace rtask

#endif
