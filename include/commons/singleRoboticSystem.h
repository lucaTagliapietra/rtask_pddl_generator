#ifndef rtask_commons_single_robotic_system_h
#define rtask_commons_single_robotic_system_h

#include <string>
#include <vector>

#include "rtask_msgs/SingleRoboticSystem.h"

#include "commons/device.h"
#include "commons/property.h"

namespace rtask {
  namespace commons {

    class SingleRoboticSystem
    {
    public:
      SingleRoboticSystem() = default;
      ~SingleRoboticSystem() = default;

      SingleRoboticSystem(const std::string& t_name,
                          const std::vector<Device> t_devices = {},
                          const std::vector<Property> t_extra_properties = {});
      SingleRoboticSystem(const rtask_msgs::SingleRoboticSystem& t_msg);
      SingleRoboticSystem(const rtask_msgs::SingleRoboticSystemConstPtr t_msg_ptr);
      SingleRoboticSystem(XmlRpc::XmlRpcValue& t_rpc_val);

      // ---------------
      // Device level
      // ---------------
      rtask_msgs::SingleRoboticSystem toMsg() const;

      void clear();
      void set(const std::string& t_name,
               const std::vector<Device>& t_devices,
               const std::vector<Property> t_extra_properties);

      inline bool isValid() const { return valid_; }

      inline std::string getName() const { return name_; }
      inline std::vector<Device> getDevices() const { return devices_; }

      inline std::vector<Property> getExtraProperties() const { return extra_properties_; }

      std::vector<std::string> getExtraPropertyList() const;
      std::vector<std::string> getDeviceList() const;

      // --------------
      // Property Level
      // --------------
      bool hasExtraProperty(const std::string& t_name) const;
      bool deleteExtraProperty(const std::string& t_name);
      bool isExtraPropertyValid(const std::string& t_name) const;
      std::pair<bool, Property> getExtraProperty(const std::string& t_name) const;
      void setExtraProperty(const std::string& t_name, const PropertyVariant& t_val);

      // ----------------
      // Device Level
      // ----------------
      bool hasDevice(const std::string& t_name) const;
      bool deleteDevice(const std::string& t_name);
      bool isDeviceValid(const std::string& t_name) const;
      std::pair<bool, Device> getDevice(const std::string& t_name) const;
      void setDevice(const std::string& t_name,
                     const DeviceClass& t_dev_class,
                     const std::string& t_dev_subclass,
                     const std::vector<Property> t_dev_unique_props,
                     const std::vector<Property> t_dev_extra_properties,
                     const std::vector<Capability> t_dev_capabilities);
      void setDevice(const std::string& t_name, const Device& t_dev);
      // ---------
      // Operators
      // ---------
      bool operator==(const SingleRoboticSystem& t_srs) const;
      SingleRoboticSystem& operator=(const SingleRoboticSystem& t_srs);

    private:
      bool valid_{false};
      std::string name_{};
      std::vector<Device> devices_{};
      std::vector<Property> extra_properties_{};

      void updValidity();
      void fromMsg(const rtask_msgs::SingleRoboticSystem& t_msg);
    };

    static std::ostream& operator<<(std::ostream& out, const SingleRoboticSystem& srs)
    {
      out << "SingleRoboticSystem name: " << srs.getName() << std::endl;
      out << "Devices: " << std::endl;
      unsigned int i = 0;
      for (const auto& d : srs.getDevices()) {
        out << "\t"
            << " - d[" << i << "]: " << d;
        ++i;
      }
      out << "Extra properties: " << std::endl;
      i = 0;
      for (const auto& p : srs.getExtraProperties()) {
        out << "\t"
            << " - p[" << i << "]: " << p;
        ++i;
      }

      return out << std::endl;
    }
  }; // namespace commons
} // namespace rtask

#endif
