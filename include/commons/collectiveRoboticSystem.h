#ifndef rtask_commons_collective_robotic_system_h
#define rtask_commons_collective_robotic_system_h

#include <string>
#include <vector>

#include "rtask_msgs/CollectiveRoboticSystem.h"

#include "commons/property.h"
#include "commons/singleRoboticSystem.h"

namespace rtask {
  namespace commons {

    class CollectiveRoboticSystem
    {
    public:
      CollectiveRoboticSystem() = default;
      ~CollectiveRoboticSystem() = default;

      CollectiveRoboticSystem(const std::string& t_name,
                              const std::vector<SingleRoboticSystem> t_srs = {},
                              const std::vector<Property> t_extra_properties = {});
      CollectiveRoboticSystem(const rtask_msgs::CollectiveRoboticSystem& t_msg);
      CollectiveRoboticSystem(const rtask_msgs::CollectiveRoboticSystemConstPtr t_msg_ptr);
      CollectiveRoboticSystem(XmlRpc::XmlRpcValue& t_rpc_val);

      // ---------------
      // Device level
      // ---------------
      rtask_msgs::CollectiveRoboticSystem toMsg() const;

      void clear();
      void set(const std::string& t_name,
               const std::vector<SingleRoboticSystem>& t_srs,
               const std::vector<Property>& t_extra_properties);

      inline bool isValid() const { return valid_; }

      inline std::string getName() const { return name_; }
      inline std::vector<SingleRoboticSystem> getSingleRoboticSystems() const { return single_robotic_systems_; }
      inline std::vector<Property> getExtraProperties() const { return extra_properties_; }

      std::vector<std::string> getExtraPropertyList() const;
      std::vector<std::string> getSingleRoboticSystemList() const;

      // --------------
      // Property Level
      // --------------
      bool hasExtraProperty(const std::string& t_name) const;
      bool deleteExtraProperty(const std::string& t_name);
      bool isExtraPropertyValid(const std::string& t_name) const;
      std::pair<bool, Property> getExtraProperty(const std::string& t_name) const;
      void setExtraProperty(const std::string& t_name, const PropertyVariant& t_val);

      // -------------------------
      // SingleRoboticSystem Level
      // -------------------------
      bool hasSingleRoboticSystem(const std::string& t_name) const;
      bool deleteSingleRoboticSystem(const std::string& t_name);
      bool isSingleRoboticSystemValid(const std::string& t_name) const;
      std::pair<bool, SingleRoboticSystem> getSingleRoboticSystem(const std::string& t_name) const;
      void setSingleRoboticSystem(const std::string& t_name,
                                  const std::vector<Device> t_devices,
                                  const std::vector<Property> t_extra_properties);
      void setSingleRoboticSystem(const std::string& t_name, const SingleRoboticSystem& t_srs);
      // ---------
      // Operators
      // ---------
      bool operator==(const CollectiveRoboticSystem& t_crs) const;
      CollectiveRoboticSystem& operator=(const CollectiveRoboticSystem& t_crs);

    private:
      bool valid_{false};
      std::string name_{};
      std::vector<SingleRoboticSystem> single_robotic_systems_{};
      std::vector<Property> extra_properties_{};

      void updValidity();
      void fromMsg(const rtask_msgs::CollectiveRoboticSystem& t_msg);
    };

    static std::ostream& operator<<(std::ostream& out, const CollectiveRoboticSystem& crs)
    {
      out << "CollectiveRoboticSystem name: " << crs.getName() << std::endl;
      out << "SingleRoboticSystems: " << std::endl;
      unsigned int i = 0;
      for (const auto& srs : crs.getSingleRoboticSystems()) {
        out << "\t"
            << " - srs[" << i << "]: " << srs;
        ++i;
      }
      out << "Extra properties: " << std::endl;
      i = 0;
      for (const auto& p : crs.getExtraProperties()) {
        out << "\t"
            << " - p[" << i << "]: " << p;
        ++i;
      }

      return out << std::endl;
    }
  }; // namespace commons
} // namespace rtask

#endif
