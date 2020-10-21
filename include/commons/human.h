#ifndef rtask_commons_human_h
#define rtask_commons_human_h

#include <string>
#include <vector>

#include "rtask_msgs/Human.h"

#include "commons/device.h"
#include "commons/property.h"

namespace rtask {
  namespace commons {

    class Human
    {
    public:
      Human() = default;
      ~Human() = default;

      Human(const std::string& t_name,
            const std::vector<Device> t_tools = {},
            const std::vector<Property> t_extra_properties = {});
      Human(const rtask_msgs::Human& t_msg);
      Human(const rtask_msgs::HumanConstPtr t_msg_ptr);
      Human(XmlRpc::XmlRpcValue& t_rpc_val);

      // ---------------
      // Device level
      // ---------------
      rtask_msgs::Human toMsg() const;

      void clear();
      void set(const std::string& t_name,
               const std::vector<Device>& t_tools,
               const std::vector<Property> t_extra_properties);

      inline bool isValid() const { return valid_; }

      inline std::string getName() const { return name_; }
      inline std::vector<Device> getTools() const { return tools_; }

      inline std::vector<Property> getExtraProperties() const { return extra_properties_; }

      std::vector<std::string> getExtraPropertyList() const;
      std::vector<std::string> getToolList() const;

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
      bool hasTool(const std::string& t_name) const;
      bool deleteTool(const std::string& t_name);
      bool isToolValid(const std::string& t_name) const;
      std::pair<bool, Device> getTool(const std::string& t_name) const;
      void setTool(const std::string& t_name,
                   const DeviceClass& t_tool_class,
                   const std::string& t_tool_subclass,
                   const std::vector<Property> t_tool_unique_props,
                   const std::vector<Property> t_tool_extra_properties,
                   const std::vector<Capability> t_tool_capabilities);
      void setTool(const std::string& t_name, const Device& t_tool);
      // ---------
      // Operators
      // ---------
      bool operator==(const Human& t_h) const;
      Human& operator=(const Human& t_h);

    private:
      bool valid_{false};
      std::string name_{};
      std::vector<Device> tools_{};
      std::vector<Property> extra_properties_{};

      void updValidity();
      void fromMsg(const rtask_msgs::Human& t_msg);
    };

    static std::ostream& operator<<(std::ostream& out, const Human& human)
    {
      out << "SingleRoboticSystem name: " << human.getName() << std::endl;
      out << "Tools: " << std::endl;
      unsigned int i = 0;
      for (const auto& t : human.getTools()) {
        out << "\t"
            << " - d[" << i << "]: " << t;
        ++i;
      }
      out << "Extra properties: " << std::endl;
      i = 0;
      for (const auto& p : human.getExtraProperties()) {
        out << "\t"
            << " - p[" << i << "]: " << p;
        ++i;
      }

      return out << std::endl;
    };
  }; // namespace commons
} // namespace rtask

#endif
