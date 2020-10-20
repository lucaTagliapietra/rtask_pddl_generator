#ifndef rtask_commons_property_h
#define rtask_commons_property_h

#include <limits>
#include <string>
#include <variant>
#include <vector>

#include "xmlrpcpp/XmlRpc.h"

#include "rtask_msgs/Property.h"

namespace rtask {
  namespace commons {

    using PropertyVariant = std::variant<bool, int, double, std::string>;

    enum PropertyType
    {
      BOOL = 0,
      INT = 1,
      DOUBLE = 2,
      STRING = 3
    };

    class Property
    {
    public:
      Property() = default;
      ~Property() = default;

      Property(const std::string& t_name, const PropertyVariant& t_val = {});
      Property(const rtask_msgs::PropertyConstPtr t_msg_ptr);
      Property(const rtask_msgs::Property& t_msg);
      Property(XmlRpc::XmlRpcValue& t_rpc_val);

      rtask_msgs::Property toMsg() const;

      void set(const std::string& t_name, const PropertyVariant& t_val);
      void setValue(const PropertyVariant& t_val);

      std::pair<bool, PropertyVariant> getValue() const;
      inline std::string getName() const { return name_; }
      inline PropertyType getType() const { return static_cast<PropertyType>(value_.index()); }
      inline bool isValid() const { return valid_; }

      bool operator==(const Property& t_property) const;
      Property& operator=(const Property& t_property);

    private:
      void fromMsg(const rtask_msgs::Property& t_msg);

      std::string name_{};
      bool valid_{false};
      PropertyVariant value_{};
    };

    static std::ostream& operator<<(std::ostream& out, const Property& p)
    {
      auto t = p.getType();
      out << "name: " << p.getName() << std::endl
          << "type: " << t << std::endl
          << "valid: " << p.isValid() << std::endl;

      PropertyVariant v = p.getValue().second;
      switch (t) {
        case PropertyType::BOOL: {
          out << "value: " << std::get<bool>(v) << std::endl;
          break;
        }
        case PropertyType::INT: {
          out << "value: " << std::get<int>(v) << std::endl;
          break;
        }
        case PropertyType::DOUBLE: {
          out << "value: " << std::get<double>(v) << std::endl;
          break;
        }
        case PropertyType::STRING: {
          out << "value: " << std::get<std::string>(v) << std::endl;
          break;
        }
      }
      return out << std::endl;
    }

    static std::ostream& operator<<(std::ostream& out, const PropertyType& pt)
    {
      switch (pt) {
        case PropertyType::BOOL: {
          out << "Bool";
          break;
        };
        case PropertyType::INT: {
          out << "Int";
          break;
        };
        case PropertyType::DOUBLE: {
          out << "Double";
          break;
        };
        case PropertyType::STRING: {
          out << "String";
          break;
        };
      }
      return out << std::endl;
    }

  } // namespace commons
} // namespace rtask

#endif
