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
      Property() = delete;

      Property(const std::string& t_name, const bool t_val);
      Property(const std::string& t_name, const int t_val);
      Property(const std::string& t_name, const double t_val);
      Property(const std::string& t_name, const std::string t_val);

      Property(const rtask_msgs::PropertyConstPtr t_msg_ptr);
      Property(const rtask_msgs::Property& t_msg);
      Property(XmlRpc::XmlRpcValue& t_rpc_val);

      rtask_msgs::Property toMsg() const;

      void updValue(const bool t_val);
      void updValue(const int t_val);
      void updValue(const double t_val);
      void updValue(const std::string& t_val);

      bool getValue(bool& t_val) const;
      bool getValue(int& t_val) const;
      bool getValue(double& t_val) const;
      bool getValue(std::string& t_val) const;

      inline std::string getName() const { return name_; }
      inline PropertyType getType() const { return static_cast<PropertyType>(value_.index()); }

      bool operator==(const Property& t_property) const;

    private:
      void fromMsg(const rtask_msgs::Property& t_msg);

      std::string name_;
      PropertyVariant value_;
    };

    static std::ostream& operator<<(std::ostream& out, const Property& p)
    {
      auto t = p.getType();
      out << "name: " << p.getName() << std::endl << "type: " << t << std::endl;

      switch (t) {
        case PropertyType::BOOL: {
          bool val;
          p.getValue(val);
          out << "value: " << val << std::endl;
          break;
        }
        case PropertyType::INT: {
          int val;
          p.getValue(val);
          out << "value: " << val << std::endl;
          break;
        }
        case PropertyType::DOUBLE: {
          double val;
          p.getValue(val);
          out << "value: " << val << std::endl;
          break;
        }
        case PropertyType::STRING: {
          std::string val;
          p.getValue(val);
          out << "value: " << val << std::endl;
          break;
        }
      }
      return out << std::endl;
    }

  } // namespace commons
} // namespace rtask

#endif
