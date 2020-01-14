#ifndef rtask_commons_property_h
#define rtask_commons_property_h

#include <limits>
#include <string>
#include <vector>

#include "xmlrpcpp/XmlRpc.h"

#include "rtask_msgs/Property.h"

namespace rtask {
  namespace commons {

    class Property
    {
    public:
      enum Type
      {
        Undefined = -1,
        Boolean = 0,
        Integer = 1,
        Double = 2,
        String = 3
      };

      Property();
      Property(const std::string& t_name, const bool t_value);
      Property(const std::string& t_name, const int t_value);
      Property(const std::string& t_name, const double t_value);
      Property(const std::string& t_name, const std::string& t_value);
      Property(const rtask_msgs::PropertyConstPtr t_msg);
      Property(const rtask_msgs::Property& t_property_msg);
      Property(XmlRpc::XmlRpcValue& t_structured_value);

      ~Property() {}

      rtask_msgs::PropertyPtr toPropertyMsg() const;

      inline std::string getName() const { return m_params.name; }
      inline Type getType() const { return m_params.type; }
      inline bool isValid() const { return m_params.valid; }
      bool getValue(bool& t_value) const;
      bool getValue(int& t_value) const;
      bool getValue(double& t_value) const;
      bool getValue(std::string& t_value) const;

      void setProperty(const std::string& t_name, const bool t_value);
      void setProperty(const std::string& t_name, const int& t_value);
      void setProperty(const std::string& t_name, const double& t_value);
      void setProperty(const std::string& t_name, const std::string& t_value);
      bool setPropertyFromXmlRpc(XmlRpc::XmlRpcValue& t_structured_value);
      void setFromPropertyMsg(const rtask_msgs::PropertyConstPtr t_msg_ptr);
      void setFromPropertyMsg(const rtask_msgs::Property& t_msg);

      // ---------
      // Operators
      // ---------

      bool operator==(const Property& t_property) const;

    private:
      bool setName(const std::string& t_name);
      bool setValue(const bool t_value);
      bool setValue(const int t_value);
      bool setValue(const double t_value);
      bool setValue(const std::string& t_value);

      struct Parameters
      {
        bool valid = false;
        std::string name = "";
        Type type = Type::Undefined;
        bool bool_value = std::numeric_limits<bool>::quiet_NaN();
        int int_value = std::numeric_limits<int>::quiet_NaN();
        double double_value = std::numeric_limits<double>::quiet_NaN();
        std::string str_value = "";
      };

      Parameters m_params;
    };

    static std::ostream& operator<<(std::ostream& out, const Property& p)
    {
      out << "valid: " << p.isValid() << std::endl
          << "name: " << p.getName() << std::endl
          << "type: " << p.getType() << std::endl;

      bool bool_value;
      int int_value;
      double double_value;
      std::string string_value;

      switch (p.getType()) {
        case rtask::commons::Property::Type::Boolean:
          p.getValue(bool_value);
          out << "value: " << bool_value << std::endl;
          break;
        case rtask::commons::Property::Type::Integer:
          p.getValue(int_value);
          out << "value: " << int_value << std::endl;
          break;
        case rtask::commons::Property::Type::Double:
          p.getValue(double_value);
          out << "value: " << double_value << std::endl;
          break;
        case rtask::commons::Property::Type::String:
          p.getValue(string_value);
          out << "value: " << string_value << std::endl;
          break;
        default:
          out << "Unknown Type.";
      }

      return out << std::endl;
    }
  } // namespace commons
} // namespace rtask

#endif
