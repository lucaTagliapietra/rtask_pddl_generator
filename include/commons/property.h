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

    class Property
    {
    public:
      enum Type
      {
        Boolean = 0,
        Integer = 1,
        Double = 2,
        String = 3
      };

      Property() = delete;

      Property(const std::string& t_name, bool t_val);
      Property(const std::string& t_name, int t_val);
      Property(const std::string& t_name, double t_val);
      Property(const std::string& t_name, std::string t_val);

      void set_value(bool& t_val);
      void set_value(int& t_val);
      void set_value(double& t_val);
      void set_value(std::string& t_val);

      inline std::string getName() const { return name_; }

      const PropertyVariant get_variant() const;
      std::pair<Type, std::string> get();
      Type get_property_type() const;

    private:
      std::string name_;
      PropertyVariant value_;
    };

    //    class Property
    //    {
    //    public:
    //      enum Type
    //      {
    //        Boolean = 0,
    //        Integer = 1,
    //        Double = 2,
    //        String = 3
    //      };

    //      Property() = delete;
    //      Property(const std::string& t_name, const bool t_value);
    //      Property(const std::string& t_name, const int t_value);
    //      Property(const std::string& t_name, const double t_value);
    //      Property(const std::string& t_name, const std::string& t_value);
    //      Property(const rtask_msgs::PropertyConstPtr t_msg);
    //      Property(const rtask_msgs::Property& t_property_msg);
    //      Property(XmlRpc::XmlRpcValue& t_structured_value);

    //      ~Property() {}

    //      rtask_msgs::PropertyPtr toPropertyMsg() const;

    //      inline std::string getName() const { return m_params.name; }
    //      inline Type getType() const { return m_params.type; }

    //      bool getValue(bool& t_value) const;
    //      bool getValue(int& t_value) const;
    //      bool getValue(double& t_value) const;
    //      bool getValue(std::string& t_value) const;

    //      void setProperty(const std::string& t_name, const bool t_value);
    //      void setProperty(const std::string& t_name, const int& t_value);
    //      void setProperty(const std::string& t_name, const double& t_value);
    //      void setProperty(const std::string& t_name, const std::string& t_value);

    //      bool setPropertyFromXmlRpc(XmlRpc::XmlRpcValue& t_structured_value);
    //      void setFromPropertyMsg(const rtask_msgs::PropertyConstPtr t_msg_ptr);
    //      void setFromPropertyMsg(const rtask_msgs::Property& t_msg);

    //      // ---------
    //      // Operators
    //      // ---------

    //      bool operator==(const Property& t_property) const;

    //    private:
    //      bool setName(const std::string& t_name);
    //      bool setValue(const bool t_value);
    //      bool setValue(const int t_value);
    //      bool setValue(const double t_value);
    //      bool setValue(const std::string& t_value);

    //      struct Parameters
    //      {
    //        std::string name;
    //        Type type;
    //        std::optional<bool> bool_val;
    //        std::optional<int> int_val;
    //        std::optional<double> double_val;
    //        std::optional<std::string> str_val;
    //        //        int int_value = std::numeric_limits<int>::quiet_NaN();
    //        //        double double_value = std::numeric_limits<double>::quiet_NaN();
    //        //        std::string str_value = "";
    //      };

    //      Parameters m_params;
    //    };

    //    static std::ostream& operator<<(std::ostream& out, const Property& p)
    //    {
    //      out << "valid: " << p.isValid() << std::endl
    //          << "name: " << p.getName() << std::endl
    //          << "type: " << p.getType() << std::endl;

    //      bool bool_value;
    //      int int_value;
    //      double double_value;
    //      std::string string_value;

    //      switch (p.getType()) {
    //        case Property::Type::Boolean:
    //          p.getValue(bool_value);
    //          out << "value: " << bool_value << std::endl;
    //          break;
    //        case rtask::commons::Property::Type::Integer:
    //          p.getValue(int_value);
    //          out << "value: " << int_value << std::endl;
    //          break;
    //        case rtask::commons::Property::Type::Double:
    //          p.getValue(double_value);
    //          out << "value: " << double_value << std::endl;
    //          break;
    //        case rtask::commons::Property::Type::String:
    //          p.getValue(string_value);
    //          out << "value: " << string_value << std::endl;
    //          break;
    //      }
    //      return out << std::endl;
    //    }
  } // namespace commons
} // namespace rtask

#endif
