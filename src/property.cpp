#include "commons/property.h"
#include "commons/utils.h"

#include "boost/make_shared.hpp"

// ------------
// CONSTRUCTORS
// ------------

rtask::commons::Property::Property()
  : m_params({})
{}

rtask::commons::Property::Property(const std::string& t_name, const bool t_value)
{
  setProperty(t_name, t_value);
};

rtask::commons::Property::Property(const std::string& t_name, const int t_value)
{
  setProperty(t_name, t_value);
}

rtask::commons::Property::Property(const std::string& t_name, const double t_value)
{
  setProperty(t_name, t_value);
};

rtask::commons::Property::Property(const std::string& t_name, const std::string& t_value)
{
  setProperty(t_name, t_value);
};

rtask::commons::Property::Property(const rtask_msgs::PropertyConstPtr t_msg)
{
  setFromPropertyMsg(t_msg);
}

rtask::commons::Property::Property(const rtask_msgs::Property& t_property_msg)
{
  setFromPropertyMsg(t_property_msg);
}

rtask::commons::Property::Property(XmlRpc::XmlRpcValue& t_structured_value)
{
  setPropertyFromXmlRpc(t_structured_value);
}

// ----------------
// PUBLIC FUNCTIONS
// ----------------

rtask_msgs::PropertyPtr rtask::commons::Property::toPropertyMsg() const
{
  rtask_msgs::PropertyPtr a_property = boost::make_shared<rtask_msgs::Property>();
  a_property->name = m_params.name;
  switch (m_params.type) {
    case Type::Boolean:
      a_property->type = rtask_msgs::Property::BOOLEAN;
      a_property->bool_value = m_params.bool_value;
      break;
    case Type::Integer:
      a_property->type = rtask_msgs::Property::INTEGER;
      a_property->int_value = m_params.int_value;
      break;
    case Type::Double:
      a_property->type = rtask_msgs::Property::DOUBLE;
      a_property->double_value = m_params.double_value;
      break;
    case Type::String:
      a_property->type = rtask_msgs::Property::STRING;
      a_property->str_value = m_params.str_value;
      break;
    default:
      std::cout << "Unknown Type, empty message returned." << std::endl;
  }
  return a_property;
}

void rtask::commons::Property::setProperty(const std::string& t_name, const bool t_value)
{
  m_params.valid = setName(t_name) && setValue(t_value);
};

void rtask::commons::Property::setProperty(const std::string& t_name, const int t_value)
{
  m_params.valid = setName(t_name) && setValue(t_value);
};

void rtask::commons::Property::setProperty(const std::string& t_name, const double t_value)
{
  m_params.valid = setName(t_name) && setValue(t_value);
};

void rtask::commons::Property::setProperty(const std::string& t_name, const std::string& t_value)
{
  m_params.valid = setName(t_name) && setValue(t_value);
};

bool rtask::commons::Property::setPropertyFromXmlRpc(XmlRpc::XmlRpcValue& t_structured_value)
{
  if (utils::checkXmlRpcSanity("name", t_structured_value, XmlRpc::XmlRpcValue::TypeString)
      && utils::checkXmlRpcSanity("type", t_structured_value, XmlRpc::XmlRpcValue::TypeInt)) {
    std::string name = static_cast<std::string>(t_structured_value["name"]);
    switch (static_cast<int>(t_structured_value["type"])) {
      case Type::Boolean: {
        if (utils::checkXmlRpcSanity("bool_value", t_structured_value, XmlRpc::XmlRpcValue::TypeBoolean)) {
          setProperty(name, static_cast<bool>(t_structured_value["bool_value"]));
        }
        break;
      }
      case Type::Integer: {
        if (utils::checkXmlRpcSanity("int_value", t_structured_value, XmlRpc::XmlRpcValue::TypeInt)) {
          setProperty(name, static_cast<int>(t_structured_value["int_value"]));
        }
        break;
      }
      case Type::Double: {
        if (utils::checkXmlRpcSanity("double_value", t_structured_value, XmlRpc::XmlRpcValue::TypeDouble)) {
          setProperty(name, static_cast<double>(t_structured_value["double_value"]));
        }
        break;
      }
      case Type::String: {
        if (utils::checkXmlRpcSanity("str_value", t_structured_value, XmlRpc::XmlRpcValue::TypeString)) {
          setProperty(name, static_cast<std::string>(t_structured_value["str_value"]));
        }
        break;
      }
      default: {
        break;
      }
    }
  }
  else {
    m_params.valid = false;
  }
  return m_params.valid;
}

void rtask::commons::Property::setFromPropertyMsg(const rtask_msgs::PropertyConstPtr t_msg_ptr)
{
  setFromPropertyMsg(*t_msg_ptr);
};

void rtask::commons::Property::setFromPropertyMsg(const rtask_msgs::Property& t_msg)
{
  switch (t_msg.type) {
    case rtask_msgs::Property::BOOLEAN:
      m_params.valid = setName(t_msg.name) && setValue(t_msg.bool_value);
      break;
    case rtask_msgs::Property::INTEGER:
      m_params.valid = setName(t_msg.name) && setValue(static_cast<int>(t_msg.int_value));
      break;
    case rtask_msgs::Property::DOUBLE:
      m_params.valid = setName(t_msg.name) && setValue(t_msg.double_value);
      break;
    case rtask_msgs::Property::STRING:
      m_params.valid = setName(t_msg.name) && setValue(t_msg.str_value);
      break;
    default:
      m_params.valid = false;
  }
};

bool rtask::commons::Property::getValue(bool& t_value) const
{
  if (m_params.type != Type::Boolean) {
    return false;
  }
  t_value = m_params.bool_value;
  return true;
}
bool rtask::commons::Property::getValue(int& t_value) const
{
  if (m_params.type != Type::Integer) {
    return false;
  }
  t_value = m_params.int_value;
  return true;
}
bool rtask::commons::Property::getValue(double& t_value) const
{
  if (m_params.type != Type::Double) {
    return false;
  }
  t_value = m_params.double_value;
  return true;
}
bool rtask::commons::Property::getValue(std::string& t_value) const
{
  if (m_params.type != Type::String) {
    return false;
  }
  t_value = m_params.str_value;
  return true;
}

// ---------
// Operators
// ---------

bool rtask::commons::Property::operator==(const Property& t_property)
{
  if ((m_params.name == t_property.getName()) && m_params.type == t_property.getType()) {
    switch (m_params.type) {
      case Type::Boolean:
        return (m_params.bool_value == t_property.m_params.bool_value);
      case Type::Integer:
        return (m_params.int_value == t_property.m_params.int_value);
      case Type::Double:
        return (m_params.double_value == t_property.m_params.double_value);
      case Type::String:
        return (m_params.str_value == t_property.m_params.str_value);
      default:
        return false;
    }
  }
  else {
    return false;
  }
}

// -----------------
// PRIVATE FUNCTIONS
// -----------------

bool rtask::commons::Property::setName(const std::string& t_name)
{
  if (!t_name.empty()) {
    m_params.name = t_name;
    return true;
  }
  return false;
}
bool rtask::commons::Property::setValue(const bool t_value)
{
  m_params.type = Type::Boolean;
  m_params.bool_value = t_value;
  return true;
}
bool rtask::commons::Property::setValue(const int t_value)
{
  m_params.type = Type::Integer;
  m_params.int_value = t_value;
  return true;
}
bool rtask::commons::Property::setValue(const double t_value)
{
  m_params.type = Type::Double;
  m_params.double_value = t_value;
  return true;
}
bool rtask::commons::Property::setValue(const std::string& t_value)
{
  m_params.type = Type::String;
  m_params.str_value = t_value;
  return true;
}
