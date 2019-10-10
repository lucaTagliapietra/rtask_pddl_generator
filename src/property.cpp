#include "commons/property.h"

rtask::commons::Property::Property()
  : m_params({})
{}

rtask::commons::Property::Property(const std::string& t_name, const bool t_value)
{
  m_params.name = t_name;
  m_params.type = Type::Boolean;
  m_params.bool_value = t_value;
};

rtask::commons::Property::Property(const std::string& t_name, const int t_value)
{
  m_params.name = t_name;
  m_params.type = Type::Integer;
  m_params.int_value = t_value;
};

rtask::commons::Property::Property(const std::string& t_name, const double t_value)
{
  m_params.name = t_name;
  m_params.type = Type::Double;
  m_params.double_value = t_value;
};

rtask::commons::Property::Property(const std::string& t_name, const std::string& t_value)
{
  m_params.name = t_name;
  m_params.type = Type::String;
  m_params.str_value = t_value;
};

rtask::commons::Property::Property(const rtask_msgs::PropertyConstPtr t_msg)
{
  setFromPropertyMsg(t_msg);
}

rtask::commons::Property::Property(const rtask_msgs::Property& t_property_msg)
{
  setFromPropertyMsg(t_property_msg);
}

rtask_msgs::PropertyPtr rtask::commons::Property::toPropertyMsg() const
{

  rtask_msgs::PropertyPtr a_property{};
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

void rtask::commons::Property::setFromPropertyMsg(const rtask_msgs::PropertyConstPtr t_msg_ptr)
{
  setFromPropertyMsg(*t_msg_ptr);
};

void rtask::commons::Property::setFromPropertyMsg(const rtask_msgs::Property& t_msg)
{
  m_params.name = t_msg.name;
  switch (t_msg.type) {
    case rtask_msgs::Property::BOOLEAN:
      m_params.type = Type::Boolean;
      m_params.bool_value = t_msg.bool_value;
      break;
    case rtask_msgs::Property::INTEGER:
      m_params.type = Type::Integer;
      m_params.int_value = static_cast<int>(t_msg.int_value);
      break;
    case rtask_msgs::Property::DOUBLE:
      m_params.type = Type::Double;
      m_params.double_value = t_msg.double_value;
      break;
    case rtask_msgs::Property::STRING:
      m_params.type = Type::String;
      m_params.str_value = t_msg.str_value;
      break;
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

void rtask::commons::Property::setName(const std::string& t_name)
{
  m_params.name = t_name;
}
void rtask::commons::Property::setValue(const bool t_value)
{
  m_params.type = Type::Boolean;
  m_params.bool_value = t_value;
}
void rtask::commons::Property::setValue(const int t_value)
{
  m_params.type = Type::Integer;
  m_params.int_value = t_value;
}
void rtask::commons::Property::setValue(const double t_value)
{
  m_params.type = Type::Double;
  m_params.double_value = t_value;
}
void rtask::commons::Property::setValue(const std::string& t_value)
{
  m_params.type = Type::String;
  m_params.str_value = t_value;
}
