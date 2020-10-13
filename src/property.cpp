#include "commons/property.h"
#include "commons/utils.h"

// ------------
// CONSTRUCTORS
// ------------

rtask::commons::Property::Property(const std::string& t_name, bool t_val)
  : name_(t_name)
  , value_(t_val)
{}
rtask::commons::Property::Property(const std::string& t_name, int t_val)
  : name_(t_name)
  , value_(t_val){};
rtask::commons::Property::Property(const std::string& t_name, double t_val)
  : name_(t_name)
  , value_(t_val){};
rtask::commons::Property::Property(const std::string& t_name, std::string t_val)
  : name_(t_name)
  , value_(t_val){};

rtask::commons::Property::Property(const rtask_msgs::PropertyConstPtr t_msg_ptr)
{
  fromMsg(*t_msg_ptr);
}

rtask::commons::Property::Property(const rtask_msgs::Property& t_msg)
{
  fromMsg(t_msg);
}

rtask::commons::Property::Property(XmlRpc::XmlRpcValue& t_rpc_val)
{
  if (commons::utils::checkXmlRpcSanity("name", t_rpc_val, XmlRpc::XmlRpcValue::TypeString)) {
    std::string name = static_cast<std::string>(t_rpc_val["name"]);

    auto type = utils::getTagValueType("value", t_rpc_val);
    switch (type) {
      case XmlRpc::XmlRpcValue::TypeBoolean: {
        name_ = name;
        value_ = static_cast<bool>(t_rpc_val["value"]);
        break;
      }
      case XmlRpc::XmlRpcValue::TypeInt: {
        name_ = name;
        value_ = static_cast<int>(t_rpc_val["value"]);
        break;
      }
      case XmlRpc::XmlRpcValue::TypeDouble: {
        name_ = name;
        value_ = static_cast<double>(t_rpc_val["value"]);
        break;
      }
      case XmlRpc::XmlRpcValue::TypeString: {
        name_ = name;
        value_ = static_cast<int>(t_rpc_val["value"]);
        break;
      }
      default: {
        std::cout << "Malformed Property named: " << name << std::endl;
        break;
      }
    }
  }
}

void rtask::commons::Property::fromMsg(const rtask_msgs::Property& t_msg)
{
  name_ = t_msg.name;
  auto type = static_cast<PropertyType>(t_msg.type);
  switch (type) {
    case PropertyType::BOOL: {
      value_ = false;
      if (strcasecmp(t_msg.value.c_str(), "true") == 0 || atoi(t_msg.value.c_str()) != 0) {
        value_ = true;
      }
      break;
    }
    case PropertyType::INT: {
      value_ = atoi(t_msg.value.c_str());
      break;
    }
    case PropertyType::DOUBLE: {
      value_ = atof(t_msg.value.c_str());
      break;
    }
    case PropertyType::STRING: {
      value_ = t_msg.value;
      break;
    }
  }
}

rtask_msgs::Property rtask::commons::Property::toMsg() const
{
  rtask_msgs::Property msg;
  msg.name = name_;
  msg.type = static_cast<uint8_t>(value_.index());
  std::stringstream ss;
  std::visit([&ss](auto&& arg) { ss << arg; }, value_);
  msg.value = ss.str();
  return msg;
}

void rtask::commons::Property::updValue(const bool t_val)
{
  value_ = t_val;
}

void rtask::commons::Property::updValue(const int t_val)
{
  value_ = t_val;
}

void rtask::commons::Property::updValue(const double t_val)
{
  value_ = t_val;
}

void rtask::commons::Property::updValue(const std::string& t_val)
{
  value_ = t_val;
}

bool rtask::commons::Property::getValue(bool& t_val) const
{
  if (static_cast<PropertyType>(value_.index()) != PropertyType::BOOL) {
    return false;
  }
  t_val = std::get<bool>(value_);
  return true;
}

bool rtask::commons::Property::getValue(int& t_val) const
{
  if (static_cast<PropertyType>(value_.index()) != PropertyType::INT) {
    return false;
  }
  t_val = std::get<int>(value_);
  return true;
}
bool rtask::commons::Property::getValue(double& t_val) const
{
  if (static_cast<PropertyType>(value_.index()) != PropertyType::DOUBLE) {
    return false;
  }
  t_val = std::get<double>(value_);
  return true;
}
bool rtask::commons::Property::getValue(std::string& t_val) const
{
  if (static_cast<PropertyType>(value_.index()) != PropertyType::STRING) {
    return false;
  }
  t_val = std::get<std::string>(value_);
  return true;
}
