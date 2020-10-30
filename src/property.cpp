#include "commons/property.h"
#include "commons/utils.h"

// ------------
// CONSTRUCTORS
// ------------

rtask::commons::Property::Property(const std::string& t_name, const PropertyVariant& t_val)
  : name_(t_name)
  , value_(t_val)
{
  valid_ = !name_.empty() && !value_.valueless_by_exception();
};

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
        value_ = static_cast<std::string>(t_rpc_val["value"]);
        break;
      }
      default: {
        std::cout << "Malformed Property named: " << name << std::endl;
        std::cout << " Invalid Property" << std::endl;
        valid_ = false;
        break;
      }
    }
    valid_ = true;
  }
  else {
    std::cout << " Invalid Property" << std::endl;
    valid_ = false;
  }
}

void rtask::commons::Property::fromMsg(const rtask_msgs::Property& t_msg)
{
  name_ = t_msg.name;
  if (name_.empty()) {
    std::cout << " Invalid Property" << std::endl;
    valid_ = false;
    return;
  }
  valid_ = true;
  auto type = static_cast<PropertyType>(t_msg.type);
  switch (type) {
    case PropertyType::BOOL: {
      try {
        if (strcasecmp(t_msg.value.c_str(), "true") == 0 || std::stoi(t_msg.value) != 0) {
          value_ = true;
        }
        value_ = false;
      }
      catch (std::exception const& e) {
        std::cout << e.what() << " Invalid Property" << std::endl;
        valid_ = false;
      }
      break;
    }
    case PropertyType::INT: {
      try {
        value_ = std::stoi(t_msg.value);
      }
      catch (std::exception const& e) {
        std::cout << e.what() << " Invalid Property" << std::endl;
        valid_ = false;
      }
      break;
    }

    case PropertyType::DOUBLE: {
      try {
        value_ = std::stod(t_msg.value);
      }
      catch (std::exception const& e) {
        std::cout << e.what() << " Invalid Property" << std::endl;
        valid_ = false;
      }
      break;
    }
    case PropertyType::STRING: {
      if (t_msg.value.empty()) {
        valid_ = false;
        std::cout << " Invalid Property" << std::endl;
      }
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

void rtask::commons::Property::set(const std::string& t_name, const PropertyVariant& t_val)
{
  name_ = t_name;
  value_ = t_val;
  valid_ = !name_.empty() && !value_.valueless_by_exception();
}

void rtask::commons::Property::setValue(const PropertyVariant& t_val)
{
  value_ = t_val;
  valid_ = !name_.empty() && !value_.valueless_by_exception();
}

std::pair<bool, rtask::commons::PropertyVariant> rtask::commons::Property::getValue() const
{
  return {valid_, value_};
}

bool rtask::commons::Property::operator==(const Property& t_property) const
{
  return (name_ == t_property.getName()) && (valid_ == t_property.isValid())
         && (t_property.getValue().second == value_);
}

rtask::commons::Property& rtask::commons::Property::operator=(const Property& t_property)
{
  name_ = t_property.getName();
  valid_ = t_property.isValid();
  value_ = t_property.getValue().second;
  return *this;
}
