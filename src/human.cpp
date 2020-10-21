#include "commons/human.h"
#include "commons/utils.h"

// ------------
// CONSTRUCTORS
rtask::commons::Human::Human(const std::string& t_name,
                             const std::vector<Device> t_tools,
                             const std::vector<Property> t_extra_properties)
{
  set(t_name, t_tools, t_extra_properties);
}

rtask::commons::Human::Human(const rtask_msgs::Human& t_msg)
{
  fromMsg(t_msg);
}

rtask::commons::Human::Human(const rtask_msgs::HumanConstPtr t_msg_ptr)
{
  fromMsg(*t_msg_ptr);
}

rtask::commons::Human::Human(XmlRpc::XmlRpcValue& t_rpc_val)
{
  if (commons::utils::checkXmlRpcSanity("name", t_rpc_val, XmlRpc::XmlRpcValue::TypeString)) {
    name_ = static_cast<std::string>(t_rpc_val["name"]);
  }

  if (commons::utils::checkXmlRpcSanity("tools", t_rpc_val, XmlRpc::XmlRpcValue::TypeArray)) {
    if (t_rpc_val["tools"].size() == 0) {
      std::cout << "Empty tools vector for Human " << name_ << std::endl;
    }
    else {
      for (int i = 0; i < t_rpc_val["tools"].size(); ++i) {
        Device t(t_rpc_val["tools"][i]);
        if (t.isValid()) {
          tools_.push_back(t);
        }
      }
    }
  }
  if (commons::utils::checkXmlRpcSanity("extra_properties", t_rpc_val, XmlRpc::XmlRpcValue::TypeArray)) {
    if (t_rpc_val["extra_properties"].size() == 0) {
      std::cout << "Empty extra_properties vector for Device " << name_ << std::endl;
    }
    else {
      for (int i = 0; i < t_rpc_val["extra_properties"].size(); ++i) {
        Property p(t_rpc_val["extra_properties"][i]);
        if (p.isValid()) {
          extra_properties_.push_back(p);
        }
      }
    }
  }
  updValidity();
}

rtask_msgs::Human rtask::commons::Human::toMsg() const
{
  rtask_msgs::Human msg;
  msg.name = name_;
  for (const auto& t : tools_) {
    msg.tools.emplace_back(t.toMsg());
  }

  for (const auto& ep : extra_properties_) {
    msg.extra_properties.emplace_back(ep.toMsg());
  }
  return msg;
}

void rtask::commons::Human::fromMsg(const rtask_msgs::Human& t_msg)
{
  name_ = t_msg.name;
  for (const auto& t : t_msg.tools) {
    tools_.emplace_back(t);
  }
  for (const auto& ep : t_msg.extra_properties) {
    extra_properties_.emplace_back(ep);
  }

  updValidity();
}

void rtask::commons::Human::clear()
{
  name_.clear();
  tools_.clear();
  extra_properties_.clear();
}

void rtask::commons::Human::set(const std::string& t_name,
                                const std::vector<Device>& t_tools,
                                const std::vector<Property> t_extra_properties)
{
  name_ = t_name;
  tools_ = t_tools;
  extra_properties_ = t_extra_properties;
  updValidity();
}

std::vector<std::string> rtask::commons::Human::getExtraPropertyList() const
{
  std::vector<std::string> out;
  std::for_each(
    extra_properties_.begin(), extra_properties_.end(), [&out](const auto& p) { out.emplace_back(p.getName()); });
  return out;
}

std::vector<std::string> rtask::commons::Human::getToolList() const
{
  std::vector<std::string> out;
  std::for_each(tools_.begin(), tools_.end(), [&out](const auto& t) { out.emplace_back(t.getName()); });
  return out;
}

bool rtask::commons::Human::hasExtraProperty(const std::string& t_name) const
{
  auto it = std::find_if(
    extra_properties_.begin(), extra_properties_.end(), [t_name](auto& p) { return p.getName() == t_name; });
  return it != extra_properties_.end();
}

bool rtask::commons::Human::deleteExtraProperty(const std::string& t_name)
{
  auto it = std::find_if(
    extra_properties_.begin(), extra_properties_.end(), [t_name](auto& p) { return p.getName() == t_name; });
  if (it == extra_properties_.end()) {
    return false;
  }

  extra_properties_.erase(it);
  updValidity();
  return true;
}

bool rtask::commons::Human::isExtraPropertyValid(const std::string& t_name) const
{
  auto it = std::find_if(
    extra_properties_.begin(), extra_properties_.end(), [t_name](auto& p) { return p.getName() == t_name; });
  if (it == extra_properties_.end()) {
    return false;
  }
  return it->isValid();
}

std::pair<bool, rtask::commons::Property> rtask::commons::Human::getExtraProperty(const std::string& t_name) const
{
  auto it = std::find_if(
    extra_properties_.begin(), extra_properties_.end(), [t_name](auto& p) { return p.getName() == t_name; });
  if (it == extra_properties_.end()) {
    return {false, {}};
  }
  return {true, *it};
}

void rtask::commons::Human::setExtraProperty(const std::string& t_name, const PropertyVariant& t_val)
{
  auto it = std::find_if(
    extra_properties_.begin(), extra_properties_.end(), [t_name](auto& p) { return p.getName() == t_name; });

  if (it == extra_properties_.end()) {
    extra_properties_.emplace_back(t_name, t_val);
  }
  it->setValue(t_val);
  updValidity();
}

bool rtask::commons::Human::hasTool(const std::string& t_name) const
{
  auto it = std::find_if(tools_.begin(), tools_.end(), [t_name](auto& d) { return d.getName() == t_name; });
  return it != tools_.end();
}

bool rtask::commons::Human::deleteTool(const std::string& t_name)
{
  auto it = std::find_if(tools_.begin(), tools_.end(), [t_name](auto& d) { return d.getName() == t_name; });
  if (it == tools_.end()) {
    return false;
  }

  tools_.erase(it);
  updValidity();
  return true;
}

bool rtask::commons::Human::isToolValid(const std::string& t_name) const
{
  auto it = std::find_if(tools_.begin(), tools_.end(), [t_name](auto& t) { return t.getName() == t_name; });
  if (it == tools_.end()) {
    return false;
  }
  return it->isValid();
}

std::pair<bool, rtask::commons::Device> rtask::commons::Human::getTool(const std::string& t_name) const
{
  auto it = std::find_if(tools_.begin(), tools_.end(), [t_name](auto& t) { return t.getName() == t_name; });
  if (it == tools_.end()) {
    return {false, {}};
  }
  return {true, *it};
}

void rtask::commons::Human::setTool(const std::string& t_name,
                                    const DeviceClass& t_tool_class,
                                    const std::string& t_tool_subclass,
                                    const std::vector<Property> t_tool_unique_props,
                                    const std::vector<Property> t_tool_extra_properties,
                                    const std::vector<Capability> t_tool_capabilities)
{
  auto it = std::find_if(tools_.begin(), tools_.end(), [t_name](auto& t) { return t.getName() == t_name; });

  if (it == tools_.end()) {
    tools_.emplace_back(
      t_name, t_tool_class, t_tool_subclass, t_tool_unique_props, t_tool_extra_properties, t_tool_capabilities);
  }
  it->set(t_name, t_tool_class, t_tool_subclass, t_tool_unique_props, t_tool_extra_properties, t_tool_capabilities);
  updValidity();
}

void rtask::commons::Human::setTool(const std::string& t_name, const Device& t_tool)
{
  auto it = std::find_if(tools_.begin(), tools_.end(), [t_name](auto& t) { return t.getName() == t_name; });

  if (it == tools_.end()) {
    tools_.emplace_back(t_tool);
  }
  it->set(t_name,
          t_tool.getClass(),
          t_tool.getSubclass(),
          t_tool.getUniqueProperties(),
          t_tool.getExtraProperties(),
          t_tool.getCapabilities());
  updValidity();
}

bool rtask::commons::Human::operator==(const rtask::commons::Human& t_h) const
{
  if (!(name_ == t_h.getName() && valid_ == t_h.isValid())) {
    return false;
  }

  auto other_tools = t_h.getTools();
  for (const auto& t : tools_) {
    auto it = std::find_if(other_tools.begin(), other_tools.end(), [t](auto& ot) { return t == ot; });
    if (it == other_tools.end()) {
      return false;
    }
  }
  return true;
}

rtask::commons::Human& rtask::commons::Human::operator=(const rtask::commons::Human& t_h)
{
  name_ = t_h.getName();
  valid_ = t_h.isValid();
  extra_properties_ = t_h.getExtraProperties();
  tools_ = t_h.getTools();
  return *this;
}

void rtask::commons::Human::updValidity()
{
  valid_ = !name_.empty();
  std::for_each(extra_properties_.begin(), extra_properties_.end(), [this](const auto& p) { valid_ &= p.isValid(); });
  std::for_each(tools_.begin(), tools_.end(), [this](const auto& t) { valid_ &= t.isValid(); });
}
