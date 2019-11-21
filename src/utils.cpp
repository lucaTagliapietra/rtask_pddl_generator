#include <iostream>

#include "commons/utils.h"

bool rtask::commons::utils::checkXmlRpcSanity(const std::string& t_tag,
                                              XmlRpc::XmlRpcValue& t_node,
                                              const XmlRpc::XmlRpcValue::Type t_type)
{
  if (!t_node.hasMember(t_tag)) {
    std::cout << "Tag: " << t_tag << " not found" << std::endl;
    return false;
  }
  if (t_node[t_tag].getType() != t_type) {
    std::cout << "Tag: " << t_tag << ". Type different from expected" << std::endl;
    return false;
  }
  if (t_type == XmlRpc::XmlRpcValue::TypeString) {
    if (!t_node[t_tag].valid() || t_node[t_tag] == "") {
      std::cout << "Tag: " << t_tag << ". Empty or not set string value not allowed" << std::endl;
      return false;
    }
  }
  else {
    if (!t_node[t_tag].valid()) {
      std::cout << "Tag: " << t_tag << ". Empty value not allowed." << std::endl;
      return false;
    }
  }
  return true;
}
