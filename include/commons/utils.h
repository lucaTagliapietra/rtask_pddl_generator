#ifndef rtask_commons_utils_h
#define rtask_commons_utils_h

#include <string>

#include "xmlrpcpp/XmlRpcValue.h"

namespace rtask {
  namespace commons {
    namespace utils {
      bool
      checkXmlRpcSanity(const std::string& t_tag, XmlRpc::XmlRpcValue& t_node, const XmlRpc::XmlRpcValue::Type t_type);
      XmlRpc::XmlRpcValue::Type getTagValueType(const std::string& t_tag, XmlRpc::XmlRpcValue& t_node);

    } // namespace utils
  } // namespace commons
} // namespace rtask

#endif
