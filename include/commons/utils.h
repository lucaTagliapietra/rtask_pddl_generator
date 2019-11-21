#include <string>

#include "xmlrpcpp/XmlRpcValue.h"

namespace rtask {
  namespace commons {
    namespace utils {
      bool
      checkXmlRpcSanity(const std::string& t_tag, XmlRpc::XmlRpcValue& t_node, const XmlRpc::XmlRpcValue::Type t_type);
    }
  } // namespace commons
} // namespace rtask
