#ifndef rtask_commons_pddl_generator_helpers_h
#define rtask_commons_pddl_generator_helpers_h

#include "xmlrpcpp/XmlRpc.h"

#include "LogicalExpression.h"
#include "NumericalExpression.h"

#include "Term.h"

#include <any>
#include <iostream>
#include <memory>
#include <string>
#include <unordered_map>

namespace rtask {
  namespace commons {
    namespace pddl_generator {

      namespace helpers {

        const std::string PAD = std::string("\t");

        bool checkXmlRpcSanity(const std::string& t_tag,
                               XmlRpc::XmlRpcValue& t_node,
                               const XmlRpc::XmlRpcValue::Type t_type,
                               bool allow_empty_string = false);

        XmlRpc::XmlRpcValue::Type getTagValueType(const std::string& t_tag, XmlRpc::XmlRpcValue& t_node);

        LogicalExpressionType getLogicalExprTypeFromXmlRpc(XmlRpc::XmlRpcValue& t_rpc_val);
        NumericalExpressionType getNumericalExprTypeFromXmlRpc(XmlRpc::XmlRpcValue& t_rpc_val);

        std::shared_ptr<LogicalExpression> getLogicalExprFromXmlRpc(XmlRpc::XmlRpcValue& t_rpc_val);
        // std::shared_ptr < NumericalExpression> getNumericalExprTypeFromXmlRpc(XmlRpc::XmlRpcValue& t_rpc_val);

        std::string logicalExprToPddl(LogicalExprPtr t_ptr, bool t_typing = true, int t_pad_lv = 0);

        std::any getAsChild(Term& t_parent);
        std::any getAsChild(std::shared_ptr<Term> t_parent_ptr);

        std::any getAsChild(LogicalExpression& t_parent);
        std::any getAsChild(std::shared_ptr<LogicalExpression> t_parent_ptr);

        bool operator==(const LogicalExpression& t_first, const LogicalExpression& t_second);

        std::string padding(int t_n_pads);
        std::pair<int, std::vector<std::string>> getPddlAligners(int t_pad_lv);

      } // namespace helpers
      std::ostream& operator<<(std::ostream& t_out, std::shared_ptr<LogicalExpression> t_expr);

    } // namespace pddl_generator
  } // namespace commons
} // namespace rtask

#endif
