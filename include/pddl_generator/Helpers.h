#ifndef rtask_commons_pddl_generator_helpers_h
#define rtask_commons_pddl_generator_helpers_h

#include "xmlrpcpp/XmlRpc.h"

#include "LogicalExpression.h"

#include "Term.h"

#include <any>
#include <iostream>
#include <memory>
#include <string>
#include <unordered_map>

namespace rtask {
  namespace commons {
    namespace pddl_generator {

      enum class NumericalExpressionType
      {
        Invalid = 0,
        NumericExpressionBase,
        LiteralNumericalExpression,
        ArithmeticExpression,
        NumericalTerm
      };

      namespace helpers {

        bool checkXmlRpcSanity(const std::string& t_tag,
                               XmlRpc::XmlRpcValue& t_node,
                               const XmlRpc::XmlRpcValue::Type t_type,
                               bool allow_empty_string = false);

        XmlRpc::XmlRpcValue::Type getTagValueType(const std::string& t_tag, XmlRpc::XmlRpcValue& t_node);

        LogicalExpressionType getLogicalExprTypeFromXmlRpc(XmlRpc::XmlRpcValue& t_rpc_val);
        NumericalExpressionType getNumericalExprTypeFromXmlRpc(XmlRpc::XmlRpcValue& t_rpc_val);

        std::shared_ptr<LogicalExpression> getLogicalExprFromXmlRpc(XmlRpc::XmlRpcValue& t_rpc_val);
        // std::shared_ptr < NumericalExpression> getNumericalExprTypeFromXmlRpc(XmlRpc::XmlRpcValue& t_rpc_val);

        std::string logicalExprToPddl(std::shared_ptr<LogicalExpression> t_ptr, const bool t_typing = true);

        std::any getAsChild(Term& t_parent);
        std::any getAsChild(std::shared_ptr<Term> t_parent_ptr);

        std::any getAsChild(LogicalExpression& t_parent);
        std::any getAsChild(std::shared_ptr<LogicalExpression> t_parent_ptr);

        bool operator==(const LogicalExpression& t_first, const LogicalExpression& t_second);

      } // namespace helpers
      std::ostream& operator<<(std::ostream& t_out, std::shared_ptr<LogicalExpression> t_expr);

    } // namespace pddl_generator
  } // namespace commons
} // namespace rtask

#endif
