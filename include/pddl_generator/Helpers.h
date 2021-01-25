#ifndef rtask_commons_pddl_generator_helpers_h
#define rtask_commons_pddl_generator_helpers_h

#include "xmlrpcpp/XmlRpc.h"

#include "BooleanExpression.h"

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

        BooleanExpressionType getBooleanExprTypeFromXmlRpc(XmlRpc::XmlRpcValue& t_rpc_val);
        NumericalExpressionType getNumericalExprTypeFromXmlRpc(XmlRpc::XmlRpcValue& t_rpc_val);

        bool checkXmlRpcSanity(const std::string& t_tag,
                               XmlRpc::XmlRpcValue& t_node,
                               const XmlRpc::XmlRpcValue::Type t_type,
                               bool allow_empty_string = false);

        XmlRpc::XmlRpcValue::Type getTagValueType(const std::string& t_tag, XmlRpc::XmlRpcValue& t_node);

        //        std::any getAsChild(BooleanExpression& t_parent) { return {}; }
        //        std::any getAsChild(std::shared_ptr<BooleanExpression> t_parent_ptr) { return {}; }

        //        std::any NumericalTerm::getAsChild(Term& t_parent) const
        //        {
        //          return {dynamic_cast<NumericalTerm&>(t_parent)};
        //        }

        //        std::any NumericalTerm::getAsChild(std::shared_ptr<Term> t_parent) const
        //        {
        //          return {std::dynamic_pointer_cast<NumericalTerm>(t_parent)};
        //        }

      } // namespace helpers

    } // namespace pddl_generator
  } // namespace commons
} // namespace rtask

#endif
