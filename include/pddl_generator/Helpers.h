#ifndef rtask_commons_pddl_generator_helpers_h
#define rtask_commons_pddl_generator_helpers_h

#include "xmlrpcpp/XmlRpc.h"

#include "LiteralExpression.h"
#include "LogicalExpression.h"
#include "NumericalExpression.h"
#include "Predicate.h"

#include "Term.h"

#include <any>
#include <iostream>
#include <memory>
#include <string>

namespace rtask {
  namespace commons {
    namespace pddl_generator {

      using UmapStrVecStr = std::unordered_map<std::string, std::vector<std::string>>;

      namespace helpers {

        const std::string PAD = std::string("\t");

        bool checkXmlRpcSanity(const std::string& t_tag,
                               XmlRpc::XmlRpcValue& t_node,
                               const XmlRpc::XmlRpcValue::Type t_type,
                               bool allow_empty_string = false);
        bool hasValidXmlRpcTag(const std::string& t_tag,
                               XmlRpc::XmlRpcValue& t_node,
                               const XmlRpc::XmlRpcValue::Type t_type = XmlRpc::XmlRpcValue::Type::TypeInvalid);

        XmlRpc::XmlRpcValue::Type getTagValueType(const std::string& t_tag, XmlRpc::XmlRpcValue& t_node);

        LogicalExpressionType getLogicalExprTypeFromXmlRpc(XmlRpc::XmlRpcValue& t_rpc_val);
        NumericalExpressionType getNumericalExprTypeFromXmlRpc(XmlRpc::XmlRpcValue& t_rpc_val);

        LogicalExprPtr getLogicalExprFromXmlRpc(XmlRpc::XmlRpcValue& t_rpc_val);
        NumericalExprPtr getNumericalExprFromXmlRpc(XmlRpc::XmlRpcValue& t_rpc_val);

        std::string logicalExprToPddl(LogicalExprPtr t_ptr, bool t_typing = true, int t_pad_lv = 0);
        std::string numericalExprToPddl(NumericalExprPtr t_ptr, bool t_typing = true, int t_pad_lv = 0);

        std::any getAsChild(Term& t_parent);
        std::any getAsChild(std::shared_ptr<Term> t_parent_ptr);

        std::any getAsChild(LogicalExpression& t_parent);
        std::any getAsChild(LogicalExprPtr t_parent_ptr);

        std::any getAsChild(NumericalExpression& t_parent);
        std::any getAsChild(NumericalExprPtr t_parent_ptr);

        bool isValid(LogicalExprPtr t_expr_ptr,
                     const UmapStrStr& t_action_params,
                     const UmapStrStr& t_known_types,
                     const std::vector<LiteralTerm>& t_known_constants,
                     const std::vector<Predicate>& t_known_predicates,
                     const std::vector<LiteralExpression>& t_known_timeless,
                     bool t_is_an_effect = false);

        bool operator==(const LogicalExpression& t_first, const LogicalExpression& t_second);
        bool operator==(const NumericalExpression& t_first, const NumericalExpression& t_second);
        bool operator!=(const LogicalExpression& t_first, const LogicalExpression& t_second);
        bool operator!=(const NumericalExpression& t_first, const NumericalExpression& t_second);

        std::string padding(int t_n_pads);
        std::pair<int, std::vector<std::string>> getPddlAligners(int t_pad_lv);

        UmapStrVecStr buildTypesHierarchy(const UmapStrStr& t_known_types);
        UmapStrVecStr aggregateByParentType(const UmapStrStr& t_known_types);

      } // namespace helpers

      std::ostream& operator<<(std::ostream& t_out, LogicalExprPtr t_expr);
      std::ostream& operator<<(std::ostream& t_out, NumericalExprPtr t_expr);

      // <LEType, supportedAsCondition, supportedAsEffect>
      const static inline std::map<LogicalExpressionType, std::pair<bool, bool>> LogicalExprSupportedAs = {
        {LogicalExpressionType::Literal, {true, true}},
        {LogicalExpressionType::And, {true, true}},
        {LogicalExpressionType::Or, {true, false}},
        {LogicalExpressionType::Not, {true, true}},
        {LogicalExpressionType::When, {true, true}},
        {LogicalExpressionType::Exists, {true, false}},
        {LogicalExpressionType::ForAll, {true, false}},
        {LogicalExpressionType::Compare, {true, false}},
        {LogicalExpressionType::Arithmetic, {false, true}},
        {LogicalExpressionType::Imply, {true, false}},
        {LogicalExpressionType::Equals, {true, false}}};

    } // namespace pddl_generator
  } // namespace commons
} // namespace rtask

#endif
