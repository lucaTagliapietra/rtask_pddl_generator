#ifndef rtask_commons_pddl_generator_logical_expression_h
#define rtask_commons_pddl_generator_logical_expression_h

#include "xmlrpcpp/XmlRpc.h"

#include <unordered_map>

namespace rtask {
  namespace commons {
    namespace pddl_generator {

      using UnordStrToUIntMap = std::unordered_map<std::string, unsigned int>;

      enum class LogicalExpressionType
      {
        Invalid = 0,
        Base,
        Literal,
        Not,
        And,
        Or,
        Compare,
        When,
        Exists,
        ForAll,
        Arithmetic
      };

      class LogicalExpression
      {
      public:
        LogicalExpression() = default;
        virtual ~LogicalExpression() {}

        virtual std::string toPddl(const bool t_typing = true) const { return {}; }

        inline std::string getExpressionName() const { return expr_name_; }
        inline LogicalExpressionType getExpressionType() const { return expr_type_; }

      protected:
        std::string expr_name_{};
        LogicalExpressionType expr_type_{LogicalExpressionType::Base};
      };

    } // namespace pddl_generator
  } // namespace commons
} // namespace rtask

#endif
