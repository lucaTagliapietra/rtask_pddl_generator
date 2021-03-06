#ifndef rtask_commons_pddl_generator_logical_expression_h
#define rtask_commons_pddl_generator_logical_expression_h

#include "xmlrpcpp/XmlRpc.h"

#include <memory>
#include <unordered_map>

namespace rtask {
  namespace commons {
    namespace pddl_generator {

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
        Arithmetic,
        Imply,
        Equals
      };

      class LogicalExpression
      {
      public:
        LogicalExpression() = default;
        virtual ~LogicalExpression() {}

        virtual std::string toPddl(bool t_typing = true, int t_pad_lv = 0) const { return {}; }

        inline std::string getExpressionName() const { return expr_name_; }
        inline LogicalExpressionType getExpressionType() const { return expr_type_; }

      protected:
        std::string expr_name_{};
        LogicalExpressionType expr_type_{LogicalExpressionType::Base};
      };

      using UnordStrToUIntMap = std::unordered_map<std::string, unsigned int>;
      using LogicalExprPtr = std::shared_ptr<LogicalExpression>;
      using LogicalExprPtrVector = std::vector<std::shared_ptr<LogicalExpression>>;

    } // namespace pddl_generator
  } // namespace commons
} // namespace rtask

#endif
