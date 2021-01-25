#ifndef rtask_commons_pddl_generator_boolean_expression_h
#define rtask_commons_pddl_generator_boolean_expression_h

#include "xmlrpcpp/XmlRpc.h"

#include <unordered_map>

namespace rtask {
  namespace commons {
    namespace pddl_generator {

      using UnordStrToUIntMap = std::unordered_map<std::string, unsigned int>;

      enum class BooleanExpressionType
      {
        Invalid = 0,
        BooleanExpressionBase,
        NotExpression,
        AndExpression,
        OrExpression,
        CompareExpression,
        WhenExpression,
        ExistsExpression,
        ForAllExpression,
        LiteralBooleanExpression
      };

      class BooleanExpression
      {
      public:
        BooleanExpression() = default;
        virtual ~BooleanExpression() {}

        virtual std::string toPddl(const bool t_typing = true) const { return {}; }

        inline std::string getExpressionName() const { return expression_name_; }
        inline BooleanExpressionType getExpressionType() const { return expr_type_; }

      protected:
        std::string expression_name_{};
        BooleanExpressionType expr_type_{BooleanExpressionType::BooleanExpressionBase};
      };

    } // namespace pddl_generator
  } // namespace commons
} // namespace rtask

#endif
