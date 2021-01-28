#ifndef rtask_commons_pddl_generator_numerical_expression_h
#define rtask_commons_pddl_generator_numerical_expression_h

#include "xmlrpcpp/XmlRpc.h"

#include <memory>
#include <unordered_map>

namespace rtask {
  namespace commons {
    namespace pddl_generator {

      enum class NumericalExpressionType
      {
        Invalid = 0,
        Base,
        Function,
        Operator,
        Term
      };

      class NumericalExpression
      {
      public:
        NumericalExpression() = default;
        virtual ~NumericalExpression() {}

        virtual std::string toPddl(bool t_typing = true, int t_pad_lv = 0) const { return {}; }

        inline NumericalExpressionType getExpressionType() const { return expr_type_; }

      protected:
        NumericalExpressionType expr_type_{NumericalExpressionType::Base};
      };

      using NumericalExprPtr = std::shared_ptr<NumericalExpression>;

    } // namespace pddl_generator
  } // namespace commons
} // namespace rtask

#endif
