#ifndef rtask_commons_pddl_generator_void_expression_h
#define rtask_commons_pddl_generator_void_expression_h

#include "xmlrpcpp/XmlRpc.h"

#include <unordered_map>

namespace rtask {
  namespace commons {
    namespace pddl_generator {

      enum class VoidExpressionType
      {
        Invalid = 0,
        VoidExpressionBase,
        NotExpression,
        AndExpression,
        OrExpression,
        WhenExpression,
        LiteralBooleanExpression,
        ArithmeticExpression
      };

      class VoidExpression
      {
      public:
        VoidExpression() = default;
        virtual ~VoidExpression() {}

        virtual std::string toPddl(const bool t_typing = true) const { return {}; }

        inline std::string getExpressionName() const { return expr_name_; }
        inline VoidExpressionType getExpressionType() const { return expr_type_; }

      protected:
        std::string expr_name_{};
        VoidExpressionType expr_type_{VoidExpressionType::VoidExpressionBase};
      };

    } // namespace pddl_generator
  } // namespace commons
} // namespace rtask

#endif
