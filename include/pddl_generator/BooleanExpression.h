#ifndef rtask_commons_pddl_generator_boolean_expression_h
#define rtask_commons_pddl_generator_boolean_expression_h

#include "xmlrpcpp/XmlRpc.h"

#include <any>
#include <memory>
#include <string>
#include <unordered_map>

namespace rtask {
  namespace commons {
    namespace pddl_generator {

      using UnordStrToUIntMap = std::unordered_map<std::string, unsigned int>;

      enum class BooleanExpressionType
      {
        BooleanExpressionBase = 0,
        NotExpression = 1,
        AndExpression = 2,
        OrExpression = 3,
        CompareExpression = 4,
        WhenExpression = 5,
        ExistsExpression = 6,
        ForAllExpression = 7,
        LiteralBooleanExpression = 8
      };

      class BooleanExpression
      {
      public:
        BooleanExpression() = default;
        virtual ~BooleanExpression() {}

        virtual std::string toPddl(const bool t_typing = true) const { return {}; }

        inline std::string getExpressionName() const { return expression_name_; }
        inline BooleanExpressionType getExpressionType() const { return expr_type_; }

        virtual std::any getAsChild(BooleanExpression& t_parent) const;
        virtual std::any getAsChild(std::shared_ptr<BooleanExpression> t_parent_ptr) const;

      protected:
        std::string expression_name_{};
        BooleanExpressionType expr_type_{BooleanExpressionType::BooleanExpressionBase};
      };

    } // namespace pddl_generator
  } // namespace commons
} // namespace rtask

#endif
