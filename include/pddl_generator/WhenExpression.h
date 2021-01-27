#ifndef rtask_commons_pddl_generator_when_expression_h
#define rtask_commons_pddl_generator_when_expression_h

#include "Helpers.h"
#include "LogicalExpression.h"
#include "xmlrpcpp/XmlRpc.h"

#include <iostream>
#include <memory>
#include <string>

namespace rtask {
  namespace commons {
    namespace pddl_generator {

      class WhenExpression : public LogicalExpression
      {
      public:
        WhenExpression();
        ~WhenExpression() override = default;

        WhenExpression(const LogicalExpression& t_condition, const LogicalExpression& t_consequence);
        WhenExpression(LogicalExprPtr t_condition_ptr, LogicalExprPtr t_consequence_ptr);
        WhenExpression(XmlRpc::XmlRpcValue& t_rpc_val);

        void clear();

        void set(LogicalExprPtr t_condition_ptr, LogicalExprPtr t_consequence_ptr);
        inline void setCondition(LogicalExprPtr t_expr) { condition_.reset(t_expr.get()); }
        inline void setConsequence(LogicalExprPtr t_expr) { consequence_.reset(t_expr.get()); }

        inline std::string getExpressionName() const { return expr_name_; }
        inline LogicalExprPtr getCondition() const { return condition_; }
        inline LogicalExprPtr getConsequence() const { return consequence_; }

        // bool validate(const UnordStrToLitTermMap& t_known_constants,
        //               const UnordStrToUIntMap& t_belonging_action_args,
        //               const std::string& t_belonging_action_name) const;

        std::string toPddl(const bool t_typing = true) const override;

        WhenExpression& operator=(const WhenExpression& t_other);

      private:
        // string expression_name is already protected in parent class
        LogicalExprPtr condition_ = nullptr;
        LogicalExprPtr consequence_ = nullptr;
      };

      bool operator==(const WhenExpression& t_first, const WhenExpression& t_second);

      static std::ostream& operator<<(std::ostream& t_out, const WhenExpression& t_expr)
      {
        t_out << "WhenExpression name: " << t_expr.getExpressionName() << std::endl;
        t_out << "\t - condition: " << t_expr.getCondition() << std::endl;
        t_out << "\t - consequence: " << t_expr.getConsequence();
        return t_out;
      }

    } // namespace pddl_generator
  } // namespace commons
} // namespace rtask

#endif
