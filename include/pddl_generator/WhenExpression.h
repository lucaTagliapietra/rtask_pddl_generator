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
        inline void setCondition(LogicalExprPtr t_expr_ptr) { condition_ = t_expr_ptr; }
        inline void setConsequence(LogicalExprPtr t_expr_ptr) { consequence_ = t_expr_ptr; }

        inline std::string getExpressionName() const { return expr_name_; }
        inline LogicalExprPtr getCondition() const { return condition_; }
        inline LogicalExprPtr getConsequence() const { return consequence_; }

        bool isValid(UmapStrStr t_action_params,
                     const UmapStrStr& t_known_types,
                     const std::vector<LiteralTerm>& t_known_constants,
                     const std::vector<Predicate>& t_known_predicates,
                     const std::vector<LiteralExpression>& t_known_timeless,
                     const bool t_is_an_effect = false) const;

        std::string toPddl(bool t_typing = true, int t_pad_lv = 0) const override;

        WhenExpression& operator=(const WhenExpression& t_other);

      private:
        // string expression_name is already protected in parent class
        LogicalExprPtr condition_ = nullptr;
        LogicalExprPtr consequence_ = nullptr;
      };

      bool operator==(const WhenExpression& t_first, const WhenExpression& t_second);
      bool operator!=(const WhenExpression& t_first, const WhenExpression& t_second);
      std::ostream& operator<<(std::ostream& t_out, const WhenExpression& t_expr);
      std::ostream& operator<<(std::ostream& t_out, std::shared_ptr<WhenExpression> t_expr_ptr);

    } // namespace pddl_generator
  } // namespace commons
} // namespace rtask

#endif
