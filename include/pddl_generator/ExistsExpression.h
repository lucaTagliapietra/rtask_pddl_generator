#ifndef rtask_commons_pddl_generator_exists_expression_h
#define rtask_commons_pddl_generator_exists_expression_h

#include "Helpers.h"
#include "LiteralTerm.h"
#include "LogicalExpression.h"
#include "xmlrpcpp/XmlRpc.h"

#include <iostream>
#include <memory>
#include <string>

namespace rtask {
  namespace commons {
    namespace pddl_generator {

      class ExistsExpression : public LogicalExpression
      {
      public:
        ExistsExpression();
        ~ExistsExpression() override = default;

        ExistsExpression(const LiteralTerm& t_what, const LogicalExpression& t_condition);
        ExistsExpression(LiteralTermPtr t_what_ptr, LogicalExprPtr t_condition_ptr);
        ExistsExpression(XmlRpc::XmlRpcValue& t_rpc_val);

        void clear();

        void set(LiteralTermPtr t_what_ptr, LogicalExprPtr t_condition_ptr);
        inline void setWhat(LiteralTermPtr t_what_ptr) { what_ = t_what_ptr; }
        inline void setCondition(LogicalExprPtr t_expr) { condition_ = t_expr; }

        inline std::string getExpressionName() const { return expr_name_; }
        inline LiteralTermPtr getWhat() const { return what_; }
        inline LogicalExprPtr getCondition() const { return condition_; }

        bool isValid(UmapStrStr t_action_params,
                     const UmapStrStr& t_known_types,
                     const std::vector<LiteralTerm>& t_known_constants,
                     const std::vector<Predicate>& t_known_predicates,
                     const std::vector<LiteralExpression>& t_known_timeless,
                     const bool t_is_an_effect = false) const;

        std::string toPddl(bool t_typing = true, int t_pad_lv = 0) const override;

        ExistsExpression& operator=(const ExistsExpression& t_other);

      private:
        // string expression_name is already protected in parent class
        LiteralTermPtr what_ = nullptr;
        LogicalExprPtr condition_ = nullptr;
      };

      bool operator==(const ExistsExpression& t_first, const ExistsExpression& t_second);
      bool operator!=(const ExistsExpression& t_first, const ExistsExpression& t_second);
      std::ostream& operator<<(std::ostream& t_out, const ExistsExpression& t_expr);
      std::ostream& operator<<(std::ostream& t_out, std::shared_ptr<ExistsExpression> t_expr_ptr);

    } // namespace pddl_generator
  } // namespace commons
} // namespace rtask

#endif
