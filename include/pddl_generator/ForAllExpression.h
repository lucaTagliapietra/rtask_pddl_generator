#ifndef rtask_commons_pddl_generator_forall_expression_h
#define rtask_commons_pddl_generator_forall_expression_h

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

      class ForAllExpression : public LogicalExpression
      {
      public:
        ForAllExpression();
        ~ForAllExpression() override = default;

        ForAllExpression(const LiteralTerm& t_what, const LogicalExpression& t_condition);
        ForAllExpression(LiteralTermPtr t_what_ptr, LogicalExprPtr t_condition_ptr);
        ForAllExpression(XmlRpc::XmlRpcValue& t_rpc_val);

        void clear();

        void set(LiteralTermPtr t_what_ptr, LogicalExprPtr t_condition_ptr);
        inline void setWhat(LiteralTermPtr t_what_ptr) { what_ = t_what_ptr; }
        inline void setCondition(LogicalExprPtr t_expr_ptr) { condition_ = t_expr_ptr; }

        inline std::string getExpressionName() const { return expr_name_; }
        inline LiteralTermPtr getWhat() const { return what_; }
        inline LogicalExprPtr getCondition() const { return condition_; }

        // bool validate(const UnordStrToLitTermMap& t_known_constants,
        //               const UnordStrToUIntMap& t_belonging_action_args,
        //               const std::string& t_belonging_action_name) const;

        std::string toPddl(bool t_typing = true, int t_pad_lv = 0) const override;

        ForAllExpression& operator=(const ForAllExpression& t_other);

      private:
        // string expression_name is already protected in parent class
        LiteralTermPtr what_ = nullptr;
        LogicalExprPtr condition_ = nullptr;
      };

      bool operator==(const ForAllExpression& t_first, const ForAllExpression& t_second);

      static std::ostream& operator<<(std::ostream& t_out, const ForAllExpression& t_expr)
      {
        t_out << "ForAllExpression name: " << t_expr.getExpressionName() << std::endl;
        t_out << "\t - what: " << t_expr.getWhat() << std::endl;
        t_out << "\t - condition: " << t_expr.getCondition();
        return t_out;
      }

    } // namespace pddl_generator
  } // namespace commons
} // namespace rtask

#endif
