#ifndef rtask_commons_pddl_generator_numerical_operator_h
#define rtask_commons_pddl_generator_numerical_operator_h

#include "NumericalExpression.h"

#include "xmlrpcpp/XmlRpc.h"

#include <iostream>

namespace rtask {
  namespace commons {
    namespace pddl_generator {

      class NumericalOperator : public NumericalExpression
      {
      public:
        NumericalOperator();
        ~NumericalOperator() override = default;

        NumericalOperator(const std::string& t_op_name,
                          const NumericalExpression& t_lhs,
                          const NumericalExpression& t_rhs);
        NumericalOperator(const std::string& t_op_name, NumericalExprPtr t_lhs_ptr, NumericalExprPtr t_rhs);
        NumericalOperator(XmlRpc::XmlRpcValue& t_rpc_val);

        void clear();
        bool set(const std::string& t_op_name, NumericalExprPtr t_lhs_ptr, NumericalExprPtr t_rhs);

        bool setOperatorName(const std::string& t_op_name);
        void setLhsExpression(NumericalExprPtr t_lhs_ptr) { lhs_expr_.reset(t_lhs_ptr.get()); }
        void setRhsExpression(NumericalExprPtr t_rhs_ptr) { rhs_expr_.reset(t_rhs_ptr.get()); }

        inline std::string getOperatorName() const { return operator_name_; }
        inline NumericalExprPtr getLhsExpression() const { return lhs_expr_; }
        inline NumericalExprPtr getRhsExpression() const { return rhs_expr_; }

        //        bool validate(const UnordStrToLitTermMap& t_known_constants,
        //                      const UnordStrToUIntMap& t_belonging_action_args,
        //                      const std::string& t_belonging_action_name) const;

        std::string toPddl(bool t_typing = true, int t_pad_lv = 0) const override;

        NumericalOperator& operator=(const NumericalOperator& t_other);

      private:
        std::string operator_name_{};
        NumericalExprPtr lhs_expr_ = nullptr;
        NumericalExprPtr rhs_expr_ = nullptr;
      };

      bool operator==(const NumericalOperator& t_first, const NumericalOperator& t_second);
      std::ostream& operator<<(std::ostream& t_out, const NumericalOperator& t_expr);
    } // namespace pddl_generator
  } // namespace commons
} // namespace rtask

#endif
