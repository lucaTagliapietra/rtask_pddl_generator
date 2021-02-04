#ifndef rtask_commons_pddl_generator_compare_expression_h
#define rtask_commons_pddl_generator_compare_expression_h

#include "Helpers.h"
#include "LogicalExpression.h"
#include "NumericalExpression.h"
#include "xmlrpcpp/XmlRpc.h"

#include <iostream>
#include <memory>
#include <string>

namespace rtask {
  namespace commons {
    namespace pddl_generator {

      class CompareExpression : public LogicalExpression
      {
      public:
        CompareExpression();
        ~CompareExpression() override = default;

        CompareExpression(const std::string& t_op_name,
                          const NumericalExpression& t_lhs,
                          const NumericalExpression& t_rhs);
        CompareExpression(const std::string& t_op_name, NumericalExprPtr t_lhs, NumericalExprPtr t_rhs);
        CompareExpression(XmlRpc::XmlRpcValue& t_rpc_val);

        void clear();

        bool set(const std::string& t_op_name, NumericalExprPtr t_lhs, NumericalExprPtr t_rhs);

        bool setComparisonOperator(const std::string& t_op_name);
        inline void setLhsExpression(NumericalExprPtr t_lhs) { lhs_expr_ = t_lhs; }
        inline void setRhsExpression(NumericalExprPtr t_rhs) { rhs_expr_ = t_rhs; }

        inline std::string getExpressionName() const = delete;
        inline std::string getComparisonOperator() const { return expr_name_; }
        inline NumericalExprPtr getLhsExpression() const { return lhs_expr_; }
        inline NumericalExprPtr getRhsExpression() const { return rhs_expr_; }

        CompareExpression mirror() const;
        bool equals(const CompareExpression& t_other) const;

        // bool validate(const UnordStrToLitTermMap& t_known_constants,
        //               const UnordStrToUIntMap& t_belonging_action_args,
        //               const std::string& t_belonging_action_name) const;

        std::string toPddl(bool t_typing = true, int t_pad_lv = 0) const override;

        CompareExpression& operator=(const CompareExpression& t_other);

      private:
        // string expression_name is already protected in parent class
        NumericalExprPtr lhs_expr_ = nullptr;
        NumericalExprPtr rhs_expr_ = nullptr;
      };

      bool operator==(const CompareExpression& t_first, const CompareExpression& t_second);
      std::ostream& operator<<(std::ostream& t_out, const CompareExpression& t_expr);

    } // namespace pddl_generator
  } // namespace commons
} // namespace rtask

#endif
