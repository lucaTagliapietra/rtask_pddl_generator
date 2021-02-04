#ifndef rtask_commons_pddl_generator_imply_expression_h
#define rtask_commons_pddl_generator_imply_expression_h

#include "Helpers.h"
#include "LogicalExpression.h"
#include "xmlrpcpp/XmlRpc.h"

#include <iostream>
#include <memory>
#include <string>

namespace rtask {
  namespace commons {
    namespace pddl_generator {

      class ImplyExpression : public LogicalExpression
      {
      public:
        ImplyExpression();
        ~ImplyExpression() override = default;

        ImplyExpression(const LogicalExpression& t_condition, const LogicalExpression& t_consequence);
        ImplyExpression(LogicalExprPtr t_condition_ptr, LogicalExprPtr t_consequence_ptr);
        ImplyExpression(XmlRpc::XmlRpcValue& t_rpc_val);

        void clear();

        void set(LogicalExprPtr t_condition_ptr, LogicalExprPtr t_consequence_ptr);
        inline void setCondition(LogicalExprPtr t_expr_ptr) { condition_ = t_expr_ptr; }
        inline void setConsequence(LogicalExprPtr t_expr_ptr) { consequence_ = t_expr_ptr; }

        inline std::string getExpressionName() const { return expr_name_; }
        inline LogicalExprPtr getCondition() const { return condition_; }
        inline LogicalExprPtr getConsequence() const { return consequence_; }

        // bool validate(const UnordStrToLitTermMap& t_known_constants,
        //               const UnordStrToUIntMap& t_belonging_action_args,
        //               const std::string& t_belonging_action_name) const;

        std::string toPddl(bool t_typing = true, int t_pad_lv = 0) const override;

        ImplyExpression& operator=(const ImplyExpression& t_other);

      private:
        // string expression_name is already protected in parent class
        LogicalExprPtr condition_ = nullptr;
        LogicalExprPtr consequence_ = nullptr;
      };

      bool operator==(const ImplyExpression& t_first, const ImplyExpression& t_second);
      std::ostream& operator<<(std::ostream& t_out, const ImplyExpression& t_expr);
      std::ostream& operator<<(std::ostream& t_out, std::shared_ptr<ImplyExpression> t_expr_ptr);

    } // namespace pddl_generator
  } // namespace commons
} // namespace rtask

#endif
