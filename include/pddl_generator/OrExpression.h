#ifndef rtask_commons_pddl_generator_or_expression_h
#define rtask_commons_pddl_generator_or_expression_h

#include "Helpers.h"
#include "LogicalExpression.h"
#include "xmlrpcpp/XmlRpc.h"

#include <iostream>
#include <memory>
#include <string>

namespace rtask {
  namespace commons {
    namespace pddl_generator {

      class OrExpression : public LogicalExpression
      {
      public:
        ~OrExpression() override = default;

        OrExpression(const LogicalExprPtrVector& t_expr_vec = {});
        OrExpression(XmlRpc::XmlRpcValue& t_rpc_val);

        void clear();

        inline void set(const LogicalExprPtrVector& t_expr_vec = {}) { expr_vec_ = t_expr_vec; }

        LogicalExprPtr getExpression(int t_idx) const;
        int findExpression(LogicalExprPtr t_expr) const;
        bool hasExpression(LogicalExprPtr t_expr) const;
        bool addExpression(LogicalExprPtr t_expr);
        bool removeExpression(LogicalExprPtr t_expr);
        bool removeExpression(int t_idx);

        inline std::string getExpressionName() const { return expr_name_; }
        inline LogicalExprPtrVector getExpressions() const { return expr_vec_; }
        // bool validate(const UnordStrToLitTermMap& t_known_constants,
        //               const UnordStrToUIntMap& t_belonging_action_args,
        //               const std::string& t_belonging_action_name) const;

        std::string toPddl(bool t_typing = true, int t_pad_lv = 0) const override;

        OrExpression& operator=(const OrExpression& t_other);

      private:
        // string expression_name is already protected in parent class
        LogicalExprPtrVector expr_vec_{};
      };

      bool operator==(const OrExpression& t_first, const OrExpression& t_second);
      bool operator!=(const OrExpression& t_first, const OrExpression& t_second);
      std::ostream& operator<<(std::ostream& t_out, const OrExpression& t_expr);
      std::ostream& operator<<(std::ostream& t_out, std::shared_ptr<OrExpression> t_expr_ptr);

    } // namespace pddl_generator
  } // namespace commons
} // namespace rtask

#endif
