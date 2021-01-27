#ifndef rtask_commons_pddl_generator_not_expression_h
#define rtask_commons_pddl_generator_not_expression_h

#include "Helpers.h"
#include "LogicalExpression.h"
#include "xmlrpcpp/XmlRpc.h"

#include <iostream>
#include <memory>
#include <string>

namespace rtask {
  namespace commons {
    namespace pddl_generator {

      class NotExpression : public LogicalExpression
      {
      public:
        NotExpression();
        ~NotExpression() override = default;

        NotExpression(const LogicalExpression& t_expr);
        NotExpression(LogicalExprPtr t_expr_ptr);
        NotExpression(XmlRpc::XmlRpcValue& t_rpc_val);

        void clear();

        inline void set(LogicalExprPtr t_expr) { expr_ = t_expr; }
        inline void setExpression(LogicalExprPtr t_expr) { set(t_expr); }

        inline std::string getExpressionName() const { return expr_name_; }
        inline LogicalExprPtr getExpression() const { return expr_; }

        // bool validate(const UnordStrToLitTermMap& t_known_constants,
        //               const UnordStrToUIntMap& t_belonging_action_args,
        //               const std::string& t_belonging_action_name) const;

        std::string toPddl(const bool t_typing = true) const override;

        NotExpression& operator=(const NotExpression& t_other);

      private:
        // string expression_name is already protected in parent class
        LogicalExprPtr expr_ = nullptr;
      };

      bool operator==(const NotExpression& t_first, const NotExpression& t_second);

      static std::ostream& operator<<(std::ostream& t_out, const NotExpression& t_expr)
      {
        t_out << "NotExpression name: " << t_expr.getExpressionName() << std::endl;
        t_out << t_expr.getExpression();
        return t_out;
      }

    } // namespace pddl_generator
  } // namespace commons
} // namespace rtask

#endif
