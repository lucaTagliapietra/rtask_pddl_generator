#ifndef rtask_commons_pddl_generator_and_expression_h
#define rtask_commons_pddl_generator_and_expression_h

#include "Helpers.h"
#include "LogicalExpression.h"
#include "xmlrpcpp/XmlRpc.h"

#include <iostream>
#include <memory>
#include <string>

namespace rtask {
  namespace commons {
    namespace pddl_generator {

      using LogicalExprPtrVector = std::vector<std::shared_ptr<LogicalExpression>>;

      class AndExpression : public LogicalExpression
      {
      public:
        AndExpression();
        ~AndExpression() override = default;

        AndExpression(const LogicalExprPtrVector& t_expr_vec = {});
        AndExpression(XmlRpc::XmlRpcValue& t_rpc_val);

        void clear();

        inline void set(const LogicalExprPtrVector& t_expr_vec = {}) { expr_vec_ = t_expr_vec; }

        int findExpression(std::shared_ptr<LogicalExpression> t_expr) const;
        bool hasExpression(std::shared_ptr<LogicalExpression> t_expr) const;
        bool addExpression(std::shared_ptr<LogicalExpression> t_expr);
        bool removeExpression(std::shared_ptr<LogicalExpression> t_expr);

        inline std::string getExpressionName() const { return expr_name_; }
        inline LogicalExprPtrVector getExpressions() const { return expr_vec_; }
        // bool validate(const UnordStrToLitTermMap& t_known_constants,
        //               const UnordStrToUIntMap& t_belonging_action_args,
        //               const std::string& t_belonging_action_name) const;

        std::string toPddl(const bool t_typing = true) const override;

        AndExpression& operator=(const AndExpression& t_other);

      private:
        // string expression_name is already protected in parent class
        LogicalExprPtrVector expr_vec_{};
      };

      bool operator==(const AndExpression& t_first, const AndExpression& t_second);

      static std::ostream& operator<<(std::ostream& t_out, const AndExpression& t_expr)
      {
        t_out << "AndExpression name: " << t_expr.getExpressionName() << std::endl;
        unsigned int i = 0;
        std::cout << t_expr.getExpressions().size() << std::endl;
        for (const auto& expr : t_expr.getExpressions()) {
          (i != 0) ? t_out << std::endl : t_out << "";
          t_out << "\t"
                << " - expr[" << i << "]: " << expr;
          ++i;
        }
        return t_out;
      }

    } // namespace pddl_generator
  } // namespace commons
} // namespace rtask

#endif
