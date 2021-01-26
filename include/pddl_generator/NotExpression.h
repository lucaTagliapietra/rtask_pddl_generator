#ifndef rtask_commons_pddl_generator_not_expression_h
#define rtask_commons_pddl_generator_not_expression_h

#include "BooleanExpression.h"
#include "Helpers.h"
#include "xmlrpcpp/XmlRpc.h"

#include <iostream>
#include <memory>
#include <string>

namespace rtask {
  namespace commons {
    namespace pddl_generator {

      class NotExpression : public BooleanExpression
      {
      public:
        NotExpression();
        ~NotExpression() override = default;

        NotExpression(const BooleanExpression& t_expr);
        NotExpression(std::shared_ptr<BooleanExpression> t_expr_ptr);
        NotExpression(XmlRpc::XmlRpcValue& t_rpc_val);

        void clear();

        inline void set(std::shared_ptr<BooleanExpression> t_expr) { expr_ = t_expr; }
        inline void setExpression(std::shared_ptr<BooleanExpression> t_expr) { set(t_expr); }

        inline std::string getExpressionName() const { return expression_name_; }
        inline std::shared_ptr<BooleanExpression> getExpression() const { return expr_; }

        // bool validate(const UnordStrToLitTermMap& t_known_constants,
        //               const UnordStrToUIntMap& t_belonging_action_args,
        //               const std::string& t_belonging_action_name) const;

        std::string toPddl(const bool t_typing = true) const override;

        NotExpression& operator=(const NotExpression& t_other);

      private:
        // string expression_name is already protected in parent class
        std::shared_ptr<BooleanExpression> expr_ = nullptr;
      };

      bool operator==(const NotExpression& t_first, const NotExpression& t_second);

      static std::ostream& operator<<(std::ostream& t_out, const NotExpression& t_expr)
      {
        t_out << "Expression Name: " << t_expr.getExpressionName() << std::endl;
        t_out << t_expr.getExpression();
        return t_out;
      }

    } // namespace pddl_generator
  } // namespace commons
} // namespace rtask

#endif
