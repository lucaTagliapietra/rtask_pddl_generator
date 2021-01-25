#ifndef rtask_commons_pddl_generator_literal_boolean_expression_h
#define rtask_commons_pddl_generator_literal_boolean_expression_h

#include "BooleanExpression.h"
#include "LiteralTerm.h"
#include "xmlrpcpp/XmlRpc.h"

#include <iostream>

namespace rtask {
  namespace commons {
    namespace pddl_generator {

      class LiteralBooleanExpression : public BooleanExpression
      {
      public:
        LiteralBooleanExpression();
        ~LiteralBooleanExpression() override = default;

        LiteralBooleanExpression(const std::string& t_name, const std::vector<std::string>& t_args = {});
        LiteralBooleanExpression(XmlRpc::XmlRpcValue& t_rpc_val);

        void clear();
        bool set(const std::string& t_name, const std::vector<std::string>& t_args = {});

        bool setExpressionName(const std::string& t_expr_name);
        bool setExpressionArgs(const std::vector<std::string>& t_args);

        inline std::string getExpressionName() const { return expression_name_; }
        inline std::vector<std::string> getExpressionArgs() const { return args_; }

        bool validate(const UnordStrToLitTermMap& t_known_constants,
                      const UnordStrToUIntMap& t_belonging_action_args,
                      const std::string& t_belonging_action_name) const;

        bool operator==(const LiteralBooleanExpression& t_other) const;
        LiteralBooleanExpression& operator=(const LiteralBooleanExpression& t_other);

        std::string toPddl(const bool t_typing = true) const override;

      private:
        //   string expression_name is already protected in parent class
        std::vector<std::string> args_{};
      };

      static std::ostream& operator<<(std::ostream& t_out, const LiteralBooleanExpression& t_expr)
      {
        t_out << "Expresison Name: " << t_expr.getExpressionName() << std::endl;
        unsigned int i = 0;
        for (const auto& a : t_expr.getExpressionArgs()) {
          t_out << std::endl
                << "\t"
                << " - args[" << i << "]: " << a;
          ++i;
        }
        return t_out;
      }

    } // namespace pddl_generator
  } // namespace commons
} // namespace rtask

#endif
