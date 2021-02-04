#ifndef rtask_commons_pddl_generator_literal_expression_h
#define rtask_commons_pddl_generator_literal_expression_h

#include "LiteralTerm.h"
#include "LogicalExpression.h"
#include "xmlrpcpp/XmlRpc.h"

#include <iostream>
#include <memory>

namespace rtask {
  namespace commons {
    namespace pddl_generator {

      class LiteralExpression : public LogicalExpression
      {
      public:
        LiteralExpression();
        ~LiteralExpression() override = default;

        LiteralExpression(const std::string& t_name, const std::vector<std::string>& t_args = {});
        LiteralExpression(XmlRpc::XmlRpcValue& t_rpc_val);

        void clear();
        bool set(const std::string& t_name, const std::vector<std::string>& t_args = {});

        bool setExpressionName(const std::string& t_expr_name);
        bool setExpressionArgs(const std::vector<std::string>& t_args);

        inline std::string getExpressionName() const { return expr_name_; }
        inline std::vector<std::string> getExpressionArgs() const { return args_; }

        bool validate(const UnordStrToLitTermMap& t_known_constants,
                      const UnordStrToUIntMap& t_belonging_action_args,
                      const std::string& t_belonging_action_name) const;

        LiteralExpression& operator=(const LiteralExpression& t_other);

        std::string toPddl(bool t_typing = true, int t_pad_lv = 0) const override;

      private:
        //   string expression_name is already protected in parent class
        std::vector<std::string> args_{};
      };

      bool operator==(const LiteralExpression& t_first, const LiteralExpression& t_second);
      std::ostream& operator<<(std::ostream& t_out, const LiteralExpression& t_expr);
      std::ostream& operator<<(std::ostream& t_out, std::shared_ptr<LiteralExpression> t_expr_ptr);

    } // namespace pddl_generator
  } // namespace commons
} // namespace rtask

#endif
