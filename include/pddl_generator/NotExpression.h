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

        bool isValid(UmapStrStr t_action_params,
                     const UmapStrStr& t_known_types,
                     const std::vector<LiteralTerm>& t_known_constants,
                     const std::vector<Predicate>& t_known_predicates,
                     const std::vector<LiteralExpression>& t_known_timeless,
                     const bool t_is_an_effect = false) const;

        std::string toPddl(bool t_typing = true, int t_pad_lv = 0) const override;

        NotExpression& operator=(const NotExpression& t_other);

      private:
        // string expression_name is already protected in parent class
        LogicalExprPtr expr_ = nullptr;
      };

      bool operator==(const NotExpression& t_first, const NotExpression& t_second);
      bool operator!=(const NotExpression& t_first, const NotExpression& t_second);
      std::ostream& operator<<(std::ostream& t_out, const NotExpression& t_expr);
      std::ostream& operator<<(std::ostream& t_out, std::shared_ptr<NotExpression> t_expr_ptr);

    } // namespace pddl_generator
  } // namespace commons
} // namespace rtask

#endif
