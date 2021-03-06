#ifndef rtask_commons_pddl_generator_equals_expression_h
#define rtask_commons_pddl_generator_equals_expression_h

#include "Helpers.h"
#include "LogicalExpression.h"
#include "xmlrpcpp/XmlRpc.h"

#include <iostream>
#include <memory>
#include <string>

namespace rtask {
  namespace commons {
    namespace pddl_generator {

      class EqualsExpression : public LogicalExpression
      {
      public:
        EqualsExpression();
        ~EqualsExpression() override = default;

        EqualsExpression(const std::string& t_lhs, const std::string& t_rhs);
        EqualsExpression(XmlRpc::XmlRpcValue& t_rpc_val);

        void clear();

        void set(const std::string& t_lhs, const std::string& t_rhs);
        void setLhsTerm(const std::string& t_lhs);
        void setRhsTerm(const std::string& t_rhs);

        inline std::string getExpressionName() const { return expr_name_; }
        inline std::string getLhsTerm() const { return lhs_term_name_; }
        inline std::string getRhsTerm() const { return rhs_term_name_; }

        bool isValid(UmapStrStr t_action_params,
                     const UmapStrStr& t_known_types,
                     const std::vector<LiteralTerm>& t_known_constants,
                     const std::vector<Predicate>& t_known_predicates,
                     const std::vector<LiteralExpression>& t_known_timeless,
                     const bool t_is_an_effect = false) const;

        std::string toPddl(bool t_typing = true, int t_pad_lv = 0) const override;

        EqualsExpression& operator=(const EqualsExpression& t_other);

      private:
        // string expression_name is already protected in parent class
        std::string lhs_term_name_{};
        std::string rhs_term_name_{};
      };

      bool operator==(const EqualsExpression& t_first, const EqualsExpression& t_second);
      bool operator!=(const EqualsExpression& t_first, const EqualsExpression& t_second);
      std::ostream& operator<<(std::ostream& t_out, const EqualsExpression& t_expr);
      std::ostream& operator<<(std::ostream& t_out, std::shared_ptr<EqualsExpression> t_expr_ptr);

    } // namespace pddl_generator
  } // namespace commons
} // namespace rtask

#endif
