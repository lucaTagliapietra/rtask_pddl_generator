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

      class AndExpression : public LogicalExpression
      {
      public:
        ~AndExpression() override = default;

        AndExpression(const LogicalExprPtrVector& t_expr_vec = {});
        AndExpression(XmlRpc::XmlRpcValue& t_rpc_val);

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

        bool isValid(UmapStrStr t_action_params,
                     const UmapStrStr& t_known_types,
                     const std::vector<LiteralTerm>& t_known_constants,
                     const std::vector<Predicate>& t_known_predicates,
                     const std::vector<LiteralExpression>& t_known_timeless,
                     const bool t_is_an_effect = false) const;

        std::string toPddl(bool t_typing = true, int t_pad_lv = 0) const override;

        AndExpression& operator=(const AndExpression& t_other);

      private:
        // string expression_name is already protected in parent class
        LogicalExprPtrVector expr_vec_{};
      };

      bool operator==(const AndExpression& t_first, const AndExpression& t_second);
      bool operator!=(const AndExpression& t_first, const AndExpression& t_second);
      std::ostream& operator<<(std::ostream& t_out, const AndExpression& t_expr);
      std::ostream& operator<<(std::ostream& t_out, std::shared_ptr<AndExpression> t_expr_ptr);

    } // namespace pddl_generator
  } // namespace commons
} // namespace rtask

#endif
