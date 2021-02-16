#ifndef rtask_commons_pddl_generator_axiom_h
#define rtask_commons_pddl_generator_axiom_h

#include "LiteralExpression.h"
#include "LiteralTerm.h"
#include "LogicalExpression.h"
#include "Predicate.h"

#include "xmlrpcpp/XmlRpc.h"

#include <iostream>
#include <memory>

namespace rtask {
  namespace commons {
    namespace pddl_generator {

      class Axiom
      {
      public:
        ~Axiom() = default;

        Axiom(const LiteralTermVector& t_vars = {},
              LogicalExprPtr t_context = nullptr,
              LogicalExprPtr t_implies = nullptr);
        Axiom(XmlRpc::XmlRpcValue& t_rpc_val);

        void clear();
        bool set(const LiteralTermVector& t_vars = {},
                 LogicalExprPtr t_context = nullptr,
                 LogicalExprPtr t_implies = nullptr);

        bool setVariables(const LiteralTermVector& t_vars);
        bool setContext(LogicalExprPtr t_context);
        bool setImplies(LogicalExprPtr t_implies);

        inline int getNumVars() const { return static_cast<int>(vars_.size()); }
        inline LiteralTermVector getVariables() const { return vars_; }
        inline LogicalExprPtr getContext() const { return context_; }
        inline LogicalExprPtr getImplies() const { return implies_; }

        bool isValid(const UmapStrStr& t_known_types,
                     const std::vector<LiteralTerm>& t_known_constants,
                     const std::vector<Predicate>& t_known_predicates,
                     const std::vector<LiteralExpression>& t_known_timeless) const;
        bool isEquivalentTo(const Axiom& t_other) const;

        Axiom& operator=(const Axiom& t_other);
        std::string toPddl(bool t_typing = true, int t_pad_lv = 0) const;

      private:
        LiteralTermVector vars_{};
        UmapStrStr vars_map_{};
        LogicalExprPtr context_ = nullptr;
        LogicalExprPtr implies_ = nullptr;
      };

      bool operator==(const Axiom& t_first, const Axiom& t_second);
      bool operator!=(const Axiom& t_first, const Axiom& t_second);
      std::ostream& operator<<(std::ostream& t_out, const Axiom& t_act);
      std::ostream& operator<<(std::ostream& t_out, std::shared_ptr<Axiom> t_expr_ptr);

    }; // namespace pddl_generator
  } // namespace commons
} // namespace rtask

#endif
