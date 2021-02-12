#ifndef rtask_commons_pddl_generator_action_h
#define rtask_commons_pddl_generator_action_h

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

      class Action
      {
      public:
        Action() = default;
        ~Action() = default;

        Action(const std::string& t_name,
               const LiteralTermVector& t_params = {},
               LogicalExprPtr t_precondition = nullptr,
               LogicalExprPtr t_effect = nullptr);
        Action(XmlRpc::XmlRpcValue& t_rpc_val);

        void clear();
        bool set(const std::string& t_name,
                 const LiteralTermVector& t_params = {},
                 LogicalExprPtr t_precondition = nullptr,
                 LogicalExprPtr t_effect = nullptr);

        bool setName(const std::string& t_name);
        void setParameters(const LiteralTermVector& t_params);
        inline void setPrecondition(LogicalExprPtr t_precondition) { precondition_ = t_precondition; }
        inline void setEffect(LogicalExprPtr t_effect) { effect_ = t_effect; }

        inline std::string getName() const { return name_; }
        inline int getNumParameters() const { return static_cast<int>(params_.size()); }
        inline LiteralTermVector getParameters() const { return params_; }
        inline LogicalExprPtr getPrecondition() const { return precondition_; }
        inline LogicalExprPtr getEffect() const { return effect_; }

        bool isValid(const UmapStrStr& t_known_types,
                     const std::vector<LiteralTerm>& t_known_constants,
                     const std::vector<Predicate>& t_known_predicates,
                     const std::vector<LiteralExpression>& t_known_timeless) const;
        bool isEquivalentTo(const Action& t_other) const;

        Action& operator=(const Action& t_other);
        std::string toPddl(bool t_typing = true, int t_pad_lv = 0) const;

      private:
        std::string name_{};
        LiteralTermVector params_{};
        UmapStrStr params_map_{};
        LogicalExprPtr precondition_ = nullptr;
        LogicalExprPtr effect_ = nullptr;
      };

      bool operator==(const Action& t_first, const Action& t_second);
      bool operator!=(const Action& t_first, const Action& t_second);
      std::ostream& operator<<(std::ostream& t_out, const Action& t_act);
      std::ostream& operator<<(std::ostream& t_out, std::shared_ptr<Action> t_expr_ptr);

    }; // namespace pddl_generator
  } // namespace commons
} // namespace rtask

#endif
