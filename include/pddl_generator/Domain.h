#ifndef rtask_commons_pddl_generator_domain_h
#define rtask_commons_pddl_generator_domain_h

#include "Action.h"
#include "LiteralExpression.h"
#include "LiteralTerm.h"
#include "Predicate.h"
#include "xmlrpcpp/XmlRpc.h"

#include <iostream>
#include <memory>
#include <unordered_map>

namespace rtask {
  namespace commons {
    namespace pddl_generator {

      class Domain
      {
      public:
        Domain() = default;
        ~Domain() = default;

        Domain(const std::string& t_name,
               const std::string& t_extends_domain_name = {},
               const std::vector<std::string>& t_requirements = {},
               const UmapStrStr& t_types = {},
               const std::vector<LiteralTerm>& t_constants = {},
               const std::vector<Predicate>& t_predicates = {},
               const std::vector<LiteralExpression>& t_timeless = {},
               const std::vector<Action>& t_actions = {});
        Domain(XmlRpc::XmlRpcValue& t_rpc_val);

        void clear();
        bool set(const std::string& t_name,
                 const std::string& t_extends_domain_name = {},
                 const std::vector<std::string>& t_requirements = {},
                 const UmapStrStr& t_types = {},
                 const std::vector<LiteralTerm>& t_constants = {},
                 const std::vector<Predicate>& t_predicates = {},
                 const std::vector<LiteralExpression>& t_timeless = {},
                 const std::vector<Action>& t_actions = {});

        bool setName(const std::string& t_name);
        bool setExtendedDomainName(const std::string& t_name);
        bool setRequirements(const std::vector<std::string>& t_requirements);
        bool setTypes(const UmapStrStr& t_types);
        bool setConstants(const std::vector<LiteralTerm>& t_constants);
        bool setPredicates(const std::vector<Predicate>& t_predicates);
        bool setTimeless(const std::vector<LiteralExpression>& t_timeless);
        bool setActions(const std::vector<Action>& t_actions);

        inline std::string getName() const { return name_; }
        inline std::string getExtendedDomainName() const { return extends_domain_name_; }
        inline std::vector<std::string> getRequirements() const { return requirements_; }
        inline UmapStrStr getTypes() const { return types_; }
        inline std::vector<LiteralTerm> getConstants() const { return constants_; }
        inline std::vector<Predicate> getPredicates() const { return predicates_; }
        inline std::vector<LiteralExpression> getTimeless() const { return timeless_; }
        inline std::vector<Action> getActions() const { return actions_; }

        Domain& operator=(const Domain& t_other);
        std::string toPddl(bool t_typing = true, int t_pad_lv = 0) const;

      private:
        std::string name_{};
        std::string extends_domain_name_{};
        std::vector<std::string> requirements_{};
        UmapStrStr types_{};
        std::vector<LiteralTerm> constants_{};
        std::vector<Predicate> predicates_{};
        std::vector<LiteralExpression> timeless_{};
        std::vector<Action> actions_{};

        const static inline std::map<std::string, bool> SupportedRequirements = {{"strips", true},
                                                                                 {"typing", true},
                                                                                 {"disjunctive-preconditions", true},
                                                                                 {"equality", true},
                                                                                 {"existential-preconditions", true},
                                                                                 {"universal-preconditions", true},
                                                                                 {"quantified-preconditions", true},
                                                                                 {"conditional-effects", true},
                                                                                 {"action-expansions", false},
                                                                                 {"foreach-expansions", false},
                                                                                 {"dag-expansions", false},
                                                                                 {"domain-axioms", true},
                                                                                 {"subgoal-through-axioms", true},
                                                                                 {"safety-constraints", false},
                                                                                 {"expression-evaluation", false},
                                                                                 {"fluents", false},
                                                                                 {"open-world", false},
                                                                                 {"true-negation", false},
                                                                                 {"adl", true},
                                                                                 {"ucpop", false}};
      };

      bool operator==(const Domain& t_first, const Domain& t_second){};
      bool operator!=(const Domain& t_first, const Domain& t_second){};
      std::ostream& operator<<(std::ostream& t_out, const Domain& t_act){};
      std::ostream& operator<<(std::ostream& t_out, std::shared_ptr<Domain> t_expr_ptr){};

    }; // namespace pddl_generator
  } // namespace commons
} // namespace rtask

#endif
