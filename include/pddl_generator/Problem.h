#ifndef rtask_commons_pddl_generator_problem_h
#define rtask_commons_pddl_generator_problem_h

#include "Domain.h"
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

      using LiteralExprVector = std::vector<LiteralExpression>;
      using DomainPtr = std::shared_ptr<Domain>;

      class Problem
      {
      public:
        Problem() = default;
        ~Problem() = default;

        Problem(const std::string& t_name,
                const std::string& t_belonging_domain_name = {},
                const LiteralTermVector& t_objects = {},
                const LiteralExprVector& t_initial_state = {},
                const LogicalExprPtr t_goal_ptr = nullptr);
        Problem(XmlRpc::XmlRpcValue& t_rpc_val);

        void clear();
        bool set(const std::string& t_name,
                 const std::string& t_belonging_domain_name = {},
                 const LiteralTermVector& t_objects = {},
                 const LiteralExprVector& t_initial_state = {},
                 const LogicalExprPtr t_goal_ptr = nullptr);

        bool setName(const std::string& t_name);
        bool setBelongingDomainName(const std::string& t_name);
        bool setObjects(const LiteralTermVector& t_objs);
        bool setInitialState(const LiteralExprVector& t_initial_state);
        bool setGoal(LogicalExprPtr t_ptr);

        bool setBelongingDomain(DomainPtr t_ptr);
        inline DomainPtr getBelongingDomain() const { return belonging_domain_; }

        bool hasObject(const LiteralTerm& t_obj) const;
        bool hasInitialState(const LiteralExpression& t_state) const;

        bool hasValidUniqueObjects() const;
        bool hasValidUniqueInitialStates() const;

        bool addObject(const LiteralTerm& t_obj);
        bool addInitialState(const LiteralExpression& t_state);

        bool hasValidGoal() const;
        bool hasValidBelongingDomain() const;

        inline std::string getName() const { return name_; }
        inline std::string getBelongingDomainName() const { return belonging_domain_name_; }
        inline LiteralTermVector getObjects() const { return objects_; }
        inline LiteralExprVector getInitialState() const { return initial_state_; }
        inline LogicalExprPtr getGoal() const { return goal_; }

        bool isValid() const;
        bool isEquivalentTo(const Problem& t_other) const;

        Problem& operator=(const Problem& t_other);
        std::string toPddl(bool t_typing = true, int t_pad_lv = 0) const;

      private:
        std::string name_{};
        std::string belonging_domain_name_{};
        LiteralTermVector objects_{};
        UmapStrStr objects_map_{};
        LiteralExprVector initial_state_{};
        LogicalExprPtr goal_ = nullptr;

        DomainPtr belonging_domain_ = nullptr;

        bool checkNameValidity(const std::string& t_name) const;
        bool checkBelongingDomainNameValidity(const std::string& t_name) const;

        bool isObjectValid(const LiteralTerm& t_obj) const;
        bool hasValidUniqueObjects(const LiteralTermVector& t_objs) const;

        bool isInitialStateValid(const LiteralExpression& t_state) const;
        bool hasValidUniqueInitialStates(const LiteralExprVector& t_states) const;
      };

      bool operator==(const Problem& t_first, const Problem& t_second);
      bool operator!=(const Problem& t_first, const Problem& t_second);
      std::ostream& operator<<(std::ostream& t_out, const Problem& t_prob);
      std::ostream& operator<<(std::ostream& t_out, std::shared_ptr<Problem> t_prob_ptr);

    }; // namespace pddl_generator
  } // namespace commons
} // namespace rtask

#endif
