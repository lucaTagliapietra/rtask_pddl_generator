#ifndef rtask_commons_domain_h
#define rtask_commons_domain_h

#include "xmlrpcpp/XmlRpcValue.h"

#include "commons/action.h"
#include "commons/commons.h"

#include "rtask_msgs/Domain.h"

namespace rtask {
  namespace commons {

    class Domain
    {
    public:
      Domain() = default;
      Domain(const std::string& t_name,
             const std::vector<SemanticEntity>& t_semantic_entities = {},
             const std::vector<Predicate>& t_predicates = {},
             const std::vector<Action>& t_actions = {},
             const bool t_req_typing = false,
             const bool t_req_equality = false,
             const bool t_req_strips = false);
      Domain(const rtask_msgs::Domain& t_msg);
      Domain(const rtask_msgs::DomainConstPtr t_msg_ptr);
      Domain(XmlRpc::XmlRpcValue& t_node);

      ~Domain() = default;

      bool isValid() const;

      rtask_msgs::DomainPtr toDomainMsg() const;
      std::string toPddl() const;

      bool setDomainFromXmlRpc(XmlRpc::XmlRpcValue& t_node);
      bool setFromDomainMsg(const rtask_msgs::Domain& t_msg);
      bool setFromDomainMsg(const rtask_msgs::DomainConstPtr& t_msg_ptr);
      bool setDomain(const std::string& t_name,
                     const std::vector<SemanticEntity>& t_semantic_entities = {},
                     const std::vector<Predicate>& t_predicates = {},
                     const std::vector<Action>& t_actions = {},
                     const bool t_req_typing = false,
                     const bool t_req_equality = false,
                     const bool t_req_strips = false);

      void clear();

      std::string getName() const { return m_name; }
      std::vector<SemanticEntity> getSemanticEntities() const { return m_semantic_entities; }
      std::vector<Predicate> getPredicates() const { return m_predicates; }
      std::vector<Action> getActions() const { return m_actions; }

      // ---------------------
      // Semantic Entity Level
      // ---------------------
      bool hasSemanticEntity(const std::string& t_symbol, const std::string& t_type = "") const;
      bool getSemanticEntityType(const std::string& t_symbol, std::string& t_type) const;
      bool getSemanticEntityClass(const std::string& t_symbol, std::string& t_class) const;
      bool getSemanticEntity(const std::string& t_symbol, SemanticEntity& t_semantic_entity) const;
      bool addSemanticEntity(const std::string& t_symbol, const std::string& t_type, const std::string& t_class);
      bool removeSemanticEntity(const std::string& t_symbol, const std::string& t_type = "");

      // ---------------
      // Predicate Level
      // ---------------
      bool hasPredicate(const std::string& t_name, const std::vector<std::string>& t_args) const;
      bool hasPredicateName(const std::string& t_name) const;
      bool getPredicates(const std::string& t_name, std::vector<Predicate>& t_predicates) const;
      bool addPredicate(const std::string& t_name, const std::vector<std::string>& t_args);
      bool removePredicate(const std::string& t_name, const std::vector<std::string>& t_args);

      // ------------
      // Action Level
      // ------------
      bool hasAction(const Action& t_act) const;
      bool hasAction(const std::string& t_name,
                     const std::string& t_type,
                     const std::vector<Entity>& t_params,
                     const std::vector<Command>& t_preconditions,
                     const std::vector<Command>& t_effects) const;
      bool getActionType(const std::string& t_name, std::string& t_type) const;
      //      bool getActionParams(const std::string& t_name, std::vector<Entity>& t_params) const;
      //      bool getActionPreconditions(const std::string& t_name, std::vector<Command>& t_preconditions) const;
      //      bool getActionEffects(const std::string& t_name, std::vector<Command>& t_effects) const;
      //      bool getAction(const std::string& t_name, Action& t_action) const;
      bool addAction(const std::string& t_name,
                     const std::string& t_type,
                     const std::vector<Entity>& t_params,
                     const std::vector<Command>& t_preconditions,
                     const std::vector<Command>& t_effects);

      //      bool removeAction(const std::string& t_name);

    private:
      struct Requirements
      {
        bool typing = false;
        bool equality = false;
        bool strips = false;
      };

      std::string m_name = "";
      std::vector<SemanticEntity> m_semantic_entities = {};
      std::vector<Predicate> m_predicates = {};
      std::vector<Action> m_actions = {};
      Requirements m_requirements{};
    };

  } // namespace commons
} // namespace rtask
#endif
