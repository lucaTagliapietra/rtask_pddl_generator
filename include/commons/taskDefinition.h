#ifndef rtask_commons_task_definition_h
#define rtask_commons_task_definition_h

#include "xmlrpcpp/XmlRpcValue.h"

#include "commons/commons.h"

#include "rtask_msgs/TaskDefinition.h"

namespace rtask {
  namespace commons {

    class TaskDefinition
    {
    public:
      TaskDefinition() = default;
      TaskDefinition(const std::string& t_name,
                     const std::string& t_domain_name,
                     const std::vector<Entity>& t_entities = {},
                     const std::vector<Command>& t_initial_state = {},
                     const std::vector<Command>& t_goal_state = {},
                     const bool t_req_typing = false,
                     const bool t_req_equality = false,
                     const bool t_req_strips = false);
      TaskDefinition(const rtask_msgs::TaskDefinition& t_msg);
      TaskDefinition(const rtask_msgs::TaskDefinitionConstPtr t_msg_ptr);
      TaskDefinition(XmlRpc::XmlRpcValue& t_node);

      ~TaskDefinition() = default;

      bool isValid() const;

      rtask_msgs::TaskDefinitionPtr toTaskDefinitionMsg() const;
      std::string toPddl() const;

      bool setTaskDefinitionFromXmlRpc(XmlRpc::XmlRpcValue& t_node);
      bool setFromTaskDefinitionMsg(const rtask_msgs::TaskDefinition& t_msg);
      bool setFromTaskDefinitionMsg(const rtask_msgs::TaskDefinitionConstPtr& t_msg_ptr);
      bool setTaskDefinition(const std::string& t_name,
                             const std::string& t_domain_name,
                             const std::vector<Entity>& t_entities = {},
                             const std::vector<Command>& t_initial_state = {},
                             const std::vector<Command>& t_goal_state = {},
                             const bool t_req_typing = false,
                             const bool t_req_equality = false,
                             const bool t_req_strips = false);
      void clear();

      std::string getName() const { return m_name; }
      std::string getDomainName() const { return m_domain_name; }
      std::vector<Entity> getEntities() const { return m_entities; }
      std::vector<Command> getInitialState() const { return m_initial_state; }
      std::vector<Command> getGoalState() const { return m_goal_state; }
      bool getTypingRequirement() const { return m_requirements.typing; }
      bool getStripsRequirement() const { return m_requirements.strips; }
      bool getEqualityRequirement() const { return m_requirements.equality; }

      // ------------
      // Entity Level
      // ------------
      bool hasEntity(const std::string& t_symbol, const std::string& t_type = "") const;
      bool getEntityType(const std::string& t_symbol, std::string& t_type) const;
      bool getEntityClass(const std::string& t_symbol, std::string& t_class) const;
      bool getEntityProperties(const std::string& t_symbol, std::vector<std::string>& t_properties) const;
      bool getEntity(const std::string& t_symbol, Entity& t_entity) const;
      bool addEntity(const std::string& t_symbol,
                     const std::string& t_type,
                     const std::string& t_class,
                     const std::vector<std::string>& t_properties);
      bool removeEntity(const std::string& t_symbol, const std::string& t_type = "");

      // -------------------
      // Initial State Level
      // -------------------
      bool hasInitialState(const Command& t_cmd) const { return hasState(t_cmd, INITIAL_STATE); }
      bool addInitialState(const Command& t_cmd) { return addState(t_cmd, INITIAL_STATE); }
      bool removeInitialState(const Command& t_cmd) { return removeState(t_cmd, INITIAL_STATE); }

      // ----------------
      // Goal State Level
      // ----------------
      bool hasGoalState(const Command& t_cmd) const { return hasState(t_cmd, GOAL_STATE); }
      bool addGoalState(const Command& t_cmd) { return addState(t_cmd, GOAL_STATE); }
      bool removeGoalState(const Command& t_cmd) { return removeState(t_cmd, GOAL_STATE); }

    private:
      bool hasState(const Command& t_cmd, unsigned int t_on) const;
      bool addState(const Command& t_cmd, unsigned int t_on);
      bool removeState(const Command& t_cmd, unsigned int t_on);

      struct Requirements
      {
        bool typing = false;
        bool equality = false;
        bool strips = false;
      };

      std::string m_name = "";
      std::string m_domain_name = "";
      std::vector<Entity> m_entities = {};
      std::vector<Command> m_initial_state = {};
      std::vector<Command> m_goal_state = {};
      Requirements m_requirements{};

      const unsigned int INITIAL_STATE = 0;
      const unsigned int GOAL_STATE = 1;
    };

  } // namespace commons
} // namespace rtask

#endif
