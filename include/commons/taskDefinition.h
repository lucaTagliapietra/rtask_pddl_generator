#ifndef rtask_commons_task_definition_h
#define rtask_commons_taak_definition_h

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
                     const std::vector<Parameter>& t_objects = {},
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
                             const std::vector<Parameter>& t_objects = {},
                             const std::vector<Command>& t_initial_state = {},
                             const std::vector<Command>& t_goal_state = {},
                             const bool t_req_typing = false,
                             const bool t_req_equality = false,
                             const bool t_req_strips = false);
      void clear();

      std::string getName() const { return m_name; }
      std::string getDomainName() const { return m_domain_name; }
      std::vector<Parameter> getObjects() const { return m_objects; }
      std::vector<Command> getPreconditions() const { return m_initial_state; }
      std::vector<Command> getEffects() const { return m_goal_state; }

      // ------------
      // Object Level
      // ------------
      bool hasObject(const std::string& t_name, const std::string& t_type = "") const;
      bool getObjectType(const std::string& t_name, std::string& t_type) const;
      bool addObject(const std::string& t_name, const std::string& t_type);
      bool removeObject(const std::string& t_name, const std::string& t_type = "");

      // -------------------
      // Initial State Level
      // -------------------
      bool hasInitialCondition(const Command& t_cmd) const { return hasCondition(t_cmd, INITIAL_STATE); }
      bool addInitialCondition(const Command& t_cmd) { return addCondition(t_cmd, INITIAL_STATE); }
      bool removeInitialCondition(const Command& t_cmd) { return removeCondition(t_cmd, INITIAL_STATE); }

      // ----------------
      // Goal State Level
      // ----------------
      bool hasGoalCondition(const Command& t_cmd) const { return hasCondition(t_cmd, GOAL_STATE); }
      bool addGoalCondition(const Command& t_cmd) { return addCondition(t_cmd, GOAL_STATE); }
      bool removeGoalCondition(const Command& t_cmd) { return removeCondition(t_cmd, GOAL_STATE); }

    private:
      bool hasCondition(const Command& t_cmd, unsigned int t_on) const;
      bool addCondition(const Command& t_cmd, unsigned int t_on);
      bool removeCondition(const Command& t_cmd, unsigned int t_on);

      struct Requirements
      {
        bool typing = false;
        bool equality = false;
        bool strips = false;
      };

      std::string m_name = "";
      std::string m_domain_name = "";
      std::vector<Parameter> m_objects = {};
      std::vector<Command> m_initial_state = {};
      std::vector<Command> m_goal_state = {};
      Requirements m_requirements{};

      const unsigned int INITIAL_STATE = 0;
      const unsigned int GOAL_STATE = 1;
    };

  } // namespace commons
} // namespace rtask

#endif
