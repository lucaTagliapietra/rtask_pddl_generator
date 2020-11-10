#ifndef rtask_commons_agent_group_h
#define rtask_commons_agent_group_h

#include <string>
#include <vector>

#include "xmlrpcpp/XmlRpcValue.h"

#include "rtask_msgs/AgentGroup.h"

#include "commons/agent.h"

namespace rtask {
  namespace commons {
    class AgentGroup
    {
    public:
      AgentGroup() = default;
      ~AgentGroup() = default;

      AgentGroup(const std::string& t_name,
                 const std::string& t_description = {},
                 const std::vector<Agent> t_agents = {});
      AgentGroup(const rtask_msgs::AgentGroup& t_msg);
      AgentGroup(const rtask_msgs::AgentGroupConstPtr t_msg_ptr);
      AgentGroup(XmlRpc::XmlRpcValue& t_rpc_val);

      // -----------------
      // Agent Group Level
      // -----------------

      rtask_msgs::AgentGroup toMsg() const;

      void clear();
      void set(const std::string& t_name, const std::string& t_description, const std::vector<Agent>& t_agents);

      inline bool isValid() const { return valid_; }

      inline std::string getName() const { return name_; }
      inline std::string getDescription() const { return description_; }
      inline std::vector<Agent> getAgents() const { return agents_; }

      std::vector<std::string> getAgentList() const;

      // -----------
      // Agent Level
      // -----------

      bool hasAgent(const std::string& t_name) const;
      bool deleteAgent(const std::string& t_name);
      bool isAgentValid(const std::string& t_name) const;
      std::pair<bool, Agent> getAgent(const std::string& t_name) const;
      void setAgent(const std::string& t_name, const std::string& t_descr, const commons::AgentStatus& t_status);
      void setAgent(const std::string& t_name, const Agent& t_val);

      // ---------
      // Operators
      // ---------
      bool operator==(const AgentGroup& t_agent_group) const;
      AgentGroup& operator=(const AgentGroup& t_agent_group);

    private:
      bool valid_{false};
      std::string name_{};
      std::string description_{};
      std::vector<Agent> agents_{};

      void updValidity();
      void fromMsg(const rtask_msgs::AgentGroup& t_msg);
    };

    static std::ostream& operator<<(std::ostream& out, const AgentGroup& ag)
    {
      out << "AgentGroup name: " << ag.getName() << std::endl;
      out << " - Valid: " << ag.isValid() << std::endl;
      out << " - Description: " << ag.getDescription() << std::endl;
      unsigned int i = 0;
      for (const auto& a : ag.getAgents()) {
        out << "\t"
            << " - a[" << i << "]: " << a;
        ++i;
      }
      return out << std::endl;
    }
  } // namespace commons
} // namespace rtask

#endif // rtask_commons_agent_group_h
