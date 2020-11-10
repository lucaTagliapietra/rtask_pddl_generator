#ifndef rtask_commons_agent_h
#define rtask_commons_agent_h

#include <string>
#include <vector>

#include "xmlrpcpp/XmlRpcValue.h"

#include "rtask_msgs/Agent.h"

namespace rtask {
  namespace commons {

    enum AgentStatus
    {
      UNKNOWN = 0,
      READY = 1,
      BUSY = 2,
      DISCONNECTED = 3
    };

    class Agent
    {
    public:
      Agent() = default;
      ~Agent() = default;

      Agent(const std::string& t_name, const std::string& t_description = {}, const AgentStatus& t_status = {});
      Agent(const rtask_msgs::Agent& t_msg);
      Agent(const rtask_msgs::AgentConstPtr t_msg_ptr);
      Agent(XmlRpc::XmlRpcValue& t_rpc_val);

      // -----------
      // Agent Level
      // -----------
      void clear();
      void set(const std::string& t_name, const std::string& t_description = {}, const AgentStatus& t_status = {});

      rtask_msgs::Agent toMsg() const;

      inline std::string getName() const { return name_; }
      inline std::string getDescription() const { return description_; }
      inline AgentStatus getStatus() const { return status_; }

      inline bool isValid() const { return valid_; }
      inline bool isReady() const { return status_ == AgentStatus::READY; }
      inline bool isBusy() const { return status_ == AgentStatus::BUSY; }
      inline bool isConnected() const { return status_ == AgentStatus::READY || status_ == AgentStatus::BUSY; }

      inline void setDescription(std::string& t_description) { description_ = t_description; }
      inline void setStatus(AgentStatus t_status) { status_ = t_status; }

      // ---------
      // Operators
      // ---------
      bool operator==(const Agent& t_agent) const;
      Agent& operator=(const Agent& t_agent);

    private:
      bool valid_{false};
      std::string name_{};
      std::string description_{};
      AgentStatus status_{AgentStatus::UNKNOWN};

      void updValidity();
      void fromMsg(const rtask_msgs::Agent& t_msg);
    };

    static std::ostream& operator<<(std::ostream& out, const Agent& ag)
    {
      out << "Agent name: " << ag.getName() << std::endl;
      out << " - Valid: " << ag.isValid() << std::endl;
      out << " - Description: " << ag.getDescription() << std::endl;
      out << " - Status: " << ag.getStatus();
      return out << std::endl;
    }

    static std::ostream& operator<<(std::ostream& out, const AgentStatus& as)
    {
      switch (as) {
        case AgentStatus::UNKNOWN: {
          out << "Unknown";
          break;
        };
        case AgentStatus::READY: {
          out << "Ready";
          break;
        };
        case AgentStatus::BUSY: {
          out << "Busy";
          break;
        };
        case AgentStatus::DISCONNECTED: {
          out << "Disconnected";
          break;
        };
      }
      return out << std::endl;
    }
  } // namespace commons
} // namespace rtask

#endif // rtask_commons_agent_h
