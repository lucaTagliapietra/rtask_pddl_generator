#ifndef rtask_commons_action_h
#define rtask_commons_action_h

#include <map>

#include "xmlrpcpp/XmlRpcValue.h"

#include "commons/commons.h"

#include "rtask_msgs/Action.h"

namespace rtask {
  namespace commons {

    class Action
    {
    public:
      Action() {}
      Action(const std::string t_name,
             const std::vector<Parameter>& t_params = {},
             const std::vector<Command>& t_preconditions = {},
             const std::vector<Command>& t_effects = {});
      Action(const rtask_msgs::Action& t_msg);
      Action(const rtask_msgs::ActionConstPtr t_msg_ptr);
      Action(XmlRpc::XmlRpcValue& t_node, bool t_typing = false, bool t_equality = false, bool t_strips = false);

      ~Action() {}

      bool isValid() const;

      rtask_msgs::ActionPtr toActionMsg() const;
      std::string toPddl(bool t_typing = false, bool t_equality = false, bool t_strips = false) const;

      bool setActionFromXmlRpc(XmlRpc::XmlRpcValue& t_node,
                               bool t_typing = false,
                               bool t_equality = false,
                               bool t_strips = false);
      bool setFromActionMsg(const rtask_msgs::Action& t_msg);
      bool setFromActionMsg(const rtask_msgs::ActionConstPtr& t_msg_ptr);
      bool setAction(const std::string t_name,
                     const std::vector<Parameter>& t_params = {},
                     const std::vector<Command>& t_preconditions = {},
                     const std::vector<Command>& t_effects = {});
      void clear();

      std::string getName() const { return m_name; }
      std::vector<Parameter> getParameters() const { return m_params; }
      std::vector<Command> getPreconditions() const { return m_preconditions; }
      std::vector<Command> getEffects() const { return m_effects; }

      // ---------------
      // Parameter Level
      // ---------------
      bool hasParameter(const std::string& t_name, const std::string& t_type = "") const;
      bool getParameterType(const std::string& t_name, std::string& t_type) const;
      bool addParameter(const std::string& t_name, const std::string& t_type);
      bool removeParameter(const std::string& t_name, const std::string& t_type = "");

      // ------------------
      // Precondition Level
      // ------------------
      bool hasPrecondition(const Command& t_cmd) const { return hasCommand(t_cmd, PRECONDITION); }
      bool addPrecondition(const Command& t_cmd) { return addCommand(t_cmd, PRECONDITION); }
      bool removePrecondition(const Command& t_cmd) { return removeCommand(t_cmd, PRECONDITION); }

      // ------------
      // Effect Level
      // ------------
      bool hasEffect(const Command& t_cmd) const { return hasCommand(t_cmd, EFFECT); }
      bool addEffect(const Command& t_cmd) { return addCommand(t_cmd, EFFECT); }
      bool removeEffect(const Command& t_cmd) { return removeCommand(t_cmd, EFFECT); }

    private:
      bool hasCommand(const Command& t_cmd, unsigned int t_on) const;
      bool addCommand(const Command& t_cmd, unsigned int t_on);
      bool removeCommand(const Command& t_cmd, unsigned int t_on);

      std::string m_name = "";
      std::vector<Parameter> m_params = {};
      std::vector<Command> m_preconditions = {};
      std::vector<Command> m_effects = {};

      const unsigned int PRECONDITION = 0;
      const unsigned int EFFECT = 1;
    };
  } // namespace commons
} // namespace rtask

#endif
