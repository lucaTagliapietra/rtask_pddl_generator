#ifndef rtask_commons_condition_h
#define rtask_commons_condition_h

#include "xmlrpcpp/XmlRpc.h"

#include <string>
#include <vector>

#include "rtask_msgs/Condition.h"

namespace rtask {
  namespace commons {

    class Condition
    {
    public:
      Condition() = default;
      ~Condition() = default;

      Condition(const std::string& t_name, const std::vector<std::string>& t_args = {}, const bool t_negated = false);
      Condition(const rtask_msgs::ConditionConstPtr t_msg_ptr);
      Condition(const rtask_msgs::Condition& t_msg);
      Condition(XmlRpc::XmlRpcValue& t_rpc_val);

      // ---------------
      // Condition Level
      // ---------------
      rtask_msgs::Condition toMsg() const;

      void clear();
      void set(const std::string& t_name, const std::vector<std::string>& t_args = {}, const bool t_negated = false);

      inline std::string getName() const { return name_; }
      inline bool getNegated() const { return negated_; }
      inline std::vector<std::string> getArgs() const { return args_; }
      inline bool isValid() const { return valid_; }

      std::string toPddl(const bool t_typing = true) const;

      bool isEqual(const Condition& t_other, const bool t_typing = true) const;
      bool isEquivalent(const Condition& t_other, const bool t_typing = true) const;

      // ---------
      // Operators
      // ---------
      // bool operator==(const Condition& t_condition) const;
      Condition& operator=(const Condition& t_condition);

    private:
      void fromMsg(const rtask_msgs::Condition& t_msg);
      void updValidity();

      std::string name_{};
      std::vector<std::string> args_{};
      bool negated_{false};
      bool valid_{false};
    };

    static std::ostream& operator<<(std::ostream& out, const Condition& c)
    {
      out << "name: " << c.getName() << std::endl
          << "negated: " << c.getNegated() << std::endl
          << "valid: " << c.isValid();
      unsigned int i = 0;
      for (const auto& a : c.getArgs()) {
        out << std::endl
            << "\t"
            << " - args[" << i << "]: " << a;
        ++i;
      }
      return out << std::endl;
    }

  } // namespace commons
} // namespace rtask

#endif
