#ifndef rtask_commons_literal_condition_h
#define rtask_commons_literal_condition_h

#include "xmlrpcpp/XmlRpc.h"

#include <string>
#include <vector>

#include "commons/typedName.h"
#include "rtask_msgs/Condition.h"

namespace rtask {
  namespace commons {

    using StringVector = std::vector<std::string>;
    using UnorderedStringToUIntMap = std::unordered_map<std::string, unsigned int>;

    class LiteralCondition
    {
    public:
      LiteralCondition() = default;
      ~LiteralCondition() = default;

      LiteralCondition(const std::string& t_name, const StringVector& t_args = {}, const bool t_negated = false);
      LiteralCondition(const rtask_msgs::ConditionConstPtr t_msg_ptr);
      LiteralCondition(const rtask_msgs::Condition& t_msg);
      LiteralCondition(XmlRpc::XmlRpcValue& t_rpc_val);

      // ---------------
      // Condition Level
      // ---------------
      rtask_msgs::Condition toMsg() const;

      void clear();
      void set(const std::string& t_name, const StringVector& t_args = {}, const bool t_negated = false);

      inline std::string getName() const { return name_; }
      inline StringVector getArguments() const { return args_; }
      inline bool getNegated() const { return negated_; }
      inline void setName(const std::string& t_name) { name_ = std::move(t_name); }
      inline void setArguments(const StringVector& t_args) { args_ = std::move(t_args); }
      inline void setNegated(const bool t_negated) { negated_ = std::move(t_negated); }

      std::string toPddl(const bool t_typing = true) const;

      bool validate(const UnorderedTypedNameMap& t_known_constants,
                    const UnorderedStringToUIntMap& t_belonging_action_args,
                    const std::string& t_belonging_action_name) const;

      // ---------
      // Operators
      // ---------
      bool operator==(const LiteralCondition& t_other) const;
      LiteralCondition& operator=(const LiteralCondition& t_other);

    private:
      void fromMsg(const rtask_msgs::Condition& t_msg);

      std::string name_{};
      StringVector args_{};
      bool negated_{false};
    };

    static std::ostream& operator<<(std::ostream& out, const LiteralCondition& lc)
    {
      out << "name: " << lc.getName() << std::endl << "negated: " << lc.getNegated() << std::endl;
      unsigned int i = 0;
      for (const auto& a : lc.getArguments()) {
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
