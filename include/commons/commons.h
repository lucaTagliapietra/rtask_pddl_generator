#ifndef rtask_commons_commons_h
#define rtask_commons_commons_h

#include <string>
#include <vector>

#include "rtask_msgs/Command.h"
#include "rtask_msgs/Parameter.h"
#include "rtask_msgs/Predicate.h"

namespace rtask {
  namespace commons {

    const std::string NULL_TYPE = "null_type";

    struct Parameter
    {
      std::string name{};
      std::string type{};

      Parameter() = default;
      Parameter(const std::string& t_name, const std::string& t_type)
        : name(t_name)
        , type(t_type)
      {}
      Parameter(const rtask_msgs::Parameter& t_msg)
        : name(t_msg.name)
        , type(t_msg.type)
      {}

      bool operator==(const Parameter& t_other) const { return (name == t_other.name && type == t_other.type); }
      bool operator!=(const Parameter& t_other) { return !operator==(t_other); }

      rtask_msgs::Parameter toMsg() const
      {
        rtask_msgs::Parameter msg;
        msg.name = name;
        msg.type = type;
        return msg;
      }
    };

    struct Predicate
    {
      std::string cmd{};
      std::vector<Parameter> args{};

      Predicate() = default;
      Predicate(const std::string& t_name, const std::vector<Parameter>& t_args)
        : cmd(t_name)
        , args(t_args)
      {}

      Predicate(const rtask_msgs::Predicate& t_msg)
        : cmd(t_msg.cmd)
      {
        for (const auto& arg : t_msg.args) {
          args.push_back({arg});
        }
      }

      bool operator==(const Predicate& t_other) const
      {
        bool eq = (cmd == t_other.cmd) && (args.size() == t_other.args.size());
        if (eq) {
          for (unsigned int i = 0; i < args.size(); ++i) {
            eq &= args[i] == t_other.args[i];
          }
        }
        return eq;
      }

      bool operator!=(const Predicate& t_other) { return !operator==(t_other); }

      rtask_msgs::Predicate toMsg() const
      {
        rtask_msgs::Predicate msg;
        msg.cmd = cmd;
        for (const auto& arg : args) {
          msg.args.emplace_back(arg.toMsg());
        }
        return msg;
      }
    };

    struct Command
    {
      Predicate predicate{};
      bool negate = false;

      Command() = default;
      Command(const Predicate& t_pred, const bool t_negate)
        : predicate(t_pred)
        , negate(t_negate)
      {}

      Command(const rtask_msgs::Command& t_msg)
        : predicate(t_msg.predicate)
        , negate(t_msg.negate)
      {}

      bool operator==(const Command& t_other) const
      {
        return (predicate == t_other.predicate && (negate == t_other.negate));
      }
      bool operator!=(const Command& t_other) { return !operator==(t_other); }

      rtask_msgs::Command toMsg() const
      {
        rtask_msgs::Command msg;
        msg.predicate = predicate.toMsg();
        msg.negate = negate;
        return msg;
      }
    };

  } // namespace commons
} // namespace rtask

#endif
