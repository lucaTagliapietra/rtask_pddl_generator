#ifndef rtask_commons_predicate_h
#define rtask_commons_predicate_h

#include "xmlrpcpp/XmlRpc.h"

#include <string>
#include <vector>

#include "commons/typedName.h"

#include "rtask_msgs/Predicate.h"

namespace rtask {
  namespace commons {

    class Predicate
    {
    public:
      Predicate() = default;
      ~Predicate() = default;

      Predicate(const std::string& t_name, const std::vector<TypedName>& t_args = {});
      Predicate(const rtask_msgs::PredicateConstPtr t_msg_ptr);
      Predicate(const rtask_msgs::Predicate& t_msg);
      Predicate(XmlRpc::XmlRpcValue& t_rpc_val);

      // ---------------
      // Predicate Level
      // ---------------
      rtask_msgs::Predicate toMsg() const;

      void clear();
      void set(const std::string& t_name, const std::vector<TypedName>& t_args = {});

      inline std::string getName() const { return name_; }
      inline std::vector<TypedName> getArguments() const { return args_; }
      inline void setName(const std::string& t_name) { name_ = std::move(t_name); }
      inline void setArguments(const std::vector<TypedName>& t_args) { args_ = std::move(t_args); }

      std::vector<std::string> getArgumentNames() const;
      std::vector<std::string> getArgumentTypeNames() const;

      std::string toPddl(const bool t_typing = true) const;

      bool validate(const UnorderedTypedNameMap& t_known_types) const;

      // ---------
      // Operators
      // ---------
      bool operator==(const Predicate& t_other) const;
      Predicate& operator=(const Predicate& t_other);

    private:
      void fromMsg(const rtask_msgs::Predicate& t_msg);

      std::string name_{};
      std::vector<TypedName> args_{};
    };

    static std::ostream& operator<<(std::ostream& out, const Predicate& p)
    {
      out << "name: " << p.getName() << std::endl;
      unsigned int i = 0;
      for (const auto& arg : p.getArguments()) {
        out << "\t"
            << " - args[" << i << "]: " << arg;
        ++i;
      }
      return out << std::endl;
    }

  } // namespace commons
} // namespace rtask

#endif
