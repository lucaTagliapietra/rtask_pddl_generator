#ifndef rtask_commons_typed_name_h
#define rtask_commons_typed_name_h

#include "xmlrpcpp/XmlRpc.h"
#include <string>

#include "rtask_msgs/Parameter.h"

namespace rtask {
  namespace commons {

    class TypedName;

    using UnorderedTypedNameMap = std::unordered_map<std::string, TypedName>;

    class TypedName
    {
    public:
      TypedName() = default;
      ~TypedName() = default;

      TypedName(const std::string& t_name, const std::string& t_type = {"object"});
      TypedName(const rtask_msgs::ParameterConstPtr t_msg_ptr);
      TypedName(const rtask_msgs::Parameter& t_msg);
      TypedName(XmlRpc::XmlRpcValue& t_rpc_val);

      rtask_msgs::Parameter toMsg() const;

      void set(const std::string& t_name, const std::string& t_type);
      inline void setName(const std::string& t_name) { name_ = std::move(t_name); }
      inline void setType(const std::string& t_type) { type_name_ = std::move(t_type); }

      inline std::string getName() const { return name_; }
      inline std::string getTypeName() const { return type_name_; }

      std::string toPddl(const bool t_typing = true) const;

      bool validate(const UnorderedTypedNameMap& t_types) const;

      bool operator==(const TypedName& t_other) const;
      TypedName& operator=(const TypedName& t_other);

    private:
      void fromMsg(const rtask_msgs::Parameter& t_msg);

      std::string name_{};
      std::string type_name_{"object"};
    };

    static std::ostream& operator<<(std::ostream& t_out, const TypedName& t_tn)
    {
      t_out << "name: " << t_tn.getName() << std::endl << "type: " << t_tn.getTypeName() << std::endl;
      return t_out;
    }

  } // namespace commons
} // namespace rtask

#endif
