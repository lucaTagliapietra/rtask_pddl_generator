#ifndef rtask_commons_parameter_h
#define rtask_commons_parameter_h

#include "xmlrpcpp/XmlRpc.h"
#include <string>

#include "rtask_msgs/Parameter.h"

namespace rtask {
  namespace commons {

    class Parameter
    {
    public:
      Parameter() = default;
      ~Parameter() = default;

      Parameter(const std::string& t_name, const std::string& t_type = {});
      Parameter(const rtask_msgs::ParameterConstPtr t_msg_ptr);
      Parameter(const rtask_msgs::Parameter& t_msg);
      Parameter(XmlRpc::XmlRpcValue& t_rpc_val);

      rtask_msgs::Parameter toMsg() const;

      void set(const std::string& t_name, const std::string& t_type);
      void setType(const std::string& t_type);

      inline std::string getName() const { return name_; }
      inline std::string getType() const { return type_; }

      inline bool isValid() const { return valid_; }

      std::string toPddl(const bool t_typing = true) const;

      bool operator==(const Parameter& t_parameter) const;
      Parameter& operator=(const Parameter& t_parameter);

    private:
      void fromMsg(const rtask_msgs::Parameter& t_msg);

      std::string name_{};
      std::string type_{};
      bool valid_{false};
    };

    static std::ostream& operator<<(std::ostream& out, const Parameter& p)
    {
      out << "name: " << p.getName() << std::endl
          << "type: " << p.getType() << std::endl
          << "valid: " << p.isValid() << std::endl;
      return out;
    }

  } // namespace commons
} // namespace rtask

#endif
