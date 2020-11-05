#ifndef rtask_commons_predicate_h
#define rtask_commons_predicate_h

#include "xmlrpcpp/XmlRpc.h"

#include <string>
#include <vector>

#include "commons/parameter.h"

#include "rtask_msgs/Predicate.h"

namespace rtask {
  namespace commons {

    class Predicate
    {
    public:
      Predicate() = default;
      ~Predicate() = default;

      Predicate(const std::string& t_name, const std::vector<Parameter>& t_params = {});
      Predicate(const rtask_msgs::PredicateConstPtr t_msg_ptr);
      Predicate(const rtask_msgs::Predicate& t_msg);
      Predicate(XmlRpc::XmlRpcValue& t_rpc_val);

      // ---------------
      // Predicate Level
      // ---------------
      rtask_msgs::Predicate toMsg() const;

      void clear();
      void set(const std::string& t_name, const std::vector<Parameter>& t_params);

      inline std::string getName() const { return name_; }
      inline std::vector<Parameter> getParameters() const { return parameters_; }
      inline bool isValid() const { return valid_; }

      std::vector<std::string> getParameterList() const;

      std::string toPddl(const bool t_typing = true) const;

      // ---------------
      // Parameter Level
      // ---------------
      bool hasParameter(const std::string& t_name) const;
      bool deleteParameter(const std::string& t_name);
      bool isParameterValid(const std::string& t_name) const;
      std::pair<bool, Parameter> getParameter(const std::string& t_name) const;
      void setParameter(const std::string& t_name, const std::string& t_type = {});

      // ---------
      // Operators
      // ---------
      bool operator==(const Predicate& t_predicate) const;
      Predicate& operator=(const Predicate& t_predicate);

    private:
      void fromMsg(const rtask_msgs::Predicate& t_msg);
      void updValidity();

      std::string name_{};
      std::vector<Parameter> parameters_{};
      bool valid_{false};
    };

    static std::ostream& operator<<(std::ostream& out, const Predicate& p)
    {
      out << "name: " << p.getName() << std::endl << "valid: " << p.isValid() << std::endl;
      unsigned int i = 0;
      for (const auto& par : p.getParameters()) {
        out << "\t"
            << " - params[" << i << "]: " << par;
        ++i;
      }
      return out << std::endl;
    }

  } // namespace commons
} // namespace rtask

#endif
