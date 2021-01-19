#ifndef rtask_commons_typed_term_h
#define rtask_commons_typed_term_h

#include "iTerm.h"
#include "xmlrpcpp/XmlRpc.h"

#include <iostream>
#include <string>
#include <unordered_map>

namespace rtask {
  namespace commons {

    class TypedTerm;

    using UnorderedNameTypedTermMap = std::unordered_map<std::string, TypedTerm>;

    class TypedTerm : public Term
    {
    public:
      TypedTerm() = default;
      ~TypedTerm() = default;

      TypedTerm(const std::string& t_name, const std::string& t_type = {"object"});
      TypedTerm(XmlRpc::XmlRpcValue& t_rpc_val);

      void set(const std::string& t_name, const std::string& t_type);
      inline void setName(const std::string& t_name) { name_ = std::move(t_name); }
      inline void setType(const std::string& t_type) { type_ = std::move(t_type); }

      inline std::string getName() const { return name_; }
      inline std::string getType() const { return type_; }

      std::string toPddl(const bool t_typing = true) const override;

      bool validate(const UnorderedNameTypedTermMap& t_types) const;

      bool operator==(const TypedTerm& t_other) const;
      TypedTerm& operator=(const TypedTerm& t_other);

    private:
      std::string name_{};
      std::string type_{"object"};
    };

    static std::ostream& operator<<(std::ostream& t_out, const TypedTerm& t_tn)
    {
      t_out << "name: " << t_tn.getName() << std::endl << "type: " << t_tn.getType() << std::endl;
      return t_out;
    }

  } // namespace commons
} // namespace rtask

#endif
