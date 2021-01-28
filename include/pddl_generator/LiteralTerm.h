#ifndef rtask_commons_pddl_generator_literal_term_h
#define rtask_commons_pddl_generator_literal_term_h

#include "Term.h"
#include "xmlrpcpp/XmlRpc.h"

#include <iostream>
#include <memory>
#include <string>
#include <unordered_map>

namespace rtask {
  namespace commons {
    namespace pddl_generator {

      class LiteralTerm;

      using UnordStrToLitTermMap = std::unordered_map<std::string, LiteralTerm>;
      using LiteralTermPtr = std::shared_ptr<LiteralTerm>;

      class LiteralTerm : public Term
      {
      public:
        LiteralTerm();
        ~LiteralTerm() override = default;

        LiteralTerm(const std::string& t_name, const std::string& t_type = {"object"});
        LiteralTerm(XmlRpc::XmlRpcValue& t_rpc_val);

        void set(const std::string& t_name, const std::string& t_type);
        inline void setName(const std::string& t_name) { name_ = std::move(t_name); }
        inline void setType(const std::string& t_type) { type_ = std::move(t_type); }

        inline std::string getName() const { return name_; }
        inline std::string getType() const { return type_; }

        bool validate(const UnordStrToLitTermMap& t_types) const;

        bool operator==(const LiteralTerm& t_other) const;
        LiteralTerm& operator=(const LiteralTerm& t_other);

        std::string toPddl(bool t_typing = true, int t_pad_lv = 0) const override;

      private:
        std::string name_{};
        std::string type_{"object"};
      };

      static std::ostream& operator<<(std::ostream& t_out, const LiteralTerm& t_tn)
      {
        t_out << "name: " << t_tn.getName() << std::endl << "type: " << t_tn.getType();
        return t_out;
      }

    } // namespace pddl_generator
  } // namespace commons
} // namespace rtask

#endif
