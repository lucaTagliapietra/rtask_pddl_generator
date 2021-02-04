#ifndef rtask_commons_pddl_generator_predicate_h
#define rtask_commons_pddl_generator_predicate_h

#include "LiteralTerm.h"
#include "xmlrpcpp/XmlRpc.h"

#include <iostream>
#include <memory>

namespace rtask {
  namespace commons {
    namespace pddl_generator {

      class Predicate
      {
      public:
        Predicate() = default;
        ~Predicate() = default;

        Predicate(const std::string& t_name, const LiteralTermVector& t_params = {});
        Predicate(XmlRpc::XmlRpcValue& t_rpc_val);

        void clear();
        bool set(const std::string& t_name, const LiteralTermVector& t_params = {});

        bool setName(const std::string& t_name);
        void setParameters(const LiteralTermVector& t_params = {});

        int getParameterIndex(const std::string& t_name) const;
        int getParameterIndex(const LiteralTerm& t_param) const;

        bool hasParameter(const std::string& t_name) const;
        bool hasParameter(const LiteralTerm& t_param) const;

        ConstLiteralTermPtr getParameter(const std::string& t_name) const;
        ConstLiteralTermPtr getParameter(int t_idx) const;

        std::string getParameterType(const std::string& t_name) const;
        std::string getParameterType(int t_idx) const;

        inline std::string getName() const { return name_; }
        inline int getNumParameters() const { return static_cast<int>(params_.size()); }
        std::shared_ptr<const LiteralTermVector> getParameters() const;

        //        bool validate(const UnordStrToLitTermMap& t_known_constants,
        //                      const UnordStrToUIntMap& t_belonging_action_args,
        //                      const std::string& t_belonging_action_name) const;

        Predicate& operator=(const Predicate& t_other);
        std::string toPddl(bool t_typing = true, int t_pad_lv = 0) const;

      private:
        std::string name_{};
        LiteralTermVector params_{};
      };

      bool operator==(const Predicate& t_first, const Predicate& t_second);
      std::ostream& operator<<(std::ostream& t_out, const Predicate& t_expr);
      std::ostream& operator<<(std::ostream& t_out, std::shared_ptr<Predicate> t_expr_ptr);

    }; // namespace pddl_generator
  } // namespace commons
} // namespace rtask

#endif
