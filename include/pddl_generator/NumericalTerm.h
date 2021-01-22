#ifndef rtask_commons_pddl_generator_numerical_term_h
#define rtask_commons_pddl_generator_numerical_term_h

#include "Term.h"
#include "xmlrpcpp/XmlRpc.h"

#include <any>
#include <iostream>
#include <string>

namespace rtask {
  namespace commons {
    namespace pddl_generator {

      class NumericalTerm : public Term
      {
      public:
        NumericalTerm();
        ~NumericalTerm() override = default;

        NumericalTerm(const double& t_value);
        NumericalTerm(XmlRpc::XmlRpcValue& t_rpc_val);

        inline void set(const double& t_value) { value_ = std::move(t_value); }
        inline double getValue() const { return value_; }

        bool operator==(const NumericalTerm& t_other) const;
        NumericalTerm& operator=(const NumericalTerm& t_other);

        std::string toPddl(const bool t_typing = true) const override;
        std::any getAsChild(Term& t_parent) const override;
        std::any getAsChild(std::shared_ptr<Term> t_parent_ptr) const override;

      private:
        double value_{};
      };

      static std::ostream& operator<<(std::ostream& t_out, const NumericalTerm& t_nt)
      {
        t_out << "value: " << t_nt.getValue();
        return t_out;
      }

    } // namespace pddl_generator
  } // namespace commons
} // namespace rtask

#endif
