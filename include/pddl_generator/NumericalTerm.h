#ifndef rtask_commons_pddl_generator_numerical_term_h
#define rtask_commons_pddl_generator_numerical_term_h

#include "NumericalExpression.h"
#include "Term.h"

#include "xmlrpcpp/XmlRpc.h"

#include <iostream>
#include <limits>
#include <memory>
#include <variant>

namespace rtask {
  namespace commons {
    namespace pddl_generator {

      using IntDoubleVar = std::variant<int, double>;

      class NumericalTerm
        : public Term
        , public NumericalExpression
      {
      public:
        NumericalTerm();
        ~NumericalTerm() override = default;

        NumericalTerm(const double& t_value);
        NumericalTerm(const int& t_value);
        NumericalTerm(XmlRpc::XmlRpcValue& t_rpc_val);

        void set(const double& t_value);
        void set(const int& t_value);

        inline IntDoubleVar getValue() const { return val_; }

        std::string toPddl(bool t_typing = true, int t_pad_lv = 0) const override;

        NumericalTerm& operator=(const NumericalTerm& t_other);

      private:
        IntDoubleVar val_ = std::numeric_limits<double>::quiet_NaN();
      };

      std::ostream& operator<<(std::ostream& t_out, const NumericalTerm& t_nt);
      std::ostream& operator<<(std::ostream& t_out, std::shared_ptr<NumericalTerm> t_nt_ptr);
      bool operator==(const NumericalTerm& t_first, const NumericalTerm& t_second);

    } // namespace pddl_generator
  } // namespace commons
} // namespace rtask

#endif
