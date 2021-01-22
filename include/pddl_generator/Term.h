#ifndef rtask_commons_pddl_generator_term_h
#define rtask_commons_pddl_generator_term_h

#include "xmlrpcpp/XmlRpc.h"

#include <any>
#include <memory>
#include <string>

namespace rtask {
  namespace commons {
    namespace pddl_generator {

      enum class TermType
      {
        Base = 0,
        LiteralTerm = 1,
        NumericalTerm = 2
      };

      class Term
      {
      public:
        Term() = default;
        virtual ~Term() = default;

        virtual std::string toPddl(const bool t_typing = true) const { return {}; }
        virtual std::any getAsChild(Term& t_parent) const;
        virtual std::any getAsChild(std::shared_ptr<Term> t_parent_ptr) const;

        inline TermType getObjectType() const { return obj_type_; }

      protected:
        TermType obj_type_{TermType::Base};
      };

    } // namespace pddl_generator
  } // namespace commons
} // namespace rtask

#endif
