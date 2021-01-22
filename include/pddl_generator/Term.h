#ifndef rtask_commons_pddl_generator_term_h
#define rtask_commons_pddl_generator_term_h

#include "xmlrpcpp/XmlRpc.h"
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
        virtual ~Term() {}

        virtual std::string toPddl(const bool t_typing = true) const;

        inline TermType getObjectType() const { return obj_type_; }

      private:
        TermType obj_type_{TermType::Base};
      };

    } // namespace pddl_generator
  } // namespace commons
} // namespace rtask

#endif
