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
        Invalid = 0,
        Base,
        LiteralTerm,
        NumericalTerm
      };

      class Term
      {
      public:
        Term() = default;
        virtual ~Term() = default;

        virtual std::string toPddl(bool t_typing = true, int t_pad_lv = 0) const { return {}; }

        inline TermType getObjectType() const { return obj_type_; }

      protected:
        TermType obj_type_{TermType::Base};
      };

    } // namespace pddl_generator
  } // namespace commons
} // namespace rtask

#endif
