#ifndef rtask_commons_i_term_h
#define rtask_commons_i_term_h

#include "xmlrpcpp/XmlRpc.h"
#include <string>

namespace rtask {
  namespace commons {

    enum class TermType
    {
      Base = 0,
      Typed = 1,
      Numeric = 2,
      ConstTyped = 3,
      ConstNumeric = 4
    };

    class Term
    {
    public:
      virtual ~Term() {}

      virtual std::string toPddl(const bool t_typing = true) const;

      const TermType getClassType() const { return class_type_; };

    private:
      TermType class_type_{TermType::Base};
    };
  } // namespace commons
} // namespace rtask

#endif