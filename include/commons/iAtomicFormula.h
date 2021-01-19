#ifndef rtask_commons_i_atomic_formula_h
#define rtask_commons_i_atomic_formula_h

#include "iFormula.h"
#include "xmlrpcpp/XmlRpc.h"
#include <string>

namespace rtask {
  namespace commons {

    enum class AtomicFormulaType
    {
      Base = 0,
      Literal = 1,
      Numeric = 2
    };

    class AtomicFormula : public Formula
    {
    public:
      virtual ~AtomicFormula() {}

      virtual std::string toPddl(const bool t_typing = true) const;

      const AtomicFormulaType getClassType() const { return class_type_; };

    private:
      AtomicFormulaType class_type_{AtomicFormulaType::Base};
    };
  } // namespace commons
} // namespace rtask

#endif