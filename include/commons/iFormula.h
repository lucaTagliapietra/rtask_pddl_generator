#ifndef rtask_commons_i_formula_h
#define rtask_commons_i_formula_h

#include "xmlrpcpp/XmlRpc.h"
#include <string>

namespace rtask {
  namespace commons {

    enum FormulaType
    {
      Base = 0,
      Not = 1,
      Or = 2,
      And = 3
    };

    class Formula
    {
    public:
      virtual ~Formula() {}

      virtual std::string& toPddl();

      const FormulaType getFormulaType() const { return class_type_; };

    private:
      FormulaType class_type_{FormulaType::Base};
    };
  } // namespace commons
} // namespace rtask

#endif