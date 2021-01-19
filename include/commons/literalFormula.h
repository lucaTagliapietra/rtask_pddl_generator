#ifndef rtask_commons_literal_formula_h
#define rtask_commons_literal_formula_h

#include "xmlrpcpp/XmlRpc.h"

#include <string>
#include <vector>

#include "commons/iAtomicFormula.h"
#include "commons/typedTerm.h"

namespace rtask {
  namespace commons {

    using StringVector = std::vector<std::string>;
    using UnorderedStringToUIntMap = std::unordered_map<std::string, unsigned int>;

    class LiteralFormula : public AtomicFormula
    {
    public:
      LiteralFormula() = default;
      ~LiteralFormula() = default;

      LiteralFormula(const std::string& t_name, const StringVector& t_args = {});
      LiteralFormula(XmlRpc::XmlRpcValue& t_rpc_val);

      // ---------------
      // Condition Level
      // ---------------
      void clear();
      void set(const std::string& t_name, const StringVector& t_args = {});

      inline std::string getName() const { return name_; }
      inline StringVector getArguments() const { return args_; }
      inline void setName(const std::string& t_name) { name_ = std::move(t_name); }
      inline void setArguments(const StringVector& t_args) { args_ = std::move(t_args); }

      std::string toPddl(const bool t_typing = true) const override;

      bool validate(const UnorderedNameTypedTermMap& t_known_constants,
                    const UnorderedStringToUIntMap& t_belonging_action_args,
                    const std::string& t_belonging_action_name) const;

      // ---------
      // Operators
      // ---------
      bool operator==(const LiteralFormula& t_other) const;
      LiteralFormula& operator=(const LiteralFormula& t_other);

    private:
      std::string name_{};
      StringVector args_{};
    };

    static std::ostream& operator<<(std::ostream& out, const LiteralFormula& lc)
    {
      out << "name: " << lc.getName() << std::endl;
      unsigned int i = 0;
      for (const auto& a : lc.getArguments()) {
        out << std::endl
            << "\t"
            << " - args[" << i << "]: " << a;
        ++i;
      }
      return out << std::endl;
    }

  } // namespace commons
} // namespace rtask

#endif
