#ifndef rtask_commons_pddl_generator_numerical_function_h
#define rtask_commons_pddl_generator_numerical_function_h

#include "NumericalExpression.h"
#include "xmlrpcpp/XmlRpc.h"

#include <iostream>

namespace rtask {
  namespace commons {
    namespace pddl_generator {

      class NumericalFunction : public NumericalExpression
      {
      public:
        NumericalFunction();
        ~NumericalFunction() override = default;

        NumericalFunction(const std::string& t_name, const std::vector<std::string>& t_args = {});
        NumericalFunction(XmlRpc::XmlRpcValue& t_rpc_val);

        void clear();
        bool set(const std::string& t_name, const std::vector<std::string>& t_args = {});

        bool setFunctionName(const std::string& t_fnc_name);
        bool setFunctionArgs(const std::vector<std::string>& t_args);

        inline std::string getFunctionName() const { return function_name_; }
        inline std::vector<std::string> getFunctionArgs() const { return args_; }

        //        bool validate(const UnordStrToLitTermMap& t_known_constants,
        //                      const UnordStrToUIntMap& t_belonging_action_args,
        //                      const std::string& t_belonging_action_name) const;

        NumericalFunction& operator=(const NumericalFunction& t_other);

        std::string toPddl(bool t_typing = true, int t_pad_lv = 0) const override;

      private:
        std::string function_name_{};
        std::vector<std::string> args_{};
      };

      bool operator==(const NumericalFunction& t_first, const NumericalFunction& t_second);

      static std::ostream& operator<<(std::ostream& t_out, const NumericalFunction& t_expr)
      {
        t_out << "NumericalFunction name: " << t_expr.getFunctionName() << std::endl;
        unsigned int i = 0;
        for (const auto& a : t_expr.getFunctionArgs()) {
          (i != 0) ? t_out << std::endl : t_out << "";
          t_out << "\t"
                << " - args[" << i << "]: " << a;
          ++i;
        }
        return t_out;
      }

    } // namespace pddl_generator
  } // namespace commons
} // namespace rtask

#endif