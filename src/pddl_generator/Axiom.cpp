#include "pddl_generator/Axiom.h"
#include "pddl_generator/Helpers.h"

#include <algorithm>

using namespace rtask::commons::pddl_generator;

Axiom::Axiom(const LiteralTermVector& t_vars, LogicalExprPtr t_context, LogicalExprPtr t_implies)
{
  set(t_vars, t_context, t_implies);
}

Axiom::Axiom(XmlRpc::XmlRpcValue& t_rpc_val)
{
  if (!(t_rpc_val.getType() == XmlRpc::XmlRpcValue::Type::TypeStruct
        && helpers::checkXmlRpcSanity("vars", t_rpc_val, XmlRpc::XmlRpcValue::Type::TypeArray)
        && helpers::checkXmlRpcSanity("context", t_rpc_val, XmlRpc::XmlRpcValue::Type::TypeStruct)
        && helpers::checkXmlRpcSanity("implies", t_rpc_val, XmlRpc::XmlRpcValue::Type::TypeStruct))) {
    std::cerr << "Fatal: Invalid AXIOM Structure, should be a Struct with vars, context, and implies fields"
              << std::endl;
    exit(EXIT_FAILURE);
  }

  std::vector<LiteralTerm> vars;
  if (t_rpc_val["vars"].size() == 0) {
    std::cout << "W: Empty VARS vector for AXIOM" << std::endl;
  }
  else {
    for (int i = 0; i < t_rpc_val["vars"].size(); ++i) {
      vars.emplace_back(t_rpc_val["vars"][i]);
    }
  }
  setVariables(vars);

  const auto& context = helpers::getLogicalExprFromXmlRpc(t_rpc_val["context"]);
  if (!context) {
    std::cerr << "Fatal: Invalid CONTEXT of current AXIOM" << std::endl;
    exit(EXIT_FAILURE);
  }
  setContext(context);

  const auto& implies = helpers::getLogicalExprFromXmlRpc(t_rpc_val["implies"]);
  if (!implies) {
    std::cerr << "Fatal: Invalid IMPLIES of current AXIOM" << std::endl;
    exit(EXIT_FAILURE);
  }
  setImplies(implies);
}

void Axiom::clear()
{
  vars_.clear();
  vars_map_.clear();
  context_.reset();
  implies_.reset();
}

bool Axiom::set(const LiteralTermVector& t_vars, LogicalExprPtr t_context, LogicalExprPtr t_implies)
{
  setVariables(t_vars);
  setContext(t_context);
  setImplies(t_implies);
  return true;
}

void Axiom::setVariables(const LiteralTermVector& t_vars)
{
  vars_.clear();
  vars_map_.clear();
  for (const auto& var : t_vars) {
    if (vars_map_.count(var.getName()) != 0) {
      std::cerr << "Fatal: Duplicate VAR in current AXIOM" << std::endl;
      exit(EXIT_FAILURE);
    }
    vars_map_.emplace(var.getName(), var.getType());
    vars_.push_back(var);
  }
}

bool Axiom::isValid(const UmapStrStr& t_known_types,
                    const std::vector<LiteralTerm>& t_known_constants,
                    const std::vector<Predicate>& t_known_predicates,
                    const std::vector<LiteralExpression>& t_known_timeless) const
{
  for (auto& var : vars_) {
    if (!var.isValid(t_known_types)) {
      std::cout << "Validation Error: Invalid VAR " << var << " in current AXIOM" << std::endl;
      return false;
    }

    auto it = std::find(t_known_constants.begin(), t_known_constants.end(), var);
    if (it != t_known_constants.end()) {
      std::cerr << "Validation Error: invalid VAR " << var << " in current AXIOM, constant used" << std::endl;
      return false;
    }
  }

  if (!helpers::isValid(
        context_, vars_map_, t_known_types, t_known_constants, t_known_predicates, t_known_timeless, false)) {
    std::cerr << "Validation Error: invalid CONTEXT " << context_ << " of current AXIOM" << std::endl;
    return false;
  };

  if (!helpers::isValid(
        implies_, vars_map_, t_known_types, t_known_constants, t_known_predicates, t_known_timeless, true)) {
    std::cerr << "Validation Error: invalid IMPLIES " << implies_ << " of current AXIOM" << std::endl;
    return false;
  };

  return true;
}

Axiom& Axiom::operator=(const Axiom& t_other)
{
  set(t_other.getVariables(), t_other.getContext(), t_other.getImplies());
  return *this;
}

std::string Axiom::toPddl(bool t_typing, int t_pad_lv) const
{
  auto pad_aligners = helpers::getPddlAligners(t_pad_lv);
  const auto& pad_lv = pad_aligners.first;
  const auto& aligners = pad_aligners.second;

  std::string out = aligners[0] + "(" + ":axiom" + aligners[1];

  auto pad_aligners_n = helpers::getPddlAligners(pad_lv);
  const auto& pad_lv_n = pad_aligners_n.first;
  const auto& aligners_n = pad_aligners_n.second;

  out += aligners_n[0] + ":vars (";
  for (const auto& v : vars_) {
    out += v.toPddl(t_typing) + " ";
  }
  out.pop_back();
  out += ")";

  out += aligners_n[1] + aligners_n[0] + ":context" + aligners[1];
  out += context_->toPddl(t_typing, pad_lv_n);

  out += aligners_n[1] + aligners_n[0] + ":implies" + aligners[1];
  out += implies_->toPddl(t_typing, pad_lv_n);
  out += aligners[2] + ")";
  return out;
}

bool rtask::commons::pddl_generator::operator==(const Axiom& t_first, const Axiom& t_second)
{
  if (t_first.getNumVars() != t_second.getNumVars()) {
    return false;
  }

  const auto& f_vars = t_first.getVariables();
  const auto& s_vars = t_second.getVariables();
  for (unsigned int ix = 0; ix < f_vars.size(); ++ix) {
    if (f_vars.at(ix).getType() != s_vars.at(ix).getType())
      return false;
  }

  if (!(t_first.getContext().get() == t_second.getContext().get()
        && !(t_first.getImplies().get() == t_second.getImplies().get()))) {
    return false;
  }
  return true;
}

bool rtask::commons::pddl_generator::operator!=(const Axiom& t_first, const Axiom& t_second)
{
  return !(t_first == t_second);
}

std::ostream& rtask::commons::pddl_generator::operator<<(std::ostream& t_out, const Axiom& t_ax)
{
  return t_out << " ## AXIOM ## " << std::endl << t_ax.toPddl();
}

std::ostream& rtask::commons::pddl_generator::operator<<(std::ostream& t_out, std::shared_ptr<Axiom> t_expr_ptr)
{
  t_out << (t_expr_ptr ? *t_expr_ptr : Axiom());
  return t_out;
}
