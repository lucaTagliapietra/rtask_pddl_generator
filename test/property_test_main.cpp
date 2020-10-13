#include "commons/property.h"
#include <variant>

int main(int argc, char* argv[])
{
  rtask::commons::Property test_prop("boolname", true);
  auto msg = test_prop.toMsg();
  std::cout << msg.name << std::endl;
  std::cout << std::to_string(msg.type) << std::endl;
  std::cout << msg.value << std::endl;
}
