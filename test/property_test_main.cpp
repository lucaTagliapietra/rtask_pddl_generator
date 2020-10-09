#include "commons/property.h"
#include <variant>

int main(int argc, char* argv[])
{
  rtask::commons::Property prop("prova", false);
  std::cout << "Type : " << prop.get().first << " val: " << prop.get().second << std::endl;
}
