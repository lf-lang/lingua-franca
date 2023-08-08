#include "HelloBodylessWorld/HelloBodylessWorld.hh"

void HelloBodylessWorld::Inner::hello_body([[maybe_unused]] const reactor::StartupTrigger& startup) {
  std::cout << "Hello World." << std::endl;
}
