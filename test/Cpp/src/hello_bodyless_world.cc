#include "HelloBodylessWorld/HelloBodylessWorld.hh"

void HelloBodylessWorld::Inner::hello([[maybe_unused]] const reactor::StartupTrigger& startup) {
  std::cout << "Hello World." << std::endl;
}
