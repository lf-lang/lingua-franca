package org.lflang.generator.cpp

import org.lflang.generator.PrependOperator

class CppRos2NodeGenerator {
    fun generateCode(): String {
        return with(PrependOperator) {
            """
                |#include "lf_hello_world/HelloNode.hpp"
#include <rclcpp_components/register_node_macro.hpp>


#include <thread>
#include <memory>
#include "HelloReactor/HelloReactor.hh"


namespace autoware
{
namespace tools
{
namespace lf_hello_world
{

HelloNode::HelloNode(const rclcpp::NodeOptions & node_options)
  : Node("hello_node", node_options)
{
  unsigned threads = std::thread::hardware_concurrency();
  bool fast{false};
  bool keepalive{true};

  m_env = new reactor::Environment(threads, keepalive, fast);

  reactor = std::make_unique<HelloReactor>("HelloReactor", m_env, this);

  // execute the reactor program
  m_env->assemble();
  m_reactor_threads = m_env->startup();
}

HelloNode::~HelloNode()
{
  this->reactor->environment()->async_shutdown();
  this->m_reactor_threads.join();
  delete m_env;
}

}// namespace lf_hello_world
}// namespace tools
}// namespace autoware

RCLCPP_COMPONENTS_REGISTER_NODE(
    autoware::tools::lf_hello_world::HelloNode)
            """.trimIndent()
        }
    }
}