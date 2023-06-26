package org.lflang.generator.cpp

import org.lflang.TargetConfig
import org.lflang.lf.Reactor
import org.lflang.toText
import org.lflang.toUnixString
import org.lflang.generator.cpp.CppPortGenerator.Companion.dataType
import org.lflang.inferredType
import org.lflang.lf.VarRef

/** A C++ code generator for creating a ROS2 node from reactor definition */
class CppRos2NodeGenerator(
    private val reactor: Reactor,
    private val targetConfig: TargetConfig,
    private val fileConfig: CppFileConfig
) {

    val nodeName = "${reactor.name}Node"

    fun generateHeader(): String {
        return """
            |#pragma once
            |
            |#include <rclcpp/rclcpp.hpp>
            |#include "reactor-cpp/reactor-cpp.hh"
            |
            |#include "${fileConfig.getReactorHeaderPath(reactor).toUnixString()}"
            |
            |rclcpp::Node* lf_node{nullptr};
            |
            |class $nodeName : public rclcpp::Node {
            |private:
            |  std::unique_ptr<reactor::Environment> lf_env;
            |  std::unique_ptr<${reactor.name}> lf_reactor;
            |  ${reactor.inputs.joinToString(separator = "\n", prefix = "//\n"){ "std::unique_ptr<reactor::ROS2SubEndpoint<${it.inferredType.cppType}>> ${it.name}_sub;" } }
            |  ${reactor.outputs.joinToString(separator = "\n", prefix = "//\n"){ "std::unique_ptr<reactor::ROS2PubEndpoint<${it.inferredType.cppType}>> ${it.name}_pub;" } }
            |  // thread of the LF execution
            |  std::thread lf_thread;
            |  // an additional thread that we use for waiting for LF termination
            |  // and then shutting down the LF node
            |  std::thread lf_shutdown_thread;
            |
            |  void wait_for_lf_shutdown();
            |public:
            |  $nodeName(const rclcpp::NodeOptions& node_options);
            |  ~$nodeName();
            |};
        """.trimMargin()
    }

    fun generateSource(): String {
        return """
            |#include "$nodeName.hh"
            |#include <rclcpp_components/register_node_macro.hpp>
            |
            |#include <thread>
            |
            |void $nodeName::wait_for_lf_shutdown() {
            |  lf_thread.join();
            |  this->get_node_options().context()->shutdown("LF execution terminated");
            |}
            |
            |$nodeName::$nodeName(const rclcpp::NodeOptions& node_options = rclcpp::NodeOptions())
            |  : Node("$nodeName", node_options) {
            |  unsigned workers = ${if (targetConfig.workers != 0) targetConfig.workers else "std::thread::hardware_concurrency()"};
            |  bool fast{${targetConfig.fastMode}};
            |  reactor::Duration lf_timeout{${targetConfig.timeout?.toCppCode() ?: "reactor::Duration::max()"}};
            |
            |  // provide a globally accessible reference to this node
            |  // FIXME: this is pretty hacky...
            |  lf_node = this;
            |
            |  lf_env = std::make_unique<reactor::Environment>(workers, fast, lf_timeout);
            |
            |  // instantiate the main reactor
            |  lf_reactor = std::make_unique<${reactor.name}> ("${reactor.name}", lf_env.get(), ${reactor.name}::Parameters{});
            |  ${reactor}
            |  ${reactor.inputs.joinToString(separator = "\n", prefix = "//\n")
                { "${it.name}_sub = std::make_unique<reactor::ROS2SubEndpoint<${it.inferredType.cppType}>>(\"test\",\"${it.name}_sub\", lf_env.get(), false, std::chrono::nanoseconds(0));" + "${it.name}_sub->add_port(&lf_reactor->${it.name});" 
                 }}
            |  ${reactor.outputs.joinToString(separator = "\n", prefix = "//\n") { "${it.name}_pub = std::make_unique<reactor::ROS2PubEndpoint<${it.inferredType.cppType}>>(\"test\");" +
                "${it.name}_pub->set_port(&lf_reactor->${it.name});" }}
            |  // assemble reactor program
            |  lf_env->assemble();
            |
            |  // start execution
            |  lf_thread = lf_env->startup();
            |  lf_shutdown_thread = std::thread([this] { wait_for_lf_shutdown(); });
            |}
            |
            |$nodeName::~$nodeName() {
            |  if (lf_env->phase() == reactor::Environment::Phase::Execution) { 
            |    lf_env->async_shutdown();
            |  }
            |  lf_shutdown_thread.join();
            |}
            |
            |int main(int argc, char **argv) {
            |    rclcpp::init(argc, argv);
            |    auto node = std::make_shared<$nodeName>();
            |    rclcpp::spin(node);
            |    rclcpp::shutdown();
            |}
            |
        """.trimMargin()
    }
}
