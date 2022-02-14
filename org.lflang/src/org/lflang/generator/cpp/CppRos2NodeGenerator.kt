package org.lflang.generator.cpp

import org.lflang.TargetConfig
import org.lflang.lf.Reactor
import org.lflang.toUnixString

class CppRos2NodeGenerator(
    private val main: Reactor,
    private val targetConfig: TargetConfig,
    private val fileConfig: CppFileConfig
) {

    val nodeName = "${fileConfig.name}Node"

    fun generateHeader(): String {
        return """
            |#pragma once
            |
            |#include <rclcpp/rclcpp.hpp>
            |#include "reactor-cpp/reactor-cpp.hh"
            |
            |#include "${fileConfig.getReactorHeaderPath(main).toUnixString()}"
            |
            |class $nodeName : public rclcpp::Node {
            |private:
            |  std::unique_ptr<reactor::Environment> __lf_env;
            |  std::unique_ptr<${main.name}> __lf_main_reactor;
            |  std::thread __lf_thread;
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
            |$nodeName::$nodeName(const rclcpp::NodeOptions& node_options)
            |  : Node("$nodeName", node_options) {
            |  unsigned threads = ${if (targetConfig.threads != 0) targetConfig.threads else "std::thread::hardware_concurrency()"};
            |  bool fast{${targetConfig.fastMode}};
            |  bool keepalive{${targetConfig.keepalive}};
            | 
            |  __lf_env = std::make_unique<reactor::Environment>(threads, keepalive, fast);
            |
            |  // instantiate the main reactor
            |  __lf_main_reactor = std::make_unique<${main.name}> ("${main.name}", __lf_env.get());
            |
            |  // assemble reactor program
            |  __lf_env->assemble();
            |
            |  // start execution
            |  __lf_thread = __lf_env->startup();
            |}
            |
            |$nodeName::~$nodeName() {
            |  __lf_env->async_shutdown();
            |  __lf_thread.join();
            |}
            |
            |RCLCPP_COMPONENTS_REGISTER_NODE($nodeName)
        """.trimMargin()
    }
}