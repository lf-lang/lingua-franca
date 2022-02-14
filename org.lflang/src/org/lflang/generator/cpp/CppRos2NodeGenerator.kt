package org.lflang.generator.cpp

import org.lflang.TargetConfig
import org.lflang.generator.PrependOperator.rangeTo
import org.lflang.lf.Reactor
import org.lflang.toUnixString

class CppRos2NodeGenerator(
    private val main: Reactor,
    private val targetConfig: TargetConfig,
    private val fileConfig: CppFileConfig
) {

    public val nodeName = "${fileConfig.name}Node"

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
            |  std::unique_ptr<HelloReactor> __lf_main_reactor;
            |public:
            |  $nodeName(const rclcpp::NodeOptions& node_options);
            |  ~$nodeName();
            |};
        """.trimMargin()
    }

    fun generateSource(): String {
        return """
            |#include "$nodeName.hpp"
            |#include <rclcpp_components/register_node_macro.hpp>
            |
            |#include <thread>
            |
            |$nodeName::$nodeName() {
            |  unsigned threads = ${if (targetConfig.threads != 0) targetConfig.threads else "std::thread::hardware_concurrency()"};
            |  bool fast{${targetConfig.fastMode}};
            |  bool keepalive{${targetConfig.keepalive}};
            | 
            |  __lf_env = std::make_unique<reactor::Environment>{threads, keepalive, fast};
            |
            |  // instantiate the main reactor
            |  __lf_main_reactor = std::make_unique<${main.name}> ("${main.name}", &e);
            |
            |  // assemble reactor program
            |  e.assemble();
        ${" |".. if (targetConfig.exportDependencyGraph) "e.export_dependency_graph(\"${main.name}.dot\");" else ""}
        ${" |".. if (targetConfig.exportToYaml) "e.dump_to_yaml(\"${main.name}.yaml\");" else ""}
            |
            |  // start execution
            |  auto thread = e.startup();
            |}
            |
            |$nodeName::~$nodeName() {
            |  this->reactor->environment()->async_shutdown();
            |  this->m_reactor_threads.join();  
            |}
            |
            |RCLCPP_COMPONENTS_REGISTER_NODE($nodeName)
        """.trimMargin()
    }
}