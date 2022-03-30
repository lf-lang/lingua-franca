package org.lflang.generator.cpp

import org.lflang.TargetConfig
import org.lflang.generator.PrependOperator
import org.lflang.inferredType
import org.lflang.lf.Parameter
import org.lflang.lf.Reactor
import org.lflang.toUnixString

/** C++ code generator responsible for generating the main file including the main() function */
class CppStandaloneMainGenerator(
    private val main: Reactor,
    private val targetConfig: TargetConfig,
    private val fileConfig: CppFileConfig,
) {
    // Cxxopts generation
    private fun generateParameterParser(param: Parameter): String {
        with(CppParameterGenerator) {
            with(param) {
                return if(inferredType.isTime) {
                    """
                        $targetType $name = $defaultValue;
                        options
                            .add_options()("$name", "The $name parameter passed to the main reactor ${main.name}.", cxxopts::value<$targetType>($name)->default_value(time_to_string($name)), "'FLOAT UNIT'");
                    """.trimIndent()
                } else {
                    """
                        $targetType $name = $defaultValue;
                        options
                            .add_options()("$name", "The $name parameter passed to the main reactor ${main.name}.", cxxopts::value<$targetType>($name)->default_value(any_to_string($name)), "'$targetType'");
                    """.trimIndent()
                }
            }
        }
    }

    private fun generateMainReactorInstantiation(): String =
        if (main.parameters.isEmpty())
            """auto main = std ::make_unique<${main.name}> ("${main.name}", &e);"""
        else
            """auto main = std ::make_unique<${main.name}> ("${main.name}", &e, ${main.parameters.joinToString(", ") { it.name }});"""

    fun generateCode() = with(PrependOperator) {
        """
        ${" |"..fileComment(main.eResource())}
            |
            |#include <chrono>
            |#include <thread>
            |#include <memory>
            |
            |#include "reactor-cpp/reactor-cpp.hh"
            |
            |using namespace std::chrono_literals;
            |using namespace reactor::operators;
            |
            |
            |#include "${fileConfig.getReactorHeaderPath(main).toUnixString()}"
            |
            |#include "time_parser.hh"
            |#include "lf_timeout.hh"
            |
            |int main(int argc, char **argv) {
            |  cxxopts::Options options("${fileConfig.name}", "Reactor Program");
            |
            |  unsigned workers = ${if (targetConfig.workers != 0) targetConfig.workers else "std::thread::hardware_concurrency()"};
            |  bool fast{${targetConfig.fastMode}};
            |  bool keepalive{${targetConfig.keepalive}};
            |  reactor::Duration timeout = ${targetConfig.timeout?.toCppCode() ?: "reactor::Duration::zero()"};
            |  
            |  // the timeout variable needs to be tested beyond fitting the Duration-type 
            |  options
            |    .set_width(120)
            |    .add_options()
            |      ("w,workers", "the number of worker threads used by the scheduler", cxxopts::value<unsigned>(workers)->default_value(std::to_string(workers)), "'unsigned'")
            |      ("o,timeout", "Time after which the execution is aborted.", cxxopts::value<reactor::Duration>(timeout)->default_value(time_to_string(timeout)), "'FLOAT UNIT'")
            |      ("k,keepalive", "Continue execution even when there are no events to process.", cxxopts::value<bool>(keepalive)->default_value("${targetConfig.keepalive}"))
            |      ("f,fast", "Allow logical time to run faster than physical time.", cxxopts::value<bool>(fast)->default_value("${targetConfig.fastMode}"))
            |      ("help", "Print help");
            |      
        ${" |"..main.parameters.joinToString("\n\n") { generateParameterParser(it) }}
            |
            |  auto result = options.parse(argc, argv);
            |  
            |  // if parameter --help was used, print help
            |  if (result.count("help"))
            |  {
            |       std::cout << options.help({""}) << std::endl;
            |       exit(0);
            |   }
            |   
            |   // validate time parameters (inferredType.isTime) and the timeout parameter via the validate_time_string(val) function
            |
            |  reactor::Environment e{workers, keepalive, fast};
            |
            |  // instantiate the main reactor
            |  ${generateMainReactorInstantiation()}
            |
            |  // optionally instantiate the timeout reactor
            |  std::unique_ptr<__lf_Timeout> t{nullptr};
            |  if (timeout != reactor::Duration::zero()) {
            |    t = std::make_unique<__lf_Timeout>("__lf_Timeout", &e, timeout);
            |  }
            |
            |  // assemble reactor program
            |  e.assemble();
        ${" |".. if (targetConfig.exportDependencyGraph) "e.export_dependency_graph(\"${main.name}.dot\");" else ""}
        ${" |".. if (targetConfig.exportToYaml) "e.dump_to_yaml(\"${main.name}.yaml\");" else ""}
            |
            |  // start execution
            |  auto thread = e.startup();
            |  thread.join();
            |  return 0;
            |}
        """.trimMargin()
    }
}
