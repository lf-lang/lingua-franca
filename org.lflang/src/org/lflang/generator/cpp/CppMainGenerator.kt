package org.lflang.generator.cpp

import org.lflang.TargetConfig
import org.lflang.generator.PrependOperator
import org.lflang.generator.PrependOperator.rangeTo
import org.lflang.inferredType
import org.lflang.lf.Parameter
import org.lflang.lf.Reactor
import org.lflang.toUnixString

/** C++ code generator responsible for generating the main file including the main() function */
class CppMainGenerator(
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
            |
            |class Timeout : public reactor::Reactor {
            | private:
            |  reactor::Timer timer;
            |
            |  reactor::Reaction r_timer{"r_timer", 1, this, [this]() { environment()->sync_shutdown(); }};
            |
            |
            | public:
            |  Timeout(const std ::string& name, reactor::Environment* env, reactor::Duration timeout)
            |    : reactor::Reactor(name, env)
            |    , timer{ "timer", this, reactor::Duration::zero(), timeout } {}
            |
            |  void assemble () override { r_timer.declare_trigger(& timer); }
            |};
            |
            |int main(int argc, char **argv) {
            |  cxxopts::Options options("${fileConfig.name}", "Reactor Program");
            |
            |  unsigned threads = ${if (targetConfig.threads != 0) targetConfig.threads else "std::thread::hardware_concurrency()"};
            |  bool fast{${targetConfig.fastMode}};
            |  bool keepalive{${targetConfig.keepalive}};
            |  reactor::Duration timeout = ${targetConfig.timeout?.toCppCode() ?: "reactor::Duration::zero()"};
            |  
            |  // the timeout variable needs to be tested beyond fitting the Duration-type 
            |  options
            |    .set_width(120)
            |    .add_options()
            |      ("t,threads", "the number of worker threads used by the scheduler", cxxopts::value<unsigned>(threads)->default_value(std::to_string(threads)), "'unsigned'")
            |      ("o,timeout", "Time after which the execution is aborted.", cxxopts::value<reactor::Duration>(timeout)->default_value(time_to_string(timeout)), "'FLOAT UNIT'")
            |      ("k,keepalive", "Continue execution even when there are no events to process.", cxxopts::value<bool>(keepalive))
            |      ("f,fast", "Allow logical time to run faster than physical time.", cxxopts::value<bool>(fast))
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
            |  reactor::Environment e{threads, keepalive, fast};
            |
            |  // instantiate the main reactor
            |  ${generateMainReactorInstantiation()}
            |
            |  // optionally instantiate the timeout reactor
            |  std::unique_ptr<Timeout> t{nullptr};
            |  if (timeout != reactor::Duration::zero()) {
            |    t = std::make_unique<Timeout>("Timeout", & e, timeout);
            |  }
            |
            |  // execute the reactor program
            |  e.assemble();
            |  auto thread = e.startup();
            |  thread.join();
        ${" |".. if (targetConfig.exportDependencyGraph) "e.export_dependency_graph(\"${main.name}.dot\");" else ""}
            |  return 0;
            |}
        """.trimMargin()
    }
}
