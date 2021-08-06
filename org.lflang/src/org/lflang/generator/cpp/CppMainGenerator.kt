package org.lflang.generator.cpp

import org.lflang.TargetConfig
import org.lflang.generator.PrependOperator
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

    private fun generateParameterParser(param: Parameter): String {
        with(CppParameterGenerator) {
            with(param) {
                return if(inferredType.isTime) {
                    """
                        $targetType $name = $defaultValue;
                        auto opt_$name = app.add_option("--$name", $name, "The $name parameter passed to the main reactor ${main.name}.");
                        opt_$name->check([](const std::string& val){ return validate_time_string(val); });
                        opt_$name->type_name("'FLOAT UNIT'");
                        opt_$name->default_str(time_to_quoted_string($name)); 
                    """.trimIndent()
                } else {
                    """
                        $targetType $name = $defaultValue;
                        app.add_option("--$name", $name, "The $name parameter passed to the main reactor ${main.name}.");
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
            |#include "time_parser.hh"
            |
            |#include "CLI/CLI11.hpp"
            |
            |#include "${fileConfig.getReactorHeaderPath(main).toUnixString()}"
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
            |  CLI::App app ("${fileConfig.name} Reactor Program");
            |
            |  unsigned threads = ${if (targetConfig.threads != 0) targetConfig.threads else "std::thread::hardware_concurrency()"};
            |  app.add_option("-t,--threads", threads, "the number of worker threads used by the scheduler", true);
            |
            |  reactor::Duration timeout = ${targetConfig.timeout?.toCode() ?: "reactor::Duration::zero()"};
            |  auto opt_timeout = app.add_option ("-o,--timeout", timeout, "Time after which the execution is aborted.");
            |
            |  opt_timeout->check([](const std::string& val ){ return validate_time_string(val); });
            |  opt_timeout->type_name("'FLOAT UNIT'");
            |  opt_timeout->default_str(time_to_quoted_string(timeout));
            |
            |  bool fast{${targetConfig.fastMode}};
            |  app.add_flag("-f,--fast", fast, "Allow logical time to run faster than physical time.");
            |
            |  bool keepalive {${targetConfig.keepalive}};
            |  app.add_flag("-k,--keepalive", keepalive, "Continue execution even when there are no events to process.");
            |
        ${" |"..main.parameters.joinToString("\n\n") { generateParameterParser(it) }}
            |
            |  app.get_formatter()->column_width(50);
            |
            |  CLI11_PARSE(app, argc, argv);
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
            |  auto thread = e . startup ();
            |  thread.join();
            |
            |  return 0;
            |}
        """.trimMargin()
    }
}