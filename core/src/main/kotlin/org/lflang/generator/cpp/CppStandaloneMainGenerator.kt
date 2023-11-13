package org.lflang.generator.cpp

import org.lflang.target.TargetConfig
import org.lflang.generator.PrependOperator
import org.lflang.inferredType
import org.lflang.lf.Parameter
import org.lflang.lf.Reactor
import org.lflang.target.property.ExportDependencyGraphProperty
import org.lflang.target.property.ExportToYamlProperty
import org.lflang.target.property.FastProperty
import org.lflang.target.property.TimeOutProperty
import org.lflang.target.property.WorkersProperty
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
                        $targetType $name${CppTypes.getCppInitializer(init, inferredType)};
                        options
                            .add_options()("$name", "The $name parameter passed to the main reactor ${main.name}.", cxxopts::value<$targetType>($name)->default_value(time_to_string($name)), "'FLOAT UNIT'");
                    """.trimIndent()
                } else {
                    """
                        $targetType $name${CppTypes.getCppInitializer(init, inferredType)};
                        options
                            .add_options()("$name", "The $name parameter passed to the main reactor ${main.name}.", cxxopts::value<$targetType>($name)->default_value(any_to_string($name)), "'$targetType'");
                    """.trimIndent()
                }
            }
        }
    }

    private fun generateMainReactorInstantiation(): String =
            """auto main = std ::make_unique<${main.name}> ("${main.name}", &e, ${main.name}::Parameters{${main.parameters.joinToString(", ") { ".${it.name} = ${it.name}" }}});"""

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
            |int main(int argc, char **argv) {
            |  cxxopts::Options options("${fileConfig.name}", "Reactor Program");
            |
            |  unsigned workers = ${if (targetConfig.get(WorkersProperty.INSTANCE) != 0) targetConfig.get(WorkersProperty.INSTANCE) else "std::thread::hardware_concurrency()"};
            |  bool fast{${targetConfig.get(FastProperty.INSTANCE)}};
            |  reactor::Duration timeout = ${if (targetConfig.isSet(TimeOutProperty.INSTANCE)) targetConfig.get(TimeOutProperty.INSTANCE).toCppCode() else "reactor::Duration::max()"};
            |  
            |  // the timeout variable needs to be tested beyond fitting the Duration-type 
            |  options
            |    .set_width(120)
            |    .add_options()
            |      ("w,workers", "the number of worker threads used by the scheduler", cxxopts::value<unsigned>(workers)->default_value(std::to_string(workers)), "'unsigned'")
            |      ("o,timeout", "Time after which the execution is aborted.", cxxopts::value<reactor::Duration>(timeout)->default_value(time_to_string(timeout)), "'FLOAT UNIT'")
            |      ("f,fast", "Allow logical time to run faster than physical time.", cxxopts::value<bool>(fast)->default_value("${targetConfig.get(FastProperty.INSTANCE)}"))
            |      ("help", "Print help");
            |      
        ${" |"..main.parameters.joinToString("\n\n") { generateParameterParser(it) }}
            |
            |  cxxopts::ParseResult result{};
            |  bool parse_error{false};
            |  try {
            |    result = options.parse(argc, argv);
            |  } catch (const cxxopts::OptionException& e) {
            |    reactor::log::Error() << e.what();
            |    parse_error = true;
            |  }
            |  
            |  // if parameter --help was used or there was a parse error, print help
            |  if (parse_error || result.count("help"))
            |  {
            |       std::cout << options.help({""});
            |       return parse_error ? -1 : 0;
            |  }
            |
            |  reactor::Environment e{workers, fast, timeout};
            |
            |  // instantiate the main reactor
            |  ${generateMainReactorInstantiation()}
            |
            |  // assemble reactor program
            |  e.assemble();
        ${" |".. if (targetConfig.get(ExportDependencyGraphProperty.INSTANCE)) "e.export_dependency_graph(\"${main.name}.dot\");" else ""}
        ${" |".. if (targetConfig.get(ExportToYamlProperty.INSTANCE)) "e.dump_to_yaml(\"${main.name}.yaml\");" else ""}
            |
            |  // start execution
            |  auto thread = e.startup();
            |  thread.join();
            |  return 0;
            |}
        """.trimMargin()
    }
}
