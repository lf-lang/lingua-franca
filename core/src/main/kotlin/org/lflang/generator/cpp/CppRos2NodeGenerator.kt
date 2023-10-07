package org.lflang.generator.cpp

import org.lflang.*
import org.lflang.lf.*

/** A C++ code generator for creating a ROS2 node from reactor definition */
class CppRos2NodeGenerator(
    public val reactor: Reactor,
    private val targetConfig: TargetConfig,
    private val fileConfig: CppFileConfig
) {

    val nodeName = "${reactor.name}Node"

    private var subReactorEndpointDeclarations : String = ""
    private var subReactorEndpointInitializers : String = ""
    init {
        // generating endPoint declarations and initializers for non-federate-childs of this federate
        // recursively searching through all reactors of this federate to add the corresponding endpoints if they are in a federate connection
        // maybe TODO: resolve multiports banks etc
        val todo: MutableList<Triple<String, Reactor, List<Instantiation>>> = mutableListOf(Triple("", reactor, listOf()))
        while (todo.isNotEmpty()) {
            val (prefix, r, preInst) = todo.removeFirst()

            for (inst in r.instantiations) {
                if (!AttributeUtils.isFederate(inst)) todo.add(Triple(prefix + "/" + inst.name, inst.reactor, preInst + inst))
            }
            for (con in r.connections) {
                for (index in con.leftPorts.indices) {
                    val l : VarRef = con.leftPorts[index]
                    val r : VarRef = con.rightPorts[index]
                    if (l.container != null && !AttributeUtils.isFederate(l.container)
                        && r.container != null && AttributeUtils.isFederate(r.container)) {
                        val lPort = l.variable as Output
                        var lPortVarName = prefix + l.container.name + "_" + lPort.name
                        var topic_name = prefix + l.container.name + "/" + lPort.name
                        if (con.isPhysical) {
                            lPortVarName += "_physical"
                            subReactorEndpointDeclarations += System.lineSeparator() +
                                "|  std::unique_ptr<reactor::ROS2PubEndpointPhysical<${lPort.inferredType.cppType}, ${
                                    ROSMsgType(
                                        lPort.inferredType.cppType
                                    ).wrappedCppType
                                }>> $lPortVarName;"

                            subReactorEndpointInitializers += """
                                |  $lPortVarName = std::make_unique<reactor::ROS2PubEndpointPhysical<${lPort.inferredType.cppType}, ${ROSMsgType(lPort.inferredType.cppType).wrappedCppType}>>(lf_federate_prefix + "/$topic_name");
                                |  RCLCPP_DEBUG_STREAM(this->get_logger(), "subreactor physical endpoint $lPortVarName got topic "+ lf_federate_prefix + "/$topic_name");
                                """
                        }
                        else {
                            subReactorEndpointDeclarations += System.lineSeparator() +
                                "|  std::unique_ptr<reactor::ROS2PubEndpoint<${lPort.inferredType.cppType}, ${
                                    ROSMsgType(
                                        lPort.inferredType.cppType
                                    ).wrappedCppType
                                }>> $lPortVarName;"

                            subReactorEndpointInitializers += """
                                |  $lPortVarName = std::make_unique<reactor::ROS2PubEndpoint<${lPort.inferredType.cppType}, ${ROSMsgType(lPort.inferredType.cppType).wrappedCppType}>>(lf_federate_prefix + "/$topic_name");
                                |  RCLCPP_DEBUG_STREAM(this->get_logger(), "subreactor endpoint $lPortVarName got topic "+ lf_federate_prefix + "/$topic_name");
                            """
                        }

                        subReactorEndpointInitializers += """
                        |  reactor::Reactor* ${lPortVarName}_reactor = lf_reactor.get();
                        |  bool ${lPortVarName}_subreactor_found;
                        |  ${(preInst + l.container).joinToString(separator = System.lineSeparator()) {
                            """
                                        | ${lPortVarName}_subreactor_found = false;
                                        | for(auto r : ${lPortVarName}_reactor->reactors())
                                        |   if (r->name() == "${it.name}") {
                                        |       ${lPortVarName}_subreactor_found = true;
                                        |       ${lPortVarName}_reactor = r;
                                        |   }
                                        | if (!${lPortVarName}_subreactor_found)
                                        |   RCLCPP_ERROR(this->get_logger(), "Failed to find subreactor \"${it.name}\"");
                                    """
                        }
                        }
                        |  $lPortVarName->set_port(&dynamic_cast<${l.container.reactor.name}*>(${lPortVarName}_reactor)->${lPort.name});
                        """
                    }
                    if (r.container != null && !AttributeUtils.isFederate(r.container)
                        && l.container != null && AttributeUtils.isFederate(l.container)) {
                        val rPort = r.variable as Input
                        var rPortVarName = prefix + r.container.name + "_" + rPort.name
                        var topic_name= prefix + l.container.name + "/" + (l.variable as Port).name
                        // if the container is a federate where the out port is remapped check until we find one that should not be remapped
                        if (AttributeUtils.isFederate(l.container)) {
                            topic_name = prefix
                            var prev_l = l
                            while(AttributeUtils.isFederate(prev_l.container)) {
                                topic_name += prev_l.container.name
                                var next_l : VarRef? = null;
                                for (fed_con in prev_l.container.reactor.connections) {
                                    for ((fed_con_index, fed_con_rPort) in fed_con.rightPorts.withIndex()) {
                                        if (fed_con_rPort.name == (prev_l.variable as Port).name && fed_con_rPort.container == null)
                                            next_l = fed_con.leftPorts[fed_con_index]
                                    }
                                }
                                if (next_l == null) {
                                    topic_name += "/" +(prev_l.variable as Port).name
                                    break
                                }
                                else {
                                    topic_name += "/"
                                    prev_l = next_l
                                    next_l = null
                                }
                            }
                        }

                        if (con.isPhysical) {
                            rPortVarName += "_physical"
                            subReactorEndpointDeclarations += System.lineSeparator() +
                                    "|  std::unique_ptr<reactor::ROS2SubEndpointPhysical<${rPort.inferredType.cppType}, ${
                                        ROSMsgType(
                                            rPort.inferredType.cppType
                                        ).wrappedCppType
                                    }>> $rPortVarName;"

                            subReactorEndpointInitializers += """
                                |  $rPortVarName = std::make_unique<reactor::ROS2SubEndpointPhysical${if (con.delay != null) "Delayed" else ""}
                                |     <${rPort.inferredType.cppType}, ${ROSMsgType(rPort.inferredType.cppType).wrappedCppType}>>
                                |       (lf_federate_prefix + "/$topic_name","${rPortVarName}_sub", lf_env.get()
                                |           ${if (con.delay != null) ", " + con.delay.toCppTime() else ""});
                                |  RCLCPP_DEBUG_STREAM(this->get_logger(), "subreactor physical endpoint $rPortVarName got topic "+ lf_federate_prefix + "/$topic_name");
                                """
                        }
                        else {
                            subReactorEndpointDeclarations += System.lineSeparator() +
                                "|  std::unique_ptr<reactor::ROS2SubEndpoint<${rPort.inferredType.cppType}, ${
                                    ROSMsgType(
                                        rPort.inferredType.cppType
                                    ).wrappedCppType
                                }>> $rPortVarName;"

                            subReactorEndpointInitializers += """
                                |  $rPortVarName = std::make_unique<reactor::ROS2SubEndpoint${if (con.delay != null) "Delayed" else ""}
                                |      <${rPort.inferredType.cppType}, ${ROSMsgType(rPort.inferredType.cppType).wrappedCppType}>>
                                |       (lf_federate_prefix + "/$topic_name", "${rPortVarName}_sub",lf_env.get()
                                |           ${if (con.delay != null) ", " + con.delay.toCppTime() else ""});
                                |  RCLCPP_DEBUG_STREAM(this->get_logger(), "subreactor endpoint $rPortVarName got topic "+ lf_federate_prefix + "/$topic_name");
                            """
                        }

                        subReactorEndpointInitializers += """
                        |  reactor::Reactor* ${rPortVarName}_reactor = lf_reactor.get();
                        |  bool ${rPortVarName}_subreactor_found;
                        |  ${(preInst + r.container).joinToString(separator = System.lineSeparator()) {
                            """
                                        | ${rPortVarName}_subreactor_found = false;
                                        | for(auto r : ${rPortVarName}_reactor->reactors())
                                        |   if (r->name() == "${it.name}") {
                                        |       ${rPortVarName}_subreactor_found = true;
                                        |       ${rPortVarName}_reactor = r;
                                        |   }
                                        | if (!${rPortVarName}_subreactor_found)
                                        |   RCLCPP_ERROR(this->get_logger(), "Failed to find subreactor \"${it.name}\"");
                                    """
                        }
                        }
                        |  $rPortVarName->add_port(&dynamic_cast<${r.container.reactor.name}*>(${rPortVarName}_reactor)->${rPort.name});
                        """
                    }

                }

            }


        }
    }


    private fun generateEndpointDeclarations(): String {
        // for now we dont care if the outputs/inputs of this federate are used in physical or non-physical connections
        // hence both are generated and initialized based on parameters of launch script
        // maybe TODO: one could search for any connections of this federate starting from main reactor
        //          to figure out if and how this federates' ports are used and then generate only what is needed
        //          note: for banks/multiports would somehow need to know what exact instance this is (maybe via launch params)
        return """
        | // toplevel federates' input and output endpoints
        |  ${reactor.inputs.joinToString(separator = System.lineSeparator(), prefix = "//" + System.lineSeparator()) {
            """
                |  std::unique_ptr<reactor::ROS2SubEndpoint<${it.inferredType.cppType}, ${ROSMsgType(it.inferredType.cppType).wrappedCppType}>> ${it.name};
                |  std::unique_ptr<reactor::ROS2SubEndpointPhysical<${it.inferredType.cppType}, ${ROSMsgType(it.inferredType.cppType).wrappedCppType}>> ${it.name}_physical;
            """
            }
        }
        |  ${reactor.outputs.joinToString(separator = System.lineSeparator(), prefix = "//" + System.lineSeparator()) {
            """
                |  std::unique_ptr<reactor::ROS2PubEndpoint<${it.inferredType.cppType}, ${ROSMsgType(it.inferredType.cppType).wrappedCppType}>> ${it.name};
                |  std::unique_ptr<reactor::ROS2PubEndpointPhysical<${it.inferredType.cppType}, ${ROSMsgType(it.inferredType.cppType).wrappedCppType}>> ${it.name}_physical;
                """
            }
        }
        |
        | // endPoint declarations for this federates' subreactors
        $subReactorEndpointDeclarations
        """
    }


    fun generateHeader(): String {
        return """
            |#pragma once
            |
            |#include <rclcpp/rclcpp.hpp>
            |#include "${reactor.name}.hh"
            |#include "reactor-cpp/reactor-cpp.hh"
            |#include <std_msgs/msg/int64.hpp>
            |#include "reactor-cpp/ros2_connection_endpoint.hh"
            |
            |${reactor.allCppMessageTypes.map { it.wrappedMsgCppInclude }.joinLn() }
            |
            |#include "${fileConfig.getReactorHeaderPath(reactor).toUnixString()}"
            |
            |rclcpp::Node* lf_node{nullptr};
            |
            |class $nodeName : public rclcpp::Node {
            |private:
            |  std::unique_ptr<reactor::Environment> lf_env;
            |  std::unique_ptr<${reactor.name}> lf_reactor;
            |  std::string lf_federate_prefix;
            |  static const std::string LF_FEDERATE_PREFIX_PARAM_NAME;
            |  ${
                if (reactor.isMain) """
                   | std::shared_ptr<rclcpp::Publisher<std_msgs::msg::Int64>> start_time_publisher;
                """ else """
                   | std::shared_ptr<rclcpp::Subscription<std_msgs::msg::Int64>> start_time_subscription;
                """
                 }
            |  
            |  ${generateEndpointDeclarations()}
            |  
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
        // maybe TODO: naming a reactor "Node" leads to naming collisions with the rclcpp::Node
        // for now just dont do that

        return """
            |#include "$nodeName.hh"
            |#include "${reactor.name}.hh"
            |#include <rclcpp_components/register_node_macro.hpp>
            |
            |#include <thread>
            |
            |void $nodeName::wait_for_lf_shutdown() {
            |  RCLCPP_DEBUG_STREAM(this->get_logger(), "$nodeName waiting before shutting down");
            |  lf_thread.join();
            |  RCLCPP_DEBUG_STREAM(this->get_logger(), "$nodeName shutting down");
            |  this->get_node_options().context()->shutdown("LF execution terminated");
            |}
            |
            |const std::string $nodeName::LF_FEDERATE_PREFIX_PARAM_NAME = "lf_federate_prefix";
            |
            |$nodeName::$nodeName(const rclcpp::NodeOptions& node_options = rclcpp::NodeOptions())
            |  : rclcpp::Node("$nodeName", node_options) {
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
            |  
            |  this->declare_parameter(LF_FEDERATE_PREFIX_PARAM_NAME);
            |  rclcpp::Parameter lf_federate_prefix_param;
            |  if (this->get_parameter(LF_FEDERATE_PREFIX_PARAM_NAME, lf_federate_prefix_param)) 
            |       lf_federate_prefix = lf_federate_prefix_param.as_string();
            |  else RCLCPP_WARN_STREAM(this->get_logger(), "parameter \"" + LF_FEDERATE_PREFIX_PARAM_NAME + "\" missing");
            |       
            |
            |  ${reactor.inputs.joinToString(separator = System.lineSeparator(), prefix = "//" + System.lineSeparator()) {
                """
                    |  this->declare_parameter("${it.name}");
                    |  rclcpp::Parameter ${it.name}_topic_name_param;
                    |  if (this->get_parameter("${it.name}", ${it.name}_topic_name_param)) {
                    |       std::string ${it.name}_topic_name_string = ${it.name}_topic_name_param.as_string();
                    |       rclcpp::Parameter ${it.name}_delay;
                    |       this->declare_parameter("${it.name}_delay");
                    |       if (this->get_parameter("${it.name}_delay", ${it.name}_delay)) 
                    |           ${it.name} = std::make_unique<reactor::ROS2SubEndpointDelayed<${it.inferredType.cppType}, 
                    |                ${ROSMsgType(it.inferredType.cppType).wrappedCppType}>>(${it.name}_topic_name_string,"${it.name}_sub", lf_env.get(), std::chrono::nanoseconds(${it.name}_delay.as_int()));
                    |       else ${it.name} = std::make_unique<reactor::ROS2SubEndpoint<${it.inferredType.cppType}, 
                    |                ${ROSMsgType(it.inferredType.cppType).wrappedCppType}>>(${it.name}_topic_name_string,"${it.name}_sub", lf_env.get());
                    |       ${it.name}->add_port(&lf_reactor->${it.name});
                    |  }
                    |  
                    |  this->declare_parameter("${it.name}_physical");
                    |  rclcpp::Parameter ${it.name}_physical_topic_name_param;
                    |  if (this->get_parameter("${it.name}_physical", ${it.name}_physical_topic_name_param)) {
                    |       std::string ${it.name}_physical_topic_name_string = ${it.name}_physical_topic_name_param.as_string();
                    |       rclcpp::Parameter ${it.name}_delay;
                    |       this->declare_parameter("${it.name}_physical_delay");
                    |       if (this->get_parameter("${it.name}_physical_delay", ${it.name}_delay))
                    |           ${it.name}_physical = std::make_unique<reactor::ROS2SubEndpointPhysicalDelayed<${it.inferredType.cppType}, 
                    |                ${ROSMsgType(it.inferredType.cppType).wrappedCppType}>>(${it.name}_physical_topic_name_string,"${it.name}_physical_sub", lf_env.get(), std::chrono::nanoseconds(${it.name}_delay.as_int()));
                    |       else ${it.name}_physical = std::make_unique<reactor::ROS2SubEndpointPhysical<${it.inferredType.cppType}, 
                    |                ${ROSMsgType(it.inferredType.cppType).wrappedCppType}>>(${it.name}_physical_topic_name_string,"${it.name}_physical_sub", lf_env.get());
                    |       ${it.name}_physical->add_port(&lf_reactor->${it.name});
                    |  }
                """
                    }
                }
            |  ${reactor.outputs.joinToString(separator = System.lineSeparator(), prefix = "//" + System.lineSeparator()) {
                """
                    |  this->declare_parameter("${it.name}");
                    |  rclcpp::Parameter ${it.name}_topic_name_param;
                    |  if (this->get_parameter("${it.name}", ${it.name}_topic_name_param)) {
                    |       std::string ${it.name}_topic_name_string = ${it.name}_topic_name_param.as_string();
                    |       ${it.name} = std::make_unique<reactor::ROS2PubEndpoint<${it.inferredType.cppType}, ${ROSMsgType(it.inferredType.cppType).wrappedCppType}>>(${it.name}_topic_name_string);
                    |       ${it.name}->set_port(&lf_reactor->${it.name});
                    |  }
                    |  
                    |  this->declare_parameter("${it.name}_physical");
                    |  rclcpp::Parameter ${it.name}_physical_topic_name_param;
                    |  if (this->get_parameter("${it.name}_physical", ${it.name}_physical_topic_name_param)) {
                    |       std::string ${it.name}_physical_topic_name_string = ${it.name}_physical_topic_name_param.as_string();
                    |       ${it.name}_physical = std::make_unique<reactor::ROS2PubEndpointPhysical<${it.inferredType.cppType}, ${ROSMsgType(it.inferredType.cppType).wrappedCppType}>>(${it.name}_physical_topic_name_string);
                    |       ${it.name}_physical->set_port(&lf_reactor->${it.name});
                    |  }
                """
                    }
                }
            $subReactorEndpointInitializers
            |  // assemble reactor program
            |  lf_env->assemble();
            |  
            |  const std::string LF_STARTUP_TIME_TOPIC = "__lf_startup_time__";
            |  ${ if (reactor.isMain) """
                    | reactor::TimePoint start_time = reactor::get_physical_time();
                    | start_time_publisher = this->create_publisher<std_msgs::msg::Int64>(LF_STARTUP_TIME_TOPIC, rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local());
                    | {
                    |   std_msgs::msg::Int64 time_point_message;
                    |   time_point_message.data = start_time.time_since_epoch().count();
                    |   start_time_publisher->publish(time_point_message);
                    | }
                    | 
                    | // start execution
                    |  lf_thread = lf_env->startup(start_time);
                    |  lf_shutdown_thread = std::thread([this] { wait_for_lf_shutdown(); });
                """ else """
                    |   start_time_subscription = this->create_subscription<std_msgs::msg::Int64>(LF_STARTUP_TIME_TOPIC, rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local(),
                    |          [&](const std_msgs::msg::Int64::SharedPtr msg) {
                    |                 reactor::TimePoint start_time(std::chrono::nanoseconds(msg->data));
                    |                 // start execution
                    |                 lf_thread = lf_env->startup(start_time);
                    |                 lf_shutdown_thread = std::thread([this] { wait_for_lf_shutdown(); });        
                    |                 start_time_subscription.reset();
                    |    });
                """ }
            |  
            |}
            |
            |$nodeName::~$nodeName() {
            |  if (lf_env->phase() == reactor::Phase::Execution) {
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
