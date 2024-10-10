package org.lflang.generator.c;

import static org.lflang.generator.c.CGenerator.variableStructType;

import org.lflang.AttributeUtils;
import org.lflang.MessageReporter;
import org.lflang.ast.ASTUtils;
import org.lflang.generator.CodeBuilder;
import org.lflang.generator.PortInstance;
import org.lflang.lf.Input;
import org.lflang.lf.Output;
import org.lflang.lf.Port;
import org.lflang.lf.ReactorDecl;
import org.lflang.target.Target;
import org.lflang.target.TargetConfig;
import org.lflang.target.property.SchedulerProperty;
import org.lflang.target.property.type.SchedulerType.Scheduler;

/**
 * Generates C code to declare and initialize ports.
 *
 * @author Edward A. Lee
 * @author Soroush Bateni
 * @author Hou Seng Wong
 */
public class CPortGenerator {
  /** Generate fields in the self struct for input and output ports */
  public static void generateDeclarations(
      TypeParameterizedReactor tpr, CTypes types, CodeBuilder body, CodeBuilder constructorCode) {
    generateInputDeclarations(tpr, types, body, constructorCode);
    generateOutputDeclarations(tpr, types, body, constructorCode);
  }

  /**
   * This code-generates the allocation and initialization of the `output_ports` pointer-array on
   * the self_base_t. It is used by the STATIC scheduler to reset `is_present` fields on a
   * per-reactor level. Standard way is resetting all `is_present` fields at the beginning of each
   * tag. With STATIC scheduler we advance time in different reactors individually and must also
   * reset the `is_present` fields individually.
   *
   * @param tpr
   * @param decl
   * @param constructorCode
   */
  public static void generateOutputPortsPointerArray(
      TypeParameterizedReactor tpr, ReactorDecl decl, CodeBuilder constructorCode) {

    var outputs = ASTUtils.allOutputs(tpr.reactor());
    int numOutputs = outputs.size();

    constructorCode.pr("#ifdef REACTOR_LOCAL_TIME");
    constructorCode.pr("self->base.num_output_ports = " + numOutputs + ";");
    constructorCode.pr(
        "self->base.output_ports = (lf_port_base_t **) calloc("
            + numOutputs
            + ", sizeof(lf_port_base_t*));");
    constructorCode.pr("LF_ASSERT(self->base.output_ports != NULL, \"Out of memory\");");

    for (int i = 0; i < numOutputs; i++) {
      constructorCode.pr(
          "self->base.output_ports["
              + i
              + "]= (lf_port_base_t *) &self->_lf_"
              + outputs.get(i).getName()
              + ";");
    }
    constructorCode.pr("#endif");
  }

  /**
   * Generate the struct type definitions for the port of the reactor
   *
   * @param tpr The reactor
   * @param port The port to generate the struct
   * @param target The target of the code generation (C, CCpp or Python)
   * @param messageReporter The error reporter
   * @param types The helper object for types related stuff
   * @param federatedExtension The code needed to support federated execution
   * @param userFacing Whether this struct is to be presented in a user-facing header
   * @param decl The reactorDecl if this struct is for the header of this reactor's container; null
   *     otherwise
   * @return The auxiliary struct for the port as a string
   */
  public static String generateAuxiliaryStruct(
      TargetConfig targetConfig,
      TypeParameterizedReactor tpr,
      Port port,
      Target target,
      MessageReporter messageReporter,
      CTypes types,
      CodeBuilder federatedExtension,
      CodeBuilder staticExtension,
      boolean userFacing,
      ReactorDecl decl) {
    assert decl == null || userFacing;
    var code = new CodeBuilder();
    code.pr("typedef struct {");
    code.indent();
    // NOTE: The following fields are required to be the first ones so that
    // pointer to this struct can be cast to a (lf_port_base_t*) or to
    // (token_template_t*) to access these fields for any port.
    // IMPORTANT: These must match exactly the fields defined in port.h!!
    code.pr(
        String.join(
            "\n",
            "token_type_t type;", // From token_template_t
            "lf_token_t* token;", // From token_template_t
            "size_t length;", // From token_template_t
            "bool is_present;",
            "lf_port_internal_t _base;"));
    code.pr(valueDeclaration(tpr, port, target, messageReporter, types));
    code.pr(federatedExtension.toString());
    if (targetConfig.get(SchedulerProperty.INSTANCE).type() == Scheduler.STATIC) {
      code.pr(staticExtension.toString());
    }
    code.unindent();
    var name =
        decl != null
            ? localPortName(tpr, decl, port.getName())
            : variableStructType(port, tpr, userFacing);
    code.pr("} " + name + ";");
    return code.toString();
  }

  public static String localPortName(
      TypeParameterizedReactor tpr, ReactorDecl decl, String portName) {
    return decl.getName().toLowerCase() + tpr.argsString() + "_" + portName + "_t";
  }

  /**
   * Allocate memory for the input port.
   *
   * @param input The input port
   * @param reactorSelfStruct The name of the self struct
   */
  public static String initializeInputMultiport(PortInstance input, String reactorSelfStruct) {
    var portRefName = CUtil.portRefName(input);
    // If the port is a multiport, create an array.
    if (input.isMultiport()) {
      String result =
          String.join(
              "\n",
              portRefName + "_width = " + input.getWidth() + ";",
              "// Allocate memory for multiport inputs.",
              portRefName + " = (" + variableStructType(input) + "**)lf_allocate(",
              "        " + input.getWidth() + ", sizeof(" + variableStructType(input) + "*),",
              "        &" + reactorSelfStruct + "->base.allocations); ",
              "// Set inputs by default to an always absent default input.",
              "for (int i = 0; i < " + input.getWidth() + "; i++) {",
              "    "
                  + portRefName
                  + "[i] = &"
                  + reactorSelfStruct
                  + "->_lf_default__"
                  + input.getName()
                  + ";",
              "}");
      if (AttributeUtils.isSparse(input.getDefinition())) {
        return String.join(
            "\n",
            result,
            "if (" + input.getWidth() + " >= LF_SPARSE_WIDTH_THRESHOLD) {",
            "    " + portRefName + "__sparse = (lf_sparse_io_record_t*)lf_allocate(1,",
            "            sizeof(lf_sparse_io_record_t) + sizeof(size_t) * "
                + input.getWidth()
                + "/LF_SPARSE_CAPACITY_DIVIDER,",
            "            &" + reactorSelfStruct + "->base.allocations);",
            "    "
                + portRefName
                + "__sparse->capacity = "
                + input.getWidth()
                + "/LF_SPARSE_CAPACITY_DIVIDER;",
            "    if (sparse_io_record_sizes.start == NULL) {",
            "        sparse_io_record_sizes = vector_new(1);",
            "    }",
            "    vector_push(&sparse_io_record_sizes, (void*)&" + portRefName + "__sparse->size);",
            "}");
      }
      return result;
    } else {
      return String.join(
          "\n",
          "// width of -2 indicates that it is not a multiport.",
          portRefName + "_width = -2;");
    }
  }

  /**
   * Allocate memory for the output port.
   *
   * @param output The output port
   * @param reactorSelfStruct The name of the self struct
   */
  public static String initializeOutputMultiport(PortInstance output, String reactorSelfStruct) {
    var portRefName = CUtil.portRefName(output);
    var portStructType = variableStructType(output);
    return output.isMultiport()
        ? String.join(
            "\n",
            portRefName + "_width = " + output.getWidth() + ";",
            "// Allocate memory for multiport output.",
            portRefName + " = (" + portStructType + "*)lf_allocate(",
            "        " + output.getWidth() + ", sizeof(" + portStructType + "),",
            "        &" + reactorSelfStruct + "->base.allocations); ",
            portRefName + "_pointers = (" + portStructType + "**)lf_allocate(",
            "        " + output.getWidth() + ", sizeof(" + portStructType + "*),",
            "        &" + reactorSelfStruct + "->base.allocations); ",
            "// Assign each output port pointer to be used in",
            "// reactions to facilitate user access to output ports",
            "for(int i=0; i < " + output.getWidth() + "; i++) {",
            "        " + portRefName + "_pointers[i] = &(" + portRefName + "[i]);",
            "}")
        : String.join(
            "\n",
            "// width of -2 indicates that it is not a multiport.",
            portRefName + "_width = -2;");
  }

  /**
   * For the specified port, return a declaration for port struct to contain the value of the port.
   * A multiport output with width 4 and type {@code int[10]}, for example, will result in this:
   *
   * <pre><code>
   *     int value[10];
   * </code></pre>
   *
   * There will be an array of size 4 of structs, each containing this value array.
   *
   * @param port The port.
   * @return A string providing the value field of the port struct.
   */
  private static String valueDeclaration(
      TypeParameterizedReactor tpr,
      Port port,
      Target target,
      MessageReporter messageReporter,
      CTypes types) {
    if (port.getType() == null && target.requiresTypes) {
      // This should have been caught by the validator.
      String message = "Port is required to have a type: " + port.getName();
      messageReporter.at(port).error(message);
      return "";
    }
    // Do not convert to lf_token_t* using lfTypeToTokenType because there
    // will be a separate field pointing to the token.
    return types.getVariableDeclaration(tpr, ASTUtils.getInferredType(port), "value", false) + ";";
  }

  /**
   * Generate fields in the self struct for input ports
   *
   * <p>If the port is a multiport, the input field is an array of pointers that will be allocated
   * separately for each instance because the sizes may be different. Otherwise, it is a simple
   * pointer.
   */
  private static void generateInputDeclarations(
      TypeParameterizedReactor tpr, CTypes types, CodeBuilder body, CodeBuilder constructorCode) {
    for (Input input : ASTUtils.allInputs(tpr.reactor())) {
      var inputName = input.getName();
      if (ASTUtils.isMultiport(input)) {
        body.pr(
            String.join(
                "\n",
                "// Multiport input array will be malloc'd later.",
                variableStructType(input, tpr, false) + "** _lf_" + inputName + ";",
                "int _lf_" + inputName + "_width;",
                "// Default input (in case it does not get connected)",
                variableStructType(input, tpr, false) + " _lf_default__" + inputName + ";",
                "// Struct to support efficiently reading sparse inputs.",
                "lf_sparse_io_record_t* _lf_" + inputName + "__sparse;"));
      } else {
        // input is not a multiport.
        body.pr(
            String.join(
                "\n",
                variableStructType(input, tpr, false) + "* _lf_" + inputName + ";",
                "// width of -2 indicates that it is not a multiport.",
                "int _lf_" + inputName + "_width;",
                "// Default input (in case it does not get connected)",
                variableStructType(input, tpr, false) + " _lf_default__" + inputName + ";"));

        constructorCode.pr(
            String.join(
                "\n",
                "// Set input by default to an always absent default input.",
                "self->_lf_" + inputName + " = &self->_lf_default__" + inputName + ";"));
      }
      constructorCode.pr(
          String.join(
              "\n",
              "// Set the default source reactor pointer",
              "self->_lf_default__" + inputName + "._base.source_reactor = (self_base_t*)self;"));
      // Initialize element_size in the port struct.
      var rootType = CUtil.rootType(types.getTargetType(input));
      var size = (rootType.equals("void")) ? "0" : "sizeof(" + rootType + ")";
      constructorCode.pr("self->_lf_" + inputName + "->type.element_size = " + size + ";");
    }
  }

  /** Generate fields in the self struct for output ports */
  private static void generateOutputDeclarations(
      TypeParameterizedReactor tpr, CTypes types, CodeBuilder body, CodeBuilder constructorCode) {
    for (Output output : ASTUtils.allOutputs(tpr.reactor())) {
      // If the port is a multiport, create an array to be allocated
      // at instantiation.
      var outputName = output.getName();
      if (ASTUtils.isMultiport(output)) {
        body.pr(
            String.join(
                "\n",
                "// Array of output ports.",
                variableStructType(output, tpr, false) + "* _lf_" + outputName + ";",
                "int _lf_" + outputName + "_width;",
                "// An array of pointers to the individual ports. Useful",
                "// for the lf_set macros to work out-of-the-box for",
                "// multiports in the body of reactions because their ",
                "// value can be accessed via a -> operator (e.g.,foo[i]->value).",
                "// So we have to handle multiports specially here a construct that",
                "// array of pointers.",
                variableStructType(output, tpr, false) + "** _lf_" + outputName + "_pointers;"));
      } else {
        body.pr(
            String.join(
                "\n",
                variableStructType(output, tpr, false) + " _lf_" + outputName + ";",
                "int _lf_" + outputName + "_width;"));
      }
      constructorCode.pr(
          String.join(
              "\n",
              "// Set the default source reactor pointer",
              "self->_lf_" + outputName + "._base.source_reactor = (self_base_t*)self;"));
      // Initialize element_size in the port struct.
      var rootType = CUtil.rootType(types.getTargetType(output));
      var size = (rootType.equals("void")) ? "0" : "sizeof(" + rootType + ")";
      constructorCode.pr("self->_lf_" + outputName + ".type.element_size = " + size + ";");
    }
  }
}
