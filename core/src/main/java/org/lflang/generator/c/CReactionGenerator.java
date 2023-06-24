package org.lflang.generator.c;

import static org.lflang.generator.c.CUtil.generateWidthVariable;
import static org.lflang.util.StringUtil.addDoubleQuotes;

import java.util.LinkedHashMap;
import java.util.LinkedHashSet;
import java.util.LinkedList;
import java.util.List;
import java.util.Map;
import java.util.Set;
import org.lflang.InferredType;
import org.lflang.MessageReporter;
import org.lflang.TargetConfig;
import org.lflang.ast.ASTUtils;
import org.lflang.federated.extensions.CExtensionUtils;
import org.lflang.generator.CodeBuilder;
import org.lflang.lf.Action;
import org.lflang.lf.ActionOrigin;
import org.lflang.lf.BuiltinTriggerRef;
import org.lflang.lf.Code;
import org.lflang.lf.Input;
import org.lflang.lf.Instantiation;
import org.lflang.lf.LfFactory;
import org.lflang.lf.Mode;
import org.lflang.lf.ModeTransition;
import org.lflang.lf.Output;
import org.lflang.lf.Port;
import org.lflang.lf.Reaction;
import org.lflang.lf.Reactor;
import org.lflang.lf.Timer;
import org.lflang.lf.TriggerRef;
import org.lflang.lf.VarRef;
import org.lflang.lf.Variable;
import org.lflang.lf.Watchdog;
import org.lflang.util.StringUtil;

public class CReactionGenerator {
  protected static String DISABLE_REACTION_INITIALIZATION_MARKER =
      "// **** Do not include initialization code in this reaction."; // FIXME: Such markers should
  // not exist (#1687)

  /**
   * Generate necessary initialization code inside the body of the reaction that belongs to reactor
   * decl.
   *
   * @param body The body of the reaction. Used to check for the
   *     DISABLE_REACTION_INITIALIZATION_MARKER.
   * @param reaction The initialization code will be generated for this specific reaction
   * @param tpr The reactor that has the reaction
   * @param reactionIndex The index of the reaction relative to other reactions in the reactor,
   *     starting from 0
   */
  public static String generateInitializationForReaction(
      String body,
      Reaction reaction,
      TypeParameterizedReactor tpr,
      int reactionIndex,
      CTypes types,
      MessageReporter messageReporter,
      Instantiation mainDef,
      boolean requiresTypes) {
    // Construct the reactionInitialization code to go into
    // the body of the function before the verbatim code.
    CodeBuilder reactionInitialization = new CodeBuilder();

    CodeBuilder code = new CodeBuilder();

    // Define the "self" struct.
    String structType = CUtil.selfType(tpr);
    // A null structType means there are no inputs, state,
    // or anything else. No need to declare it.
    if (structType != null) {
      code.pr(
          String.join(
              "\n",
              structType
                  + "* self = ("
                  + structType
                  + "*)instance_args; SUPPRESS_UNUSED_WARNING(self);"));
    }

    // Do not generate the initialization code if the body is marked
    // to not generate it.
    if (body.startsWith(DISABLE_REACTION_INITIALIZATION_MARKER)) {
      return code.toString();
    }

    // A reaction may send to or receive from multiple ports of
    // a contained reactor. The variables for these ports need to
    // all be declared as fields of the same struct. Hence, we first
    // collect the fields to be defined in the structs and then
    // generate the structs.
    Map<Instantiation, CodeBuilder> fieldsForStructsForContainedReactors = new LinkedHashMap<>();

    // Actions may appear twice, first as a trigger, then with the outputs.
    // But we need to declare it only once. Collect in this data structure
    // the actions that are declared as triggered so that if they appear
    // again with the outputs, they are not defined a second time.
    // That second redefinition would trigger a compile error.
    Set<Action> actionsAsTriggers = new LinkedHashSet<>();

    // Next, add the triggers (input and actions; timers are not needed).
    // This defines a local variable in the reaction function whose
    // name matches that of the trigger. The value of the local variable
    // is a struct with a value and is_present field, the latter a boolean
    // that indicates whether the input/action is present.
    // If the trigger is an output, then it is an output of a
    // contained reactor. In this case, a struct with the name
    // of the contained reactor is created with one field that is
    // a pointer to a struct with a value and is_present field.
    // E.g., if the contained reactor is named 'c' and its output
    // port is named 'out', then c.out->value c.out->is_present are
    // defined so that they can be used in the verbatim code.
    for (TriggerRef trigger : ASTUtils.convertToEmptyListIfNull(reaction.getTriggers())) {
      if (trigger instanceof VarRef triggerAsVarRef) {
        if (triggerAsVarRef.getVariable() instanceof Port) {
          generatePortVariablesInReaction(
              reactionInitialization,
              fieldsForStructsForContainedReactors,
              triggerAsVarRef,
              tpr,
              types);
        } else if (triggerAsVarRef.getVariable() instanceof Action) {
          reactionInitialization.pr(
              generateActionVariablesInReaction(
                  (Action) triggerAsVarRef.getVariable(), tpr, types));
          actionsAsTriggers.add((Action) triggerAsVarRef.getVariable());
        }
      }
    }
    if (reaction.getTriggers() == null || reaction.getTriggers().size() == 0) {
      // No triggers are given, which means react to any input.
      // Declare an argument for every input.
      // NOTE: this does not include contained outputs.
      for (Input input : tpr.reactor().getInputs()) {
        reactionInitialization.pr(generateInputVariablesInReaction(input, tpr, types));
      }
    }
    // Define argument for non-triggering inputs.
    for (VarRef src : ASTUtils.convertToEmptyListIfNull(reaction.getSources())) {
      if (src.getVariable() instanceof Port) {
        generatePortVariablesInReaction(
            reactionInitialization, fieldsForStructsForContainedReactors, src, tpr, types);
      } else if (src.getVariable() instanceof Action) {
        // It's a bit odd to read but not be triggered by an action, but
        // OK, I guess we allow it.
        reactionInitialization.pr(
            generateActionVariablesInReaction((Action) src.getVariable(), tpr, types));
        actionsAsTriggers.add((Action) src.getVariable());
      }
    }

    // Define variables for each declared output or action.
    // In the case of outputs, the variable is a pointer to where the
    // output is stored. This gives the reaction code access to any previous
    // value that may have been written to that output in an earlier reaction.
    if (reaction.getEffects() != null) {
      for (VarRef effect : reaction.getEffects()) {
        Variable variable = effect.getVariable();
        if (variable instanceof Action) {
          // It is an action, not an output.
          // If it has already appeared as trigger, do not redefine it.
          if (!actionsAsTriggers.contains(effect.getVariable())) {
            reactionInitialization.pr(
                CGenerator.variableStructType(variable, tpr, false)
                    + "* "
                    + variable.getName()
                    + " = &self->_lf_"
                    + variable.getName()
                    + ";");
          }
        } else if (effect.getVariable() instanceof Mode) {
          // Mode change effect
          int idx = ASTUtils.allModes(tpr.reactor()).indexOf((Mode) effect.getVariable());
          String name = effect.getVariable().getName();
          if (idx >= 0) {
            reactionInitialization.pr(
                "reactor_mode_t* "
                    + name
                    + " = &self->_lf__modes["
                    + idx
                    + "];\n"
                    + "lf_mode_change_type_t _lf_"
                    + name
                    + "_change_type = "
                    + (effect.getTransition() == ModeTransition.HISTORY
                        ? "history_transition"
                        : "reset_transition")
                    + ";");
          } else {
            messageReporter
                .at(reaction)
                .error("In generateReaction(): " + name + " not a valid mode of this reactor.");
          }
        } else {
          if (variable instanceof Output) {
            reactionInitialization.pr(
                generateOutputVariablesInReaction(effect, tpr, messageReporter, requiresTypes));
          } else if (variable instanceof Input) {
            // It is the input of a contained reactor.
            generateVariablesForSendingToContainedReactors(
                reactionInitialization,
                fieldsForStructsForContainedReactors,
                effect.getContainer(),
                (Input) variable,
                tpr);
          } else if (variable instanceof Watchdog) {
            reactionInitialization.pr(generateWatchdogVariablesInReaction(effect));
          } else {
            messageReporter
                .at(reaction)
                .error("In generateReaction(): effect is not an input, output, or watchdog.");
          }
        }
      }
    }
    // Before the reaction initialization,
    // generate the structs used for communication to and from contained reactors.
    for (Instantiation containedReactor : fieldsForStructsForContainedReactors.keySet()) {
      String array = "";
      if (containedReactor.getWidthSpec() != null) {
        String containedReactorWidthVar = generateWidthVariable(containedReactor.getName());
        code.pr(
            "int " + containedReactorWidthVar + " = self->_lf_" + containedReactorWidthVar + ";");
        // Windows does not support variables in arrays declared on the stack,
        // so we use the maximum size over all bank members.
        array = "[" + maxContainedReactorBankWidth(containedReactor, null, 0, mainDef) + "]";
      }
      code.pr(
          String.join(
              "\n",
              "struct " + containedReactor.getName() + " {",
              "    " + fieldsForStructsForContainedReactors.get(containedReactor),
              "} " + containedReactor.getName() + array + ";"));
    }
    // Next generate all the collected setup code.
    code.pr(reactionInitialization.toString());
    return code.toString();
  }

  /**
   * Return the maximum bank width for the given instantiation within all instantiations of its
   * parent reactor. On the first call to this method, the breadcrumbs should be null and the max
   * argument should be zero. On recursive calls, breadcrumbs is a list of nested instantiations,
   * the max is the maximum width found so far. The search for instances of the parent reactor will
   * begin with the last instantiation in the specified list.
   *
   * <p>This rather complicated method is used when a reaction sends or receives data to or from a
   * bank of contained reactors. There will be an array of structs on the self struct of the parent,
   * and the size of the array is conservatively set to the maximum of all the identified bank
   * widths. This is a bit wasteful of memory, but it avoids having to malloc the array for each
   * instance, and in typical usage, there will be few instances or instances that are all the same
   * width.
   *
   * @param containedReactor The contained reactor instantiation.
   * @param breadcrumbs null on first call (non-recursive).
   * @param max 0 on first call.
   */
  public static int maxContainedReactorBankWidth(
      Instantiation containedReactor,
      LinkedList<Instantiation> breadcrumbs,
      int max,
      Instantiation mainDef) {
    // If the instantiation is not a bank, return 1.
    if (containedReactor.getWidthSpec() == null) {
      return 1;
    }
    // If there is no main, then we just use the default width.
    if (mainDef == null) {
      return ASTUtils.width(containedReactor.getWidthSpec(), null);
    }
    LinkedList<Instantiation> nestedBreadcrumbs = breadcrumbs;
    if (nestedBreadcrumbs == null) {
      nestedBreadcrumbs = new LinkedList<>();
      nestedBreadcrumbs.add(mainDef);
    }
    int result = max;
    Reactor parent = (Reactor) containedReactor.eContainer();
    if (parent == ASTUtils.toDefinition(mainDef.getReactorClass())) {
      // The parent is main, so there can't be any other instantiations of it.
      return ASTUtils.width(containedReactor.getWidthSpec(), null);
    }
    // Search for instances of the parent within the tail of the breadcrumbs list.
    Reactor container = ASTUtils.toDefinition(nestedBreadcrumbs.get(0).getReactorClass());
    for (Instantiation instantiation : container.getInstantiations()) {
      // Put this new instantiation at the head of the list.
      nestedBreadcrumbs.add(0, instantiation);
      if (ASTUtils.toDefinition(instantiation.getReactorClass()) == parent) {
        // Found a matching instantiation of the parent.
        // Evaluate the original width specification in this context.
        int candidate = ASTUtils.width(containedReactor.getWidthSpec(), nestedBreadcrumbs);
        if (candidate > result) {
          result = candidate;
        }
      } else {
        // Found some other instantiation, not the parent.
        // Search within it for instantiations of the parent.
        // Note that we assume here that the parent cannot contain
        // instances of itself.
        int candidate =
            maxContainedReactorBankWidth(containedReactor, nestedBreadcrumbs, result, mainDef);
        if (candidate > result) {
          result = candidate;
        }
      }
      nestedBreadcrumbs.remove();
    }
    return result;
  }

  /**
   * Generate code for the body of a reaction that takes an input and schedules an action with the
   * value of that input.
   *
   * @param actionName The action to schedule
   */
  public static String generateDelayBody(String ref, String actionName, boolean isTokenType) {
    // Note that the action.type set by the base class is actually
    // the port type.
    return isTokenType
        ? String.join(
            "\n",
            "if (" + ref + "->is_present) {",
            "    // Put the whole token on the event queue, not just the payload.",
            "    // This way, the length and element_size are transported.",
            "    lf_schedule_token(" + actionName + ", 0, " + ref + "->token);",
            "}")
        : "lf_schedule_copy(" + actionName + ", 0, &" + ref + "->value, 1);  // Length is 1.";
  }

  public static String generateForwardBody(
      String outputName, String targetType, String actionName, boolean isTokenType) {
    return isTokenType
        ? String.join(
            "\n",
            DISABLE_REACTION_INITIALIZATION_MARKER,
            "self->_lf_"
                + outputName
                + ".value = ("
                + targetType
                + ")self->_lf__"
                + actionName
                + ".tmplt.token->value;",
            "_lf_replace_template_token((token_template_t*)&self->_lf_"
                + outputName
                + ", (lf_token_t*)self->_lf__"
                + actionName
                + ".tmplt.token);",
            "self->_lf_" + outputName + ".is_present = true;")
        : "lf_set(" + outputName + ", " + actionName + "->value);";
  }

  /**
   * Generate into the specified string builder the code to initialize local variables for sending
   * data to an input of a contained reactor. This will also, if necessary, generate entries for
   * local struct definitions into the struct argument. These entries point to where the data is
   * stored.
   *
   * @param builder The string builder.
   * @param structs A map from reactor instantiations to a place to write struct fields.
   * @param definition AST node defining the reactor within which this occurs
   * @param input Input of the contained reactor.
   */
  private static void generateVariablesForSendingToContainedReactors(
      CodeBuilder builder,
      Map<Instantiation, CodeBuilder> structs,
      Instantiation definition,
      Input input,
      TypeParameterizedReactor container) {
    CodeBuilder structBuilder = structs.get(definition);
    if (structBuilder == null) {
      structBuilder = new CodeBuilder();
      structs.put(definition, structBuilder);
    }
    String inputStructType =
        CGenerator.variableStructType(
            input, new TypeParameterizedReactor(definition, container), false);
    String defName = definition.getName();
    String defWidth = generateWidthVariable(defName);
    String inputName = input.getName();
    String inputWidth = generateWidthVariable(inputName);
    if (!ASTUtils.isMultiport(input)) {
      // Contained reactor's input is not a multiport.
      structBuilder.pr(inputStructType + "* " + inputName + ";");
      if (definition.getWidthSpec() != null) {
        // Contained reactor is a bank.
        builder.pr(
            String.join(
                "\n",
                "for (int bankIndex = 0; bankIndex < self->_lf_" + defWidth + "; bankIndex++) {",
                "    "
                    + defName
                    + "[bankIndex]."
                    + inputName
                    + " = &(self->_lf_"
                    + defName
                    + "[bankIndex]."
                    + inputName
                    + ");",
                "}"));
      } else {
        // Contained reactor is not a bank.
        builder.pr(
            defName + "." + inputName + " = &(self->_lf_" + defName + "." + inputName + ");");
      }
    } else {
      // Contained reactor's input is a multiport.
      structBuilder.pr(
          String.join("\n", inputStructType + "** " + inputName + ";", "int " + inputWidth + ";"));
      // If the contained reactor is a bank, then we have to set the
      // pointer for each element of the bank.
      if (definition.getWidthSpec() != null) {
        builder.pr(
            String.join(
                "\n",
                "for (int _i = 0; _i < self->_lf_" + defWidth + "; _i++) {",
                "    "
                    + defName
                    + "[_i]."
                    + inputName
                    + " = self->_lf_"
                    + defName
                    + "[_i]."
                    + inputName
                    + ";",
                "    "
                    + defName
                    + "[_i]."
                    + inputWidth
                    + " = self->_lf_"
                    + defName
                    + "[_i]."
                    + inputWidth
                    + ";",
                "}"));
      } else {
        builder.pr(
            String.join(
                "\n",
                defName + "." + inputName + " = self->_lf_" + defName + "." + inputName + ";",
                defName + "." + inputWidth + " = self->_lf_" + defName + "." + inputWidth + ";"));
      }
    }
  }

  /**
   * Generate into the specified string builder the code to initialize local variables for ports in
   * a reaction function from the "self" struct. The port may be an input of the reactor or an
   * output of a contained reactor. The second argument provides, for each contained reactor, a
   * place to write the declaration of the output of that reactor that is triggering reactions.
   *
   * @param builder The place into which to write the code.
   * @param structs A map from reactor instantiations to a place to write struct fields.
   */
  private static void generatePortVariablesInReaction(
      CodeBuilder builder,
      Map<Instantiation, CodeBuilder> structs,
      VarRef port,
      TypeParameterizedReactor tpr,
      CTypes types) {
    if (port.getVariable() instanceof Input) {
      builder.pr(generateInputVariablesInReaction((Input) port.getVariable(), tpr, types));
    } else {
      // port is an output of a contained reactor.
      Output output = (Output) port.getVariable();
      String portStructType =
          CGenerator.variableStructType(
              output, new TypeParameterizedReactor(port.getContainer(), tpr), false);

      CodeBuilder structBuilder = structs.get(port.getContainer());
      if (structBuilder == null) {
        structBuilder = new CodeBuilder();
        structs.put(port.getContainer(), structBuilder);
      }
      String subName = port.getContainer().getName();
      String reactorWidth = generateWidthVariable(subName);
      String outputName = output.getName();
      String outputWidth = generateWidthVariable(outputName);
      // First define the struct containing the output value and indicator
      // of its presence.
      if (!ASTUtils.isMultiport(output)) {
        // Output is not a multiport.
        structBuilder.pr(portStructType + "* " + outputName + ";");
      } else {
        // Output is a multiport.
        structBuilder.pr(
            String.join(
                "\n", portStructType + "** " + outputName + ";", "int " + outputWidth + ";"));
      }

      // Next, initialize the struct with the current values.
      if (port.getContainer().getWidthSpec() != null) {
        // Output is in a bank.
        builder.pr(
            String.join(
                "\n",
                "for (int i = 0; i < " + reactorWidth + "; i++) {",
                "    "
                    + subName
                    + "[i]."
                    + outputName
                    + " = self->_lf_"
                    + subName
                    + "[i]."
                    + outputName
                    + ";",
                "}"));
        if (ASTUtils.isMultiport(output)) {
          builder.pr(
              String.join(
                  "\n",
                  "for (int i = 0; i < " + reactorWidth + "; i++) {",
                  "    "
                      + subName
                      + "[i]."
                      + outputWidth
                      + " = self->_lf_"
                      + subName
                      + "[i]."
                      + outputWidth
                      + ";",
                  "}"));
        }
      } else {
        // Output is not in a bank.
        builder.pr(subName + "." + outputName + " = self->_lf_" + subName + "." + outputName + ";");
        if (ASTUtils.isMultiport(output)) {
          builder.pr(
              subName + "." + outputWidth + " = self->_lf_" + subName + "." + outputWidth + ";");
        }
      }
    }
  }

  /**
   * Generate action variables for a reaction.
   *
   * @param action The action.
   */
  private static String generateActionVariablesInReaction(
      Action action, TypeParameterizedReactor tpr, CTypes types) {
    String structType = CGenerator.variableStructType(action, tpr, false);
    // If the action has a type, create variables for accessing the value.
    InferredType type = ASTUtils.getInferredType(action);
    // Pointer to the lf_token_t sent as the payload in the trigger.
    String tokenPointer = "(self->_lf__" + action.getName() + ".tmplt.token)";
    CodeBuilder builder = new CodeBuilder();

    builder.pr(
        String.join(
            "\n",
            "// Expose the action struct as a local variable whose name matches the action name.",
            structType + "* " + action.getName() + " = &self->_lf_" + action.getName() + ";",
            "// Set the fields of the action struct to match the current trigger.",
            action.getName() + "->is_present = (bool)self->_lf__" + action.getName() + ".status;",
            action.getName()
                + "->has_value = ("
                + tokenPointer
                + " != NULL && "
                + tokenPointer
                + "->value != NULL);",
            "_lf_replace_template_token((token_template_t*)"
                + action.getName()
                + ", "
                + tokenPointer
                + ");"));
    // Set the value field only if there is a type.
    if (!type.isUndefined()) {
      // The value field will either be a copy (for primitive types)
      // or a pointer (for types ending in *).
      builder.pr("if (" + action.getName() + "->has_value) {");
      builder.indent();
      if (CUtil.isTokenType(type, types)) {
        builder.pr(
            action.getName()
                + "->value = ("
                + types.getTargetType(type)
                + ")"
                + tokenPointer
                + "->value;");
      } else {
        builder.pr(
            action.getName()
                + "->value = *("
                + types.getTargetType(type)
                + "*)"
                + tokenPointer
                + "->value;");
      }
      builder.unindent();
      builder.pr("}");
    }
    return builder.toString();
  }

  /**
   * Generate into the specified string builder the code to initialize local variables for the
   * specified input port in a reaction function from the "self" struct.
   *
   * @param input The input statement from the AST.
   * @param tpr The reactor.
   */
  private static String generateInputVariablesInReaction(
      Input input, TypeParameterizedReactor tpr, CTypes types) {
    String structType = CGenerator.variableStructType(input, tpr, false);
    InferredType inputType = ASTUtils.getInferredType(input);
    CodeBuilder builder = new CodeBuilder();
    String inputName = input.getName();
    String inputWidth = generateWidthVariable(inputName);

    // Create the local variable whose name matches the input name.
    // If the input has not been declared mutable, then this is a pointer
    // to the upstream output. Otherwise, it is a copy of the upstream output,
    // which nevertheless points to the same token and value (hence, as done
    // below, we have to use lf_writable_copy()). There are 8 cases,
    // depending on whether the input is mutable, whether it is a multiport,
    // and whether it is a token type.
    // Easy case first.
    if (!input.isMutable()
        && !CUtil.isTokenType(inputType, types)
        && !ASTUtils.isMultiport(input)) {
      // Non-mutable, non-multiport, primitive type.
      builder.pr(structType + "* " + inputName + " = self->_lf_" + inputName + ";");
    } else if (input.isMutable()
        && !CUtil.isTokenType(inputType, types)
        && !ASTUtils.isMultiport(input)) {
      // Mutable, non-multiport, primitive type.
      builder.pr(
          String.join(
              "\n",
              "// Mutable input, so copy the input into a temporary variable.",
              "// The input value on the struct is a copy.",
              structType + " _lf_tmp_" + inputName + " = *(self->_lf_" + inputName + ");",
              structType + "* " + inputName + " = &_lf_tmp_" + inputName + ";"));
    } else if (!input.isMutable()
        && CUtil.isTokenType(inputType, types)
        && !ASTUtils.isMultiport(input)) {
      // Non-mutable, non-multiport, token type.
      builder.pr(
          String.join(
              "\n",
              structType + "* " + inputName + " = self->_lf_" + inputName + ";",
              "if (" + inputName + "->is_present) {",
              "    " + inputName + "->length = " + inputName + "->token->length;",
              "    "
                  + inputName
                  + "->value = ("
                  + types.getTargetType(inputType)
                  + ")"
                  + inputName
                  + "->token->value;",
              "} else {",
              "    " + inputName + "->length = 0;",
              "}"));
    } else if (input.isMutable()
        && CUtil.isTokenType(inputType, types)
        && !ASTUtils.isMultiport(input)) {
      // Mutable, non-multiport, token type.
      builder.pr(
          String.join(
              "\n",
              "// Mutable input, so copy the input struct into a temporary variable.",
              structType + " _lf_tmp_" + inputName + " = *(self->_lf_" + inputName + ");",
              structType + "* " + inputName + " = &_lf_tmp_" + inputName + ";",
              inputName + "->value = NULL;", // Prevent payload from being freed.
              "if (" + inputName + "->is_present) {",
              "    " + inputName + "->length = " + inputName + "->token->length;",
              "    "
                  + inputName
                  + "->token = lf_writable_copy((lf_port_base_t*)self->_lf_"
                  + inputName
                  + ");",
              "    "
                  + inputName
                  + "->value = ("
                  + types.getTargetType(inputType)
                  + ")"
                  + inputName
                  + "->token->value;",
              "} else {",
              "    " + inputName + "->length = 0;",
              "}"));
    } else if (!input.isMutable() && ASTUtils.isMultiport(input)) {
      // Non-mutable, multiport, primitive or token type.
      builder.pr(structType + "** " + inputName + " = self->_lf_" + inputName + ";");
    } else if (CUtil.isTokenType(inputType, types)) {
      // Mutable, multiport, token type
      builder.pr(
          String.join(
              "\n",
              "// Mutable multiport input, so copy the input structs",
              "// into an array of temporary variables on the stack.",
              structType
                  + " _lf_tmp_"
                  + inputName
                  + "["
                  + CUtil.multiportWidthExpression(input)
                  + "];",
              structType + "* " + inputName + "[" + CUtil.multiportWidthExpression(input) + "];",
              "for (int i = 0; i < " + CUtil.multiportWidthExpression(input) + "; i++) {",
              "    " + inputName + "[i] = &_lf_tmp_" + inputName + "[i];",
              "    _lf_tmp_" + inputName + "[i] = *(self->_lf_" + inputName + "[i]);",
              "    // If necessary, copy the tokens.",
              "    if (" + inputName + "[i]->is_present) {",
              "        " + inputName + "[i]->length = " + inputName + "[i]->token->length;",
              "        token_template_t* _lf_input = (token_template_t*)self->_lf_"
                  + inputName
                  + "[i];",
              "        "
                  + inputName
                  + "[i]->token = lf_writable_copy("
                  + " (lf_port_base_t*)_lf_input);",
              "        "
                  + inputName
                  + "[i]->value = ("
                  + types.getTargetType(inputType)
                  + ")"
                  + inputName
                  + "[i]->token->value;",
              "    } else {",
              "        " + inputName + "[i]->length = 0;",
              "    }",
              "}"));
    } else {
      // Mutable, multiport, primitive type
      builder.pr(
          String.join(
              "\n",
              "// Mutable multiport input, so copy the input structs",
              "// into an array of temporary variables on the stack.",
              structType
                  + " _lf_tmp_"
                  + inputName
                  + "["
                  + CUtil.multiportWidthExpression(input)
                  + "];",
              structType + "* " + inputName + "[" + CUtil.multiportWidthExpression(input) + "];",
              "for (int i = 0; i < " + CUtil.multiportWidthExpression(input) + "; i++) {",
              "    " + inputName + "[i]  = &_lf_tmp_" + inputName + "[i];",
              "    // Copy the struct, which includes the value.",
              "    _lf_tmp_" + inputName + "[i] = *(self->_lf_" + inputName + "[i]);",
              "}"));
    }
    // Set the _width variable for all cases. This will be -1
    // for a variable-width multiport, which is not currently supported.
    // It will be -2 if it is not multiport.
    builder.pr(
        "int "
            + inputWidth
            + " = self->_lf_"
            + inputWidth
            + "; SUPPRESS_UNUSED_WARNING("
            + inputWidth
            + ");");
    return builder.toString();
  }

  /**
   * Generate into the specified string builder the code to initialize local variables for outputs
   * in a reaction function from the "self" struct.
   *
   * @param effect The effect declared by the reaction. This must refer to an output.
   * @param tpr The reactor containing the reaction.
   */
  public static String generateOutputVariablesInReaction(
      VarRef effect,
      TypeParameterizedReactor tpr,
      MessageReporter messageReporter,
      boolean requiresTypes) {
    Output output = (Output) effect.getVariable();
    String outputName = output.getName();
    String outputWidth = generateWidthVariable(outputName);
    if (output.getType() == null && requiresTypes) {
      messageReporter.at(output).error("Output is required to have a type: " + outputName);
      return "";
    } else {
      // The container of the output may be a contained reactor or
      // the reactor containing the reaction.
      String outputStructType =
          (effect.getContainer() == null)
              ? CGenerator.variableStructType(output, tpr, false)
              : CGenerator.variableStructType(
                  output, new TypeParameterizedReactor(effect.getContainer(), tpr), false);
      if (!ASTUtils.isMultiport(output)) {
        // Output port is not a multiport.
        return outputStructType + "* " + outputName + " = &self->_lf_" + outputName + ";";
      } else {
        // Output port is a multiport.
        // Set the _width variable.
        return String.join(
            "\n",
            "int "
                + outputWidth
                + " = self->_lf_"
                + outputWidth
                + "; SUPPRESS_UNUSED_WARNING("
                + outputWidth
                + ");",
            outputStructType + "** " + outputName + " = self->_lf_" + outputName + "_pointers;");
      }
    }
  }

  /**
   * Generate into the specified string builder the code to initialize local variables for watchdogs
   * in a reaction function from the "self" struct.
   *
   * @param effect The effect declared by the reaction. This must refer to a watchdog.
   */
  public static String generateWatchdogVariablesInReaction(VarRef effect) {
    Watchdog watchdog = (Watchdog) effect.getVariable();
    String watchdogName = watchdog.getName();
    return String.join(
        "\n",
        List.of("watchdog_t* " + watchdogName + " = &(self->_lf_watchdog_" + watchdogName + ");"));
  }

  /**
   * Generate the fields of the self struct and statements for the constructor to create and
   * initialize a reaction_t struct for each reaction in the specified reactor and a trigger_t
   * struct for each trigger (input, action, timer, or output of a contained reactor).
   *
   * @param body The place to put the code for the self struct.
   * @param tpr {@link TypeParameterizedReactor}
   * @param constructorCode The place to put the constructor code.
   */
  public static void generateReactionAndTriggerStructs(
      CodeBuilder body, TypeParameterizedReactor tpr, CodeBuilder constructorCode, CTypes types) {
    var reactionCount = 0;
    // Iterate over reactions and create initialize the reaction_t struct
    // on the self struct. Also, collect a map from triggers to the reactions
    // that are triggered by that trigger. Also, collect a set of sources
    // that are read by reactions but do not trigger reactions.
    // Finally, collect a set of triggers and sources that are outputs
    // of contained reactors.
    var triggerMap = new LinkedHashMap<Variable, LinkedList<Integer>>();
    var sourceSet = new LinkedHashSet<Variable>();
    var outputsOfContainedReactors = new LinkedHashMap<Variable, Instantiation>();
    var startupReactions = new LinkedHashSet<Integer>();
    var shutdownReactions = new LinkedHashSet<Integer>();
    var resetReactions = new LinkedHashSet<Integer>();
    for (Reaction reaction : ASTUtils.allReactions(tpr.reactor())) {
      // Create the reaction_t struct.
      body.pr(reaction, "reaction_t _lf__reaction_" + reactionCount + ";");

      // Create the map of triggers to reactions.
      for (TriggerRef trigger : reaction.getTriggers()) {
        // trigger may not be a VarRef (it could be "startup" or "shutdown").
        if (trigger instanceof VarRef triggerAsVarRef) {
          var reactionList = triggerMap.get(triggerAsVarRef.getVariable());
          if (reactionList == null) {
            reactionList = new LinkedList<>();
            triggerMap.put(triggerAsVarRef.getVariable(), reactionList);
          }
          reactionList.add(reactionCount);
          if (triggerAsVarRef.getContainer() != null) {
            outputsOfContainedReactors.put(
                triggerAsVarRef.getVariable(), triggerAsVarRef.getContainer());
          }
        } else if (trigger instanceof BuiltinTriggerRef) {
          switch (((BuiltinTriggerRef) trigger).getType()) {
            case STARTUP:
              startupReactions.add(reactionCount);
              break;
            case SHUTDOWN:
              shutdownReactions.add(reactionCount);
              break;
            case RESET:
              resetReactions.add(reactionCount);
              break;
          }
        }
      }
      // Create the set of sources read but not triggering.
      for (VarRef source : reaction.getSources()) {
        sourceSet.add(source.getVariable());
        if (source.getContainer() != null) {
          outputsOfContainedReactors.put(source.getVariable(), source.getContainer());
        }
      }

      var deadlineFunctionPointer = "NULL";
      if (reaction.getDeadline() != null) {
        // The following has to match the name chosen in generateReactions
        var deadlineFunctionName = generateDeadlineFunctionName(tpr, reactionCount);
        deadlineFunctionPointer = "&" + deadlineFunctionName;
      }

      // Assign the STP handler
      var STPFunctionPointer = "NULL";
      if (reaction.getStp() != null) {
        // The following has to match the name chosen in generateReactions
        var STPFunctionName = generateStpFunctionName(tpr, reactionCount);
        STPFunctionPointer = "&" + STPFunctionName;
      }

      // Set the defaults of the reaction_t struct in the constructor.
      // Since the self struct is allocated using calloc, there is no need to set:
      // self->_lf__reaction_"+reactionCount+".index = 0;
      // self->_lf__reaction_"+reactionCount+".chain_id = 0;
      // self->_lf__reaction_"+reactionCount+".pos = 0;
      // self->_lf__reaction_"+reactionCount+".status = inactive;
      // self->_lf__reaction_"+reactionCount+".deadline = 0LL;
      // self->_lf__reaction_"+reactionCount+".is_STP_violated = false;
      constructorCode.pr(
          reaction,
          String.join(
              "\n",
              "self->_lf__reaction_" + reactionCount + ".number = " + reactionCount + ";",
              "self->_lf__reaction_"
                  + reactionCount
                  + ".function = "
                  + CReactionGenerator.generateReactionFunctionName(tpr, reactionCount)
                  + ";",
              "self->_lf__reaction_" + reactionCount + ".self = self;",
              "self->_lf__reaction_"
                  + reactionCount
                  + ".deadline_violation_handler = "
                  + deadlineFunctionPointer
                  + ";",
              "self->_lf__reaction_" + reactionCount + ".STP_handler = " + STPFunctionPointer + ";",
              "self->_lf__reaction_" + reactionCount + ".name = " + addDoubleQuotes("?") + ";",
              reaction.eContainer() instanceof Mode
                  ? "self->_lf__reaction_"
                      + reactionCount
                      + ".mode = &self->_lf__modes["
                      + tpr.reactor().getModes().indexOf((Mode) reaction.eContainer())
                      + "];"
                  : "self->_lf__reaction_" + reactionCount + ".mode = NULL;"));
      // Increment the reactionCount even if the reaction is not in the federate
      // so that reaction indices are consistent across federates.
      reactionCount++;
    }

    // Next, create and initialize the trigger_t objects.
    // Start with the timers.
    for (Timer timer : ASTUtils.allTimers(tpr.reactor())) {
      createTriggerT(body, timer, triggerMap, constructorCode, types);
      // Since the self struct is allocated using calloc, there is no need to set falsy fields.
      constructorCode.pr("self->_lf__" + timer.getName() + ".is_timer = true;");
      constructorCode.pr(
          CExtensionUtils.surroundWithIfFederatedDecentralized(
              "self->_lf__"
                  + timer.getName()
                  + ".intended_tag = (tag_t) { .time = NEVER, .microstep = 0u};"));
    }

    // Handle builtin triggers.
    if (startupReactions.size() > 0) {
      generateBuiltinTriggeredReactionsArray(startupReactions, "startup", body, constructorCode);
    }
    // Handle shutdown triggers.
    if (shutdownReactions.size() > 0) {
      generateBuiltinTriggeredReactionsArray(shutdownReactions, "shutdown", body, constructorCode);
    }
    if (resetReactions.size() > 0) {
      generateBuiltinTriggeredReactionsArray(resetReactions, "reset", body, constructorCode);
    }

    // Next handle actions.
    for (Action action : ASTUtils.allActions(tpr.reactor())) {
      createTriggerT(body, action, triggerMap, constructorCode, types);
      var isPhysical = "true";
      if (action.getOrigin().equals(ActionOrigin.LOGICAL)) {
        isPhysical = "false";
      }
      var elementSize = "0";
      // If the action type is 'void', we need to avoid generating the code
      // 'sizeof(void)', which some compilers reject.
      var rootType = action.getType() != null ? CUtil.rootType(types.getTargetType(action)) : null;
      if (rootType != null && !rootType.equals("void")) {
        elementSize = "sizeof(" + rootType + ")";
      }

      // Since the self struct is allocated using calloc, there is no need to set:
      // self->_lf__"+action.getName()+".is_timer = false;
      constructorCode.pr(
          String.join(
              "\n",
              "self->_lf__" + action.getName() + ".is_physical = " + isPhysical + ";",
              !(action.getPolicy() == null || action.getPolicy().isEmpty())
                  ? "self->_lf__" + action.getName() + ".policy = " + action.getPolicy() + ";"
                  : "",
              // Need to set the element_size in the trigger_t and the action struct.
              "self->_lf__" + action.getName() + ".tmplt.type.element_size = " + elementSize + ";",
              "self->_lf_" + action.getName() + ".type.element_size = " + elementSize + ";"));
    }

    // Next handle inputs.
    for (Input input : ASTUtils.allInputs(tpr.reactor())) {
      createTriggerT(body, input, triggerMap, constructorCode, types);
    }

    // Next handle watchdogs.
    for (Watchdog watchdog : ASTUtils.allWatchdogs(tpr.reactor())) {
      createTriggerT(body, watchdog, triggerMap, constructorCode, types);
    }
  }

  /**
   * Define the trigger_t object on the self struct, an array of reaction_t pointers pointing to
   * reactions triggered by this variable, and initialize the pointers in the array in the
   * constructor.
   *
   * @param body The place to write the self struct entries.
   * @param variable The trigger variable (Timer, Action, Watchdog, or Input).
   * @param triggerMap A map from Variables to a list of the reaction indices triggered by the
   *     variable.
   * @param constructorCode The place to write the constructor code.
   */
  private static void createTriggerT(
      CodeBuilder body,
      Variable variable,
      LinkedHashMap<Variable, LinkedList<Integer>> triggerMap,
      CodeBuilder constructorCode,
      CTypes types) {
    var varName = variable.getName();
    // variable is a port, a timer, or an action.
    body.pr(variable, "trigger_t _lf__" + varName + ";");
    constructorCode.pr(variable, "self->_lf__" + varName + ".last = NULL;");
    constructorCode.pr(
        variable,
        CExtensionUtils.surroundWithIfFederatedDecentralized(
            "self->_lf__"
                + varName
                + ".intended_tag = (tag_t) { .time = NEVER, .microstep = 0u};"));

    // Generate the reactions triggered table.
    var reactionsTriggered = triggerMap.get(variable);
    if (reactionsTriggered != null) {
      body.pr(
          variable,
          "reaction_t* _lf__" + varName + "_reactions[" + reactionsTriggered.size() + "];");
      var count = 0;
      for (Integer reactionTriggered : reactionsTriggered) {
        constructorCode.prSourceLineNumber(variable);
        constructorCode.pr(
            variable,
            "self->_lf__"
                + varName
                + "_reactions["
                + count
                + "] = &self->_lf__reaction_"
                + reactionTriggered
                + ";");
        count++;
      }
      // Set up the trigger_t struct's pointer to the reactions.
      constructorCode.pr(
          variable,
          String.join(
              "\n",
              "self->_lf__" + varName + ".reactions = &self->_lf__" + varName + "_reactions[0];",
              "self->_lf__" + varName + ".number_of_reactions = " + count + ";"));

      // If federated, set the physical_time_of_arrival
      constructorCode.pr(
          variable,
          CExtensionUtils.surroundWithIfFederated(
              "self->_lf__" + varName + ".physical_time_of_arrival = NEVER;"));
    }
    if (variable instanceof Input) {
      var rootType = CUtil.rootType(types.getTargetType((Input) variable));
      // Since the self struct is allocated using calloc, there is no need to set falsy fields.
      // If the input type is 'void', we need to avoid generating the code
      // 'sizeof(void)', which some compilers reject.
      var size = (rootType.equals("void")) ? "0" : "sizeof(" + rootType + ")";

      constructorCode.pr("self->_lf__" + varName + ".tmplt.type.element_size = " + size + ";");
      body.pr(
          CExtensionUtils.surroundWithIfFederated(
              CExtensionUtils.createPortStatusFieldForInput((Input) variable)));
    }
  }

  public static void generateBuiltinTriggeredReactionsArray(
      Set<Integer> reactions, String name, CodeBuilder body, CodeBuilder constructorCode) {
    body.pr(
        String.join(
            "\n",
            "trigger_t _lf__" + name + ";",
            "reaction_t* _lf__" + name + "_reactions[" + reactions.size() + "];"));
    constructorCode.pr(
        CExtensionUtils.surroundWithIfFederatedDecentralized(
            "self->_lf__" + name + ".intended_tag = (tag_t) { .time = NEVER, .microstep = 0u};"));
    var i = 0;
    for (Integer reactionIndex : reactions) {
      constructorCode.pr(
          "self->_lf__"
              + name
              + "_reactions["
              + i++
              + "] = &self->_lf__reaction_"
              + reactionIndex
              + ";");
    }
    constructorCode.pr(
        String.join(
            "\n",
            "self->_lf__" + name + ".last = NULL;",
            "self->_lf__" + name + ".reactions = &self->_lf__" + name + "_reactions[0];",
            "self->_lf__" + name + ".number_of_reactions = " + reactions.size() + ";",
            "self->_lf__" + name + ".is_timer = false;"));
  }

  /**
   * Generate a reaction function definition for a reactor. This function will have a single
   * argument that is a void* pointing to a struct that contains parameters, state variables, inputs
   * (triggering or not), actions (triggering or produced), and outputs.
   *
   * @param reaction The reaction.
   * @param tpr The reactor.
   * @param reactionIndex The position of the reaction within the reactor.
   */
  public static String generateReaction(
      Reaction reaction,
      TypeParameterizedReactor tpr,
      int reactionIndex,
      Instantiation mainDef,
      MessageReporter messageReporter,
      CTypes types,
      TargetConfig targetConfig,
      boolean requiresType) {
    var code = new CodeBuilder();
    var body = ASTUtils.toText(getCode(types, reaction, tpr));
    String init =
        generateInitializationForReaction(
            body, reaction, tpr, reactionIndex, types, messageReporter, mainDef, requiresType);

    code.pr("#include " + StringUtil.addDoubleQuotes(CCoreFilesUtils.getCTargetSetHeader()));

    CMethodGenerator.generateMacrosForMethods(tpr, code);
    code.pr(
        generateFunction(
            generateReactionFunctionHeader(tpr, reactionIndex),
            init,
            getCode(types, reaction, tpr)));
    // Now generate code for the late function, if there is one
    // Note that this function can only be defined on reactions
    // in federates that have inputs from a logical connection.
    if (reaction.getStp() != null) {
      code.pr(
          generateFunction(
              generateStpFunctionHeader(tpr, reactionIndex), init, reaction.getStp().getCode()));
    }

    // Now generate code for the deadline violation function, if there is one.
    if (reaction.getDeadline() != null) {
      code.pr(
          generateFunction(
              generateDeadlineFunctionHeader(tpr, reactionIndex),
              init,
              reaction.getDeadline().getCode()));
    }
    CMethodGenerator.generateMacroUndefsForMethods(tpr.reactor(), code);
    code.pr("#include " + StringUtil.addDoubleQuotes(CCoreFilesUtils.getCTargetSetUndefHeader()));
    return code.toString();
  }

  private static Code getCode(CTypes types, Reaction r, TypeParameterizedReactor tpr) {
    if (r.getCode() != null) return r.getCode();
    Code ret = LfFactory.eINSTANCE.createCode();
    ret.setBody(
        CReactorHeaderFileGenerator.nonInlineInitialization(r, tpr)
            + "\n"
            + r.getName()
            + "( "
            + CReactorHeaderFileGenerator.reactionArguments(r, tpr)
            + " );");
    return ret;
  }

  public static String generateFunction(String header, String init, Code code) {
    var function = new CodeBuilder();
    function.pr(header + " {");
    function.indent();
    function.pr(init);
    function.prSourceLineNumber(code);
    function.pr(ASTUtils.toText(code));
    function.unindent();
    function.pr("}");
    return function.toString();
  }

  /**
   * Returns the name of the deadline function for reaction.
   *
   * @param tpr The reactor with the deadline
   * @param reactionIndex The number assigned to this reaction deadline
   */
  public static String generateDeadlineFunctionName(
      TypeParameterizedReactor tpr, int reactionIndex) {
    return CUtil.getName(tpr).toLowerCase() + "_deadline_function" + reactionIndex;
  }

  /**
   * Return the function name for specified reaction of the specified reactor.
   *
   * @param tpr The reactor
   * @param reactionIndex The reaction index.
   * @return The function name for the reaction.
   */
  public static String generateReactionFunctionName(
      TypeParameterizedReactor tpr, int reactionIndex) {
    return CUtil.getName(tpr).toLowerCase() + "reaction_function_" + reactionIndex;
  }

  /**
   * Returns the name of the stp function for reaction.
   *
   * @param tpr The reactor with the stp
   * @param reactionIndex The number assigned to this reaction deadline
   */
  public static String generateStpFunctionName(TypeParameterizedReactor tpr, int reactionIndex) {
    return CUtil.getName(tpr).toLowerCase() + "_STP_function" + reactionIndex;
  }

  /**
   * Return the top level C function header for the deadline function numbered "reactionIndex" in
   * "r"
   *
   * @param tpr The reactor declaration
   * @param reactionIndex The reaction index.
   * @return The function name for the deadline function.
   */
  public static String generateDeadlineFunctionHeader(
      TypeParameterizedReactor tpr, int reactionIndex) {
    String functionName = generateDeadlineFunctionName(tpr, reactionIndex);
    return generateFunctionHeader(functionName);
  }

  /**
   * Return the top level C function header for the reaction numbered "reactionIndex" in "r"
   *
   * @param tpr The reactor declaration
   * @param reactionIndex The reaction index.
   * @return The function name for the reaction.
   */
  public static String generateReactionFunctionHeader(
      TypeParameterizedReactor tpr, int reactionIndex) {
    String functionName = generateReactionFunctionName(tpr, reactionIndex);
    return generateFunctionHeader(functionName);
  }

  public static String generateStpFunctionHeader(TypeParameterizedReactor tpr, int reactionIndex) {
    String functionName = generateStpFunctionName(tpr, reactionIndex);
    return generateFunctionHeader(functionName);
  }

  /**
   * Return the start of a function declaration for a function that takes a {@code void*} argument
   * and returns void.
   *
   * @param functionName
   * @return
   */
  public static String generateFunctionHeader(String functionName) {
    return "void " + functionName + "(void* instance_args)";
  }
}
