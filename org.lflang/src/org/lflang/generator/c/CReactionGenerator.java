package org.lflang.generator.c;

import static org.lflang.generator.c.CUtil.generateWidthVariable;

import java.util.LinkedHashMap;
import java.util.LinkedHashSet;
import java.util.LinkedList;
import java.util.Map;
import java.util.Set;

import org.lflang.ASTUtils;
import org.lflang.ErrorReporter;
import org.lflang.InferredType;
import org.lflang.Target;
import org.lflang.federated.FederateInstance;
import org.lflang.generator.CodeBuilder;
import org.lflang.generator.ModeInstance.ModeTransitionType;
import org.lflang.lf.Action;
import org.lflang.lf.Code;
import org.lflang.lf.Input;
import org.lflang.lf.Instantiation;
import org.lflang.lf.Mode;
import org.lflang.lf.Output;
import org.lflang.lf.Port;
import org.lflang.lf.Reaction;
import org.lflang.lf.Reactor;
import org.lflang.lf.ReactorDecl;
import org.lflang.lf.TriggerRef;
import org.lflang.lf.VarRef;
import org.lflang.lf.Variable;

public class CReactionGenerator {

    /**
     * Generate necessary initialization code inside the body of the reaction that belongs to reactor decl.
     * @param body The body of the reaction. Used to check for the DISABLE_REACTION_INITIALIZATION_MARKER.
     * @param reaction The initialization code will be generated for this specific reaction
     * @param decl The reactor that has the reaction
     * @param reactionIndex The index of the reaction relative to other reactions in the reactor, starting from 0
     */ 
    public static String generateInitializationForReaction(String body, 
                                                           Reaction reaction, 
                                                           ReactorDecl decl, 
                                                           int reactionIndex, 
                                                           CTypes types,
                                                           ErrorReporter errorReporter,
                                                           Instantiation mainDef,
                                                           boolean isFederatedAndDecentralized,
                                                           boolean requiresTypes) {
        Reactor reactor = ASTUtils.toDefinition(decl);
        
        // Construct the reactionInitialization code to go into
        // the body of the function before the verbatim code.
        CodeBuilder reactionInitialization = new CodeBuilder();

        CodeBuilder code = new CodeBuilder();

        // Define the "self" struct.
        String structType = CUtil.selfType(decl);
        // A null structType means there are no inputs, state,
        // or anything else. No need to declare it.
        if (structType != null) {
             code.pr(String.join("\n",
                 "#pragma GCC diagnostic push",
                 "#pragma GCC diagnostic ignored \"-Wunused-variable\"",
                 structType+"* self = ("+structType+"*)instance_args;"
             ));
        }

        // Do not generate the initialization code if the body is marked
        // to not generate it.
        if (body.startsWith(CGenerator.DISABLE_REACTION_INITIALIZATION_MARKER)) {
             code.pr("#pragma GCC diagnostic pop");
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
            if (trigger instanceof VarRef) {
                VarRef triggerAsVarRef = (VarRef) trigger;
                if (triggerAsVarRef.getVariable() instanceof Port) {
                    generatePortVariablesInReaction(
                        reactionInitialization,
                        fieldsForStructsForContainedReactors,
                        triggerAsVarRef, 
                        decl, 
                        types);
                } else if (triggerAsVarRef.getVariable() instanceof Action) {
                    reactionInitialization.pr(generateActionVariablesInReaction(
                        (Action) triggerAsVarRef.getVariable(), 
                        decl,
                        types
                    ));
                    actionsAsTriggers.add((Action) triggerAsVarRef.getVariable());
                }
            }
        }
        if (reaction.getTriggers() == null || reaction.getTriggers().size() == 0) {
            // No triggers are given, which means react to any input.
            // Declare an argument for every input.
            // NOTE: this does not include contained outputs. 
            for (Input input : reactor.getInputs()) {
                reactionInitialization.pr(generateInputVariablesInReaction(input, decl, types));
            }
        }
        // Define argument for non-triggering inputs.
        for (VarRef src : ASTUtils.convertToEmptyListIfNull(reaction.getSources())) {
            if (src.getVariable() instanceof Port) {
                generatePortVariablesInReaction(reactionInitialization, fieldsForStructsForContainedReactors, src, decl, types);
            } else if (src.getVariable() instanceof Action) {
                // It's a bit odd to read but not be triggered by an action, but
                // OK, I guess we allow it.
                reactionInitialization.pr(generateActionVariablesInReaction(
                    (Action) src.getVariable(),
                    decl,
                    types
                ));
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
                        reactionInitialization.pr(CGenerator.variableStructType(variable, decl)+"* "+variable.getName()+" = &self->_lf_"+variable.getName()+";");
                    }
                } else if (effect.getVariable() instanceof Mode) {
                    // Mode change effect
                    int idx = ASTUtils.allModes(reactor).indexOf((Mode)effect.getVariable());
                    String name = effect.getVariable().getName();
                    if (idx >= 0) {
                        reactionInitialization.pr(
                            "reactor_mode_t* " + name + " = &self->_lf__modes[" + idx + "];\n"
                            + "char _lf_" + name + "_change_type = "
                            + (ModeTransitionType.getModeTransitionType(effect) == ModeTransitionType.HISTORY ? 2 : 1) 
                            + ";"
                        );
                    } else {
                        errorReporter.reportError(
                            reaction,
                            "In generateReaction(): " + name + " not a valid mode of this reactor."
                        );
                    }
                } else {
                    if (variable instanceof Output) {
                        reactionInitialization.pr(generateOutputVariablesInReaction(
                            effect,
                            decl,
                            errorReporter,
                            requiresTypes
                        ));
                    } else if (variable instanceof Input) {
                        // It is the input of a contained reactor.
                        generateVariablesForSendingToContainedReactors(
                            reactionInitialization,
                            fieldsForStructsForContainedReactors,
                            effect.getContainer(),
                            (Input) variable
                        );
                    } else {
                        errorReporter.reportError(
                            reaction,
                            "In generateReaction(): effect is neither an input nor an output."
                        );
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
                code.pr("int "+containedReactorWidthVar+" = self->_lf_"+containedReactorWidthVar+";");
                // Windows does not support variables in arrays declared on the stack,
                // so we use the maximum size over all bank members.
                array = "["+maxContainedReactorBankWidth(containedReactor, null, 0, mainDef)+"]";
            }
            code.pr(String.join("\n", 
                "struct "+containedReactor.getName()+" {",
                "    "+fieldsForStructsForContainedReactors.get(containedReactor)+"",
                "} "+containedReactor.getName()+array+";"
            ));
        }
        // Next generate all the collected setup code.
        code.pr(reactionInitialization.toString());
        code.pr("#pragma GCC diagnostic pop");

        if (reaction.getStp() == null) {
            // Pass down the intended_tag to all input and output effects
            // downstream if the current reaction does not have a STP
            // handler.
            code.pr(generateIntendedTagInheritence(body, reaction, decl, reactionIndex, types, isFederatedAndDecentralized));
        }
        return code.toString();
    }

    /**
     * Return the maximum bank width for the given instantiation within all
     * instantiations of its parent reactor.
     * On the first call to this method, the breadcrumbs should be null and the max
     * argument should be zero. On recursive calls, breadcrumbs is a list of nested
     * instantiations, the max is the maximum width found so far.  The search for
     * instances of the parent reactor will begin with the last instantiation
     * in the specified list.
     * 
     * This rather complicated method is used when a reaction sends or receives data
     * to or from a bank of contained reactors. There will be an array of structs on
     * the self struct of the parent, and the size of the array is conservatively set
     * to the maximum of all the identified bank widths.  This is a bit wasteful of
     * memory, but it avoids having to malloc the array for each instance, and in
     * typical usage, there will be few instances or instances that are all the same
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
        Instantiation mainDef
    ) {
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
                int candidate = maxContainedReactorBankWidth(containedReactor, nestedBreadcrumbs, result, mainDef);
                if (candidate > result) {
                    result = candidate;
                }
            }
            nestedBreadcrumbs.remove();
        }
        return result;
    }

    /**
     * Generate code that passes existing intended tag to all output ports
     * and actions. This intended tag is the minimum intended tag of the 
     * triggering inputs of the reaction.
     * 
     * @param body The body of the reaction. Used to check for the DISABLE_REACTION_INITIALIZATION_MARKER.
     * @param reaction The initialization code will be generated for this specific reaction
     * @param decl The reactor that has the reaction
     * @param reactionIndex The index of the reaction relative to other reactions in the reactor, starting from 0
     */
    public static String generateIntendedTagInheritence(String body, Reaction reaction, ReactorDecl decl, int reactionIndex, CTypes types, boolean isFederatedAndDecentralized) {
        // Construct the intended_tag inheritance code to go into
        // the body of the function.
        CodeBuilder intendedTagInheritenceCode = new CodeBuilder();
        // Check if the coordination mode is decentralized and if the reaction has any effects to inherit the STP violation
        if (isFederatedAndDecentralized && !(reaction.getEffects() == null || reaction.getEffects().isEmpty())) {
            intendedTagInheritenceCode.pr(String.join("\n", 
                "#pragma GCC diagnostic push",
                "#pragma GCC diagnostic ignored \"-Wunused-variable\"",
                "if (self->_lf__reaction_"+reactionIndex+".is_STP_violated == true) {"
            ));
            intendedTagInheritenceCode.indent();            
            intendedTagInheritenceCode.pr(String.join("\n", 
                "// The operations inside this if clause (if any exists) are expensive ",
                "// and must only be done if the reaction has unhandled STP violation.",
                "// Otherwise, all intended_tag values are (NEVER, 0) by default.",
                "",
                "// Inherited intended tag. This will take the minimum",
                "// intended_tag of all input triggers",
                types.getTargetTagType()+" inherited_min_intended_tag = ("+types.getTargetTagType()+") { .time = FOREVER, .microstep = UINT_MAX };"
            ));
            intendedTagInheritenceCode.pr("// Find the minimum intended tag");
            // Go through every trigger of the reaction and check the
            // value of intended_tag to choose the minimum.
            for (TriggerRef inputTrigger : ASTUtils.convertToEmptyListIfNull(reaction.getTriggers())) {
                if (inputTrigger instanceof VarRef) {
                    VarRef inputTriggerAsVarRef = (VarRef) inputTrigger;
                    Variable variable = inputTriggerAsVarRef.getVariable();
                    String variableName = inputTriggerAsVarRef.getVariable().getName();
                    if (variable instanceof Output) {
                        // Output from a contained reactor
                        String containerName = inputTriggerAsVarRef.getContainer().getName();
                        Output outputPort = (Output) variable;                        
                        if (ASTUtils.isMultiport(outputPort)) {
                            intendedTagInheritenceCode.pr(String.join("\n", 
                                "for (int i=0; i < "+containerName+"."+generateWidthVariable(variableName)+"; i++) {",
                                "    if (compare_tags("+containerName+"."+variableName+"[i]->intended_tag,",
                                "                        inherited_min_intended_tag) < 0) {",
                                "        inherited_min_intended_tag = "+containerName+"."+variableName+"[i]->intended_tag;",
                                "    }",
                                "}"
                            ));
                        } else
                            intendedTagInheritenceCode.pr(String.join("\n", 
                                "if (compare_tags("+containerName+"."+variableName+"->intended_tag,",
                                "                    inherited_min_intended_tag) < 0) {",
                                "    inherited_min_intended_tag = "+containerName+"."+variableName+"->intended_tag;",
                                "}"
                            ));
                    } else if (variable instanceof Port) {
                        // Input port
                        Port inputPort = (Port) variable; 
                        if (ASTUtils.isMultiport(inputPort)) {
                            intendedTagInheritenceCode.pr(String.join("\n", 
                                "for (int i=0; i < "+generateWidthVariable(variableName)+"; i++) {",
                                "    if (compare_tags("+variableName+"[i]->intended_tag, inherited_min_intended_tag) < 0) {",
                                "        inherited_min_intended_tag = "+variableName+"[i]->intended_tag;",
                                "    }",
                                "}"
                            ));
                        } else {
                            intendedTagInheritenceCode.pr(String.join("\n", 
                                "if (compare_tags("+variableName+"->intended_tag, inherited_min_intended_tag) < 0) {",
                                "    inherited_min_intended_tag = "+variableName+"->intended_tag;",
                                "}"
                            ));
                        }
                    } else if (variable instanceof Action) {
                        intendedTagInheritenceCode.pr(String.join("\n", 
                            "if (compare_tags("+variableName+"->trigger->intended_tag, inherited_min_intended_tag) < 0) {",
                            "    inherited_min_intended_tag = "+variableName+"->trigger->intended_tag;",
                            "}"
                        ));
                    }

                }
            }
            if (reaction.getTriggers() == null || reaction.getTriggers().size() == 0) {
                // No triggers are given, which means the reaction would react to any input.
                // We need to check the intended tag for every input.
                // NOTE: this does not include contained outputs. 
                for (Input input : ((Reactor) reaction.eContainer()).getInputs()) {
                    intendedTagInheritenceCode.pr(String.join("\n", 
                        "if (compare_tags("+input.getName()+"->intended_tag, inherited_min_intended_tag) > 0) {",
                        "    inherited_min_intended_tag = "+input.getName()+"->intended_tag;",
                        "}"
                    ));
                }
            }
            
            // Once the minimum intended tag has been found,
            // it will be passed down to the port effects
            // of the reaction. Note that the intended tag
            // will not pass on to actions downstream.
            // Last reaction that sets the intended tag for the effect
            // will be seen.
            intendedTagInheritenceCode.pr(String.join("\n", 
                "// All effects inherit the minimum intended tag of input triggers",
                "if (inherited_min_intended_tag.time != NEVER) {"
            ));
            intendedTagInheritenceCode.indent();
            for (VarRef effect : ASTUtils.convertToEmptyListIfNull(reaction.getEffects())) {
                Variable effectVar = effect.getVariable();
                Instantiation effContainer = effect.getContainer();
                if (effectVar instanceof Input) {
                    if (ASTUtils.isMultiport((Port) effectVar)) {
                        intendedTagInheritenceCode.pr(String.join("\n", 
                            "for(int i=0; i < "+effContainer.getName()+"."+generateWidthVariable(effectVar.getName())+"; i++) {",
                            "    "+effContainer.getName()+"."+effectVar.getName()+"[i]->intended_tag = inherited_min_intended_tag;",
                            "}"
                        ));
                    } else {
                        if (effContainer.getWidthSpec() != null) {
                            // Contained reactor is a bank.
                            intendedTagInheritenceCode.pr(String.join("\n", 
                                "for (int bankIndex = 0; bankIndex < self->_lf_"+generateWidthVariable(effContainer.getName())+"; bankIndex++) {",
                                "    "+effContainer.getName()+"[bankIndex]."+effectVar.getName()+" = &(self->_lf_"+effContainer.getName()+"[bankIndex]."+effectVar.getName()+");",
                                "}"
                            ));
                        } else {
                            // Input to a contained reaction
                            intendedTagInheritenceCode.pr(String.join("\n", 
                                "// Don't reset the intended tag of the output port if it has already been set.",
                                effContainer.getName()+"."+effectVar.getName()+"->intended_tag = inherited_min_intended_tag;"
                            ));
                        }
                    }                   
                }
            }
            intendedTagInheritenceCode.unindent();
            intendedTagInheritenceCode.pr("}");
            intendedTagInheritenceCode.unindent();
            intendedTagInheritenceCode.pr("#pragma GCC diagnostic pop");
            intendedTagInheritenceCode.pr("}");
            
        }
        return intendedTagInheritenceCode.toString();
    }

    /** 
     * Generate into the specified string builder the code to
     * initialize local variables for sending data to an input
     * of a contained reactor. This will also, if necessary,
     * generate entries for local struct definitions into the
     * struct argument. These entries point to where the data
     * is stored.
     * 
     * @param builder The string builder.
     * @param structs A map from reactor instantiations to a place to write
     *  struct fields.
     * @param definition AST node defining the reactor within which this occurs
     * @param input Input of the contained reactor.
     */
    private static void generateVariablesForSendingToContainedReactors(
        CodeBuilder builder,
        Map<Instantiation,CodeBuilder> structs,
        Instantiation definition,
        Input input
    ) {
        CodeBuilder structBuilder = structs.get(definition);
        if (structBuilder == null) {
            structBuilder = new CodeBuilder();
            structs.put(definition, structBuilder);
        }
        String inputStructType = CGenerator.variableStructType(input, definition.getReactorClass()).toString();
        String defName = definition.getName();
        String defWidth = generateWidthVariable(defName);
        String inputName = input.getName();
        String inputWidth = generateWidthVariable(inputName);
        if (!ASTUtils.isMultiport(input)) {
            // Contained reactor's input is not a multiport.
            structBuilder.pr(inputStructType+"* "+inputName+";");
            if (definition.getWidthSpec() != null) {
                // Contained reactor is a bank.
                builder.pr(String.join("\n", 
                    "for (int bankIndex = 0; bankIndex < self->_lf_"+defWidth+"; bankIndex++) {",
                    "    "+defName+"[bankIndex]."+inputName+" = &(self->_lf_"+defName+"[bankIndex]."+inputName+");",
                    "}"
                ));
            } else {
                // Contained reactor is not a bank.
                builder.pr(defName+"."+inputName+" = &(self->_lf_"+defName+"."+inputName+");");
            }
        } else {
            // Contained reactor's input is a multiport.
            structBuilder.pr(String.join("\n", 
                inputStructType+"** "+inputName+";",
                "int "+inputWidth+";"
            ));
            // If the contained reactor is a bank, then we have to set the
            // pointer for each element of the bank.
            if (definition.getWidthSpec() != null) { 
                builder.pr(String.join("\n", 
                    "for (int _i = 0; _i < self->_lf_"+defWidth+"; _i++) {",
                    "    "+defName+"[_i]."+inputName+" = self->_lf_"+defName+"[_i]."+inputName+";",
                    "    "+defName+"[_i]."+inputWidth+" = self->_lf_"+defName+"[_i]."+inputWidth+";",
                    "}"
                ));
            } else {
                builder.pr(String.join("\n", 
                    defName+"."+inputName+" = self->_lf_"+defName+"."+inputName+";",
                    defName+"."+inputWidth+" = self->_lf_"+defName+"."+inputWidth+";"
                ));
            }
        }
    }

    /** 
     * Generate into the specified string builder the code to
     * initialize local variables for ports in a reaction function
     * from the "self" struct. The port may be an input of the
     * reactor or an output of a contained reactor. The second
     * argument provides, for each contained reactor, a place to
     * write the declaration of the output of that reactor that
     * is triggering reactions.
     * @param builder The place into which to write the code.
     * @param structs A map from reactor instantiations to a place to write
     *  struct fields.
     * @param port The port.
     * @param reactor The reactor or import statement.
     */
    private static void generatePortVariablesInReaction(
        CodeBuilder builder,
        Map<Instantiation,CodeBuilder> structs,
        VarRef port,
        ReactorDecl decl,
        CTypes types
    ) {
        if (port.getVariable() instanceof Input) {
            builder.pr(generateInputVariablesInReaction((Input) port.getVariable(), decl, types));
        } else {
            // port is an output of a contained reactor.
            Output output = (Output) port.getVariable();
            String portStructType = CGenerator.variableStructType(output, port.getContainer().getReactorClass()).toString();
            
            CodeBuilder structBuilder = structs.get(port.getContainer());
            if (structBuilder == null) {
                structBuilder = new CodeBuilder();
                structs.put(port.getContainer(), structBuilder);
            }
            String reactorName = port.getContainer().getName();
            String reactorWidth = generateWidthVariable(reactorName);
            String outputName = output.getName();
            String outputWidth = generateWidthVariable(outputName);
            // First define the struct containing the output value and indicator
            // of its presence.
            if (!ASTUtils.isMultiport(output)) {
                // Output is not a multiport.
                structBuilder.pr(portStructType+"* "+outputName+";");
            } else {
                // Output is a multiport.
                structBuilder.pr(String.join("\n", 
                    portStructType+"** "+outputName+";",
                    "int "+outputWidth+";"
                ));
            }
            
            // Next, initialize the struct with the current values.
            if (port.getContainer().getWidthSpec() != null) {
                // Output is in a bank.
                builder.pr(String.join("\n", 
                    "for (int i = 0; i < "+reactorWidth+"; i++) {",
                    "    "+reactorName+"[i]."+outputName+" = self->_lf_"+reactorName+"[i]."+outputName+";",
                    "}"
                ));
                if (ASTUtils.isMultiport(output)) {
                    builder.pr(String.join("\n", 
                        "for (int i = 0; i < "+reactorWidth+"; i++) {",
                        "    "+reactorName+"[i]."+outputWidth+" = self->_lf_"+reactorName+"[i]."+outputWidth+";",
                        "}"
                    ));                   
                }
            } else {
                 // Output is not in a bank.
                builder.pr(reactorName+"."+outputName+" = self->_lf_"+reactorName+"."+outputName+";");                    
                if (ASTUtils.isMultiport(output)) {
                    builder.pr(reactorName+"."+outputWidth+" = self->_lf_"+reactorName+"."+outputWidth+";");     
                }
            }
        }
    }

    /** Generate action variables for a reaction.
     *  @param builder Where to write the code.
     *  @param action The action.
     *  @param reactor The reactor.
     */
    private static String generateActionVariablesInReaction(
        Action action,
        ReactorDecl decl,
        CTypes types
    ) {
        String structType = CGenerator.variableStructType(action, decl).toString();
        // If the action has a type, create variables for accessing the value.
        InferredType type = ASTUtils.getInferredType(action);
        // Pointer to the lf_token_t sent as the payload in the trigger.
        String tokenPointer = "(self->_lf__"+action.getName()+".token)";
        CodeBuilder builder = new CodeBuilder();

        builder.pr(
            String.join("\n", 
            "// Expose the action struct as a local variable whose name matches the action name.",
            structType+"* "+action.getName()+" = &self->_lf_"+action.getName()+";",
            "// Set the fields of the action struct to match the current trigger.",
            action.getName()+"->is_present = (bool)self->_lf__"+action.getName()+".status;",
            action.getName()+"->has_value = ("+tokenPointer+" != NULL && "+tokenPointer+"->value != NULL);",
            action.getName()+"->token = "+tokenPointer+";")
        );
        // Set the value field only if there is a type.
        if (!type.isUndefined()) {
            // The value field will either be a copy (for primitive types)
            // or a pointer (for types ending in *).
            builder.pr("if ("+action.getName()+"->has_value) {");
            builder.indent();
            if (CUtil.isTokenType(type, types)) {
                builder.pr(action.getName()+"->value = ("+types.getTargetType(type)+")"+tokenPointer+"->value;");
            } else {
                builder.pr(action.getName()+"->value = *("+types.getTargetType(type)+"*)"+tokenPointer+"->value;");
            }
            builder.unindent();
            builder.pr("}");
        }
        return builder.toString();
    }
    
    /** Generate into the specified string builder the code to
     *  initialize local variables for the specified input port
     *  in a reaction function from the "self" struct.
     *  @param builder The string builder.
     *  @param input The input statement from the AST.
     *  @param reactor The reactor.
     */
    private static String generateInputVariablesInReaction(
        Input input,
        ReactorDecl decl,
        CTypes types
    ) {
        String structType = CGenerator.variableStructType(input, decl).toString();
        InferredType inputType = ASTUtils.getInferredType(input);
        CodeBuilder builder = new CodeBuilder();
        String inputName = input.getName();
        String inputWidth = generateWidthVariable(inputName);
        
        // Create the local variable whose name matches the input name.
        // If the input has not been declared mutable, then this is a pointer
        // to the upstream output. Otherwise, it is a copy of the upstream output,
        // which nevertheless points to the same token and value (hence, as done
        // below, we have to use writable_copy()). There are 8 cases,
        // depending on whether the input is mutable, whether it is a multiport,
        // and whether it is a token type.
        // Easy case first.
        if (!input.isMutable() && !CUtil.isTokenType(inputType, types) && !ASTUtils.isMultiport(input)) {
            // Non-mutable, non-multiport, primitive type.
            builder.pr(structType+"* "+inputName+" = self->_lf_"+inputName+";");
        } else if (input.isMutable()&& !CUtil.isTokenType(inputType, types) && !ASTUtils.isMultiport(input)) {
            // Mutable, non-multiport, primitive type.
            builder.pr(String.join("\n", 
                "// Mutable input, so copy the input into a temporary variable.",
                "// The input value on the struct is a copy.",
                structType+" _lf_tmp_"+inputName+" = *(self->_lf_"+inputName+");",
                structType+"* "+inputName+" = &_lf_tmp_"+inputName+";"
            ));
        } else if (!input.isMutable()&& CUtil.isTokenType(inputType, types) && !ASTUtils.isMultiport(input)) {
            // Non-mutable, non-multiport, token type.
            builder.pr(String.join("\n", 
                structType+"* "+inputName+" = self->_lf_"+inputName+";",
                "if ("+inputName+"->is_present) {",
                "    "+inputName+"->length = "+inputName+"->token->length;",
                "    "+inputName+"->value = ("+types.getTargetType(inputType)+")"+inputName+"->token->value;",
                "} else {",
                "    "+inputName+"->length = 0;",
                "}"
            ));
        } else if (input.isMutable()&& CUtil.isTokenType(inputType, types) && !ASTUtils.isMultiport(input)) {
            // Mutable, non-multiport, token type.
            builder.pr(String.join("\n", 
                "// Mutable input, so copy the input struct into a temporary variable.",
                structType+" _lf_tmp_"+inputName+" = *(self->_lf_"+inputName+");",
                structType+"* "+inputName+" = &_lf_tmp_"+inputName+";",
                "if ("+inputName+"->is_present) {",
                "    "+inputName+"->length = "+inputName+"->token->length;",
                "    lf_token_t* _lf_input_token = "+inputName+"->token;",
                "    "+inputName+"->token = writable_copy(_lf_input_token);",
                "    if ("+inputName+"->token != _lf_input_token) {",
                "        // A copy of the input token has been made.",
                "        // This needs to be reference counted.",
                "        "+inputName+"->token->ref_count = 1;",
                "        // Repurpose the next_free pointer on the token to add to the list.",
                "        "+inputName+"->token->next_free = _lf_more_tokens_with_ref_count;",
                "        _lf_more_tokens_with_ref_count = "+inputName+"->token;",
                "    }",
                "    "+inputName+"->value = ("+types.getTargetType(inputType)+")"+inputName+"->token->value;",
                "} else {",
                "    "+inputName+"->length = 0;",
                "}"
            ));
        } else if (!input.isMutable()&& ASTUtils.isMultiport(input)) {
            // Non-mutable, multiport, primitive or token type.
            builder.pr(structType+"** "+inputName+" = self->_lf_"+inputName+";");
        } else if (CUtil.isTokenType(inputType, types)) {
            // Mutable, multiport, token type
            builder.pr(String.join("\n", 
                "// Mutable multiport input, so copy the input structs",
                "// into an array of temporary variables on the stack.",
                structType+" _lf_tmp_"+inputName+"["+CUtil.multiportWidthExpression(input)+"];",
                structType+"* "+inputName+"["+CUtil.multiportWidthExpression(input)+"];",
                "for (int i = 0; i < "+CUtil.multiportWidthExpression(input)+"; i++) {",
                "    "+inputName+"[i] = &_lf_tmp_"+inputName+"[i];",
                "    _lf_tmp_"+inputName+"[i] = *(self->_lf_"+inputName+"[i]);",
                "    // If necessary, copy the tokens.",
                "    if ("+inputName+"[i]->is_present) {",
                "        "+inputName+"[i]->length = "+inputName+"[i]->token->length;",
                "        lf_token_t* _lf_input_token = "+inputName+"[i]->token;",
                "        "+inputName+"[i]->token = writable_copy(_lf_input_token);",
                "        if ("+inputName+"[i]->token != _lf_input_token) {",
                "            // A copy of the input token has been made.",
                "            // This needs to be reference counted.",
                "            "+inputName+"[i]->token->ref_count = 1;",
                "            // Repurpose the next_free pointer on the token to add to the list.",
                "            "+inputName+"[i]->token->next_free = _lf_more_tokens_with_ref_count;",
                "            _lf_more_tokens_with_ref_count = "+inputName+"[i]->token;",
                "        }",
                "        "+inputName+"[i]->value = ("+types.getTargetType(inputType)+")"+inputName+"[i]->token->value;",
                "    } else {",
                "        "+inputName+"[i]->length = 0;",
                "    }",
                "}"
            ));
        } else {
            // Mutable, multiport, primitive type
            builder.pr(String.join("\n", 
                "// Mutable multiport input, so copy the input structs",
                "// into an array of temporary variables on the stack.",
                structType+" _lf_tmp_"+inputName+"["+CUtil.multiportWidthExpression(input)+"];",
                structType+"* "+inputName+"["+CUtil.multiportWidthExpression(input)+"];",
                "for (int i = 0; i < "+CUtil.multiportWidthExpression(input)+"; i++) {",
                "    "+inputName+"[i]  = &_lf_tmp_"+inputName+"[i];",
                "    // Copy the struct, which includes the value.",
                "    _lf_tmp_"+inputName+"[i] = *(self->_lf_"+inputName+"[i]);",
                "}"
            ));
        }
        // Set the _width variable for all cases. This will be -1
        // for a variable-width multiport, which is not currently supported.
        // It will be -2 if it is not multiport.
        builder.pr("int "+inputWidth+" = self->_lf_"+inputWidth+";");
        return builder.toString();
    }

    /** 
     * Generate into the specified string builder the code to
     * initialize local variables for outputs in a reaction function
     * from the "self" struct.
     * @param effect The effect declared by the reaction. This must refer to an output.
     * @param decl The reactor containing the reaction or the import statement.
     */
    public static String generateOutputVariablesInReaction(
        VarRef effect,
        ReactorDecl decl,
        ErrorReporter errorReporter,
        boolean requiresTypes
    ) {
        Output output = (Output) effect.getVariable();
        String outputName = output.getName();
        String outputWidth = generateWidthVariable(outputName);
        if (output.getType() == null && requiresTypes) {
            errorReporter.reportError(output, "Output is required to have a type: " + outputName);
            return "";
        } else {
            // The container of the output may be a contained reactor or
            // the reactor containing the reaction.
            String outputStructType = (effect.getContainer() == null) ?
                    CGenerator.variableStructType(output, decl).toString()
                    :
                    CGenerator.variableStructType(output, effect.getContainer().getReactorClass()).toString();
            if (!ASTUtils.isMultiport(output)) {
                // Output port is not a multiport.
                return outputStructType+"* "+outputName+" = &self->_lf_"+outputName+";";
            } else {
                // Output port is a multiport.
                // Set the _width variable.
                return String.join("\n",
                    "int "+outputWidth+" = self->_lf_"+outputWidth+";",
                    outputStructType+"** "+outputName+" = self->_lf_"+outputName+"_pointers;"
                );
                    
            }
        }
    }
    
    /** Generate a reaction function definition for a reactor.
     *  This function will have a single argument that is a void* pointing to
     *  a struct that contains parameters, state variables, inputs (triggering or not),
     *  actions (triggering or produced), and outputs.
     *  @param reaction The reaction.
     *  @param reactor The reactor.
     *  @param reactionIndex The position of the reaction within the reactor. 
     */
    public static String generateReaction(
        Reaction reaction, 
        ReactorDecl decl, 
        int reactionIndex, 
        Instantiation mainDef, 
        ErrorReporter errorReporter,
        CTypes types,
        boolean isFederatedAndDecentralized,
        boolean requiresType
    ) {
        var code = new CodeBuilder();
        var body = ASTUtils.toText(reaction.getCode());
        String init = generateInitializationForReaction(
                        body, reaction, decl, reactionIndex, 
                        types, errorReporter, mainDef, 
                        isFederatedAndDecentralized, 
                        requiresType);
        code.pr(generateFunction(
            generateReactionFunctionHeader(decl, reactionIndex),
            init, reaction.getCode()
        ));

        // Now generate code for the late function, if there is one
        // Note that this function can only be defined on reactions
        // in federates that have inputs from a logical connection.
        if (reaction.getStp() != null) {
            code.pr(generateFunction(
                generateStpFunctionHeader(decl, reactionIndex), 
                init, reaction.getStp().getCode()));
        }

        // Now generate code for the deadline violation function, if there is one.
        if (reaction.getDeadline() != null) {
            code.pr(generateFunction(
                generateDeadlineFunctionHeader(decl, reactionIndex), 
                init, reaction.getDeadline().getCode()));
        }
        return code.toString();
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
     * @param decl The reactor with the deadline
     * @param index The number assigned to this reaction deadline
     */
    public static String generateDeadlineFunctionName(ReactorDecl decl, int reactionIndex) {
        return decl.getName().toLowerCase() + "_deadline_function" + reactionIndex;
    }

    /** 
     * Return the function name for specified reaction of the
     * specified reactor.
     * @param reactor The reactor
     * @param reactionIndex The reaction index.
     * @return The function name for the reaction.
     */
    public static String generateReactionFunctionName(ReactorDecl reactor, int reactionIndex) {
        return reactor.getName().toLowerCase() + "reaction_function_" + reactionIndex;
    }

    /**
     * Returns the name of the stp function for reaction.
     * @param decl The reactor with the stp
     * @param index The number assigned to this reaction deadline
     */
    public static String generateStpFunctionName(ReactorDecl decl, int reactionIndex) {
        return decl.getName().toLowerCase() + "_STP_function" + reactionIndex;
    }

    /** Return the top level C function header for the deadline function numbered "reactionIndex" in "decl"
     *  @param decl The reactor declaration
     *  @param reactionIndex The reaction index.
     *  @return The function name for the deadline function.
     */
    public static String generateDeadlineFunctionHeader(ReactorDecl decl,
                                                        int reactionIndex) {
        String functionName = generateDeadlineFunctionName(decl, reactionIndex);
        return generateFunctionHeader(functionName);
    }

    /** Return the top level C function header for the reaction numbered "reactionIndex" in "decl"
     *  @param decl The reactor declaration
     *  @param reactionIndex The reaction index.
     *  @return The function name for the reaction.
     */
    public static String generateReactionFunctionHeader(ReactorDecl decl,
                                                        int reactionIndex) {
        String functionName = generateReactionFunctionName(decl, reactionIndex);
        return generateFunctionHeader(functionName);
    }

    public static String generateStpFunctionHeader(ReactorDecl decl,
                                                   int reactionIndex) {
        String functionName = generateStpFunctionName(decl, reactionIndex);
        return generateFunctionHeader(functionName);
    }

    private static String generateFunctionHeader(String functionName) {
        return "void " + functionName + "(void* instance_args)";
    }
}
