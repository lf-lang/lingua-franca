/*
 * Copyright (c) 2021, TU Dresden.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY
 * EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL
 * THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
 * STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF
 * THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.lflang.generator.rust

import org.lflang.*
import org.lflang.generator.PrependOperator
import org.lflang.generator.PrependOperator.rangeTo
import org.lflang.generator.TargetCode


/**
 * Emits modules for a single reactor.
 *
 * @author Clément Fournier - TU Dresden, INSA Rennes
 */
object RustReactorEmitter : RustEmitterBase() {

    fun Emitter.emitReactorModule(reactor: ReactorInfo) {
        makeReactorModule(this, reactor)
    }

    private fun makeReactorModule(out: Emitter, reactor: ReactorInfo) {
        out += with(reactor) {
            val typeParams = typeParamList.map { it.targetCode }.angle()
            val typeArgs = typeParamList.map { it.lfName }.angle()

            val privateParams = reactor.extraConstructionParams;

            with(reactor.names) {
                with(PrependOperator) {
                    """
                |${generatedByComment("//")}
                |#![allow(unused)]
                |
                |use $rsRuntime::prelude::*;
                |
${"             |"..reactor.preambles.joinToString("\n\n") { "// preamble {=\n${it.trimIndent()}\n// =}" }}
                |
                |/// Generated from ${loc.display()}
                |///
                |/${loc.lfTextComment()}
                |pub struct $structName$typeParams {
                |    pub __phantom: std::marker::PhantomData<(${typeParamList.map { it.lfName }.joinWithCommas()})>,
${"             |    "..reactor.stateVars.joinWithCommasLn { it.lfName + ": " + it.type }}
                |}
                |
                |#[warn(unused)]
                |impl$typeParams $structName$typeArgs {
                |
${"             |    "..reactions.joinToString("\n\n") { it.toWorkerFunction() }}
                |
                |}
                |
                |/// Parameters for the construction of a [$structName]
                |pub struct ${names.paramStructName}$typeParams {
                |    __phantom: std::marker::PhantomData<(${typeParamList.map { it.lfName }.joinWithCommas()})>,
${"             |    "..ctorParams.joinWithCommasLn { "${it.lfName.escapeRustIdent()}: ${it.type}" }}
                |}
                |
                |impl$typeParams ${names.paramStructName}$typeArgs {
                |   pub fn new(
${"             |       "..ctorParams.joinWithCommasLn { "${it.lfName.escapeRustIdent()}: ${it.type}" }}
                |   ) -> Self {
                |       Self { __phantom: std::marker::PhantomData, ${ctorParams.joinWithCommas { it.lfName.escapeRustIdent() }} }
                |   }
                |}
                |
                |struct $privateParamStruct {
${"             |       "..privateParams.joinWithCommasLn { "${it.ident.escapeRustIdent()}: ${it.type}" }}
                |}
                |
                |//------------------------//
                |
                |
                |pub struct $wrapperName$typeParams {
                |    __id: $rsRuntime::ReactorId,
                |    __impl: $structName$typeArgs,
${"             |    "..otherComponents.joinWithCommasLn { it.toStructField() }}
                |}
                |
                |impl$typeParams $wrapperName$typeArgs {
                |    #[inline]
                |    fn user_assemble(__assembler: &mut $rsRuntime::assembly::ComponentCreator<Self>,
                |                     __id: $rsRuntime::ReactorId,
                |                     __params: $paramStructName$typeArgs,
                |                     $privateParamsVarName: $privateParamStruct) -> $rsRuntime::assembly::AssemblyResult<Self> {
                |        let $ctorParamsDeconstructor = __params;
                |
                |        let __impl = {
                |            // declare them all here so that they are visible to the initializers of state vars declared later
${"             |            "..reactor.stateVars.joinWithLn { "let ${it.lfName}: ${it.type} = ${it.init};" }}
                |
                |            $structName {
                |                __phantom: std::marker::PhantomData,
${"             |                "..reactor.stateVars.joinWithCommasLn { it.lfName }}
                |            }
                |        };
                |
                |        Ok(Self {
                |            __id,
                |            __impl,
${"             |            "..otherComponents.joinWithCommasLn { it.rustFieldName + ": " + it.initialExpression() }}
                |        })
                |    }
                |}
                |
                |impl$typeParams $rsRuntime::assembly::ReactorInitializer for $wrapperName$typeArgs {
                |    type Wrapped = $structName$typeArgs;
                |    type Params = $paramStructName$typeArgs;
                |    const MAX_REACTION_ID: $rsRuntime::LocalReactionId = $rsRuntime::LocalReactionId::new($totalNumReactions - 1);
                |
                |    fn assemble(__params: Self::Params, __ctx: $rsRuntime::assembly::AssemblyCtx<Self>)
                |       -> $rsRuntime::assembly::AssemblyResult<$rsRuntime::assembly::FinishedReactor<Self>> {
                |        use $rsRuntime::assembly::TriggerLike;
                |
                |        let $ctorParamsDeconstructor = __params;
                |        __ctx.assemble(|__ctx|
${"             |            "..assembleChildReactors(assembleSelfCall())}
                |        )
                |    }
                |}
                |
                |
                |impl$typeParams $rsRuntime::ReactorBehavior for $wrapperName$typeArgs {
                |
                |    #[inline]
                |    fn id(&self) -> $rsRuntime::ReactorId {
                |        self.__id
                |    }
                |
                |    fn react(&mut self, ctx: &mut $rsRuntime::ReactionCtx, rid: $rsRuntime::LocalReactionId) {
                |        match rid.raw() {
${"             |            "..workerFunctionCalls()}
${"             |            "..syntheticTimerReactions()}
                |            _ => panic!("Invalid reaction ID: {} should be < {}", rid, <Self as $rsRuntime::assembly::ReactorInitializer>::MAX_REACTION_ID)
                |        }
                |    }
                |
                |    fn cleanup_tag(&mut self, ctx: &$rsRuntime::CleanupCtx) {
${"             |        "..otherComponents.mapNotNull { it.cleanupAction() }.joinLn()}
                |    }
                |
                |}
        """.trimMargin()
                }
            }
        }
    }

    private fun ReactorInfo.assembleChildReactors(assembleSelf: String): String {
        fun NestedReactorInstance.childDeclaration(): String {
            val type = "super::${names.wrapperName}${typeArgs.angle()}"
            val params = args.values.joinWithCommas("super::${names.paramStructName}::new(", ")")

            return if (bankWidth != null)
                "__ctx.with_child_bank::<$type, _, _>(\"$lfName\", ${bankWidth.toRustExpr()}, |bank_index| $params, |mut __ctx, $rustLocalName| {"
            else
                "__ctx.with_child::<$type, _>(\"$lfName\", $params, |mut __ctx, $rustLocalName| {"
        }

        val portRefs = this.portReferences
        fun NestedReactorInstance.portWidthDecls(): List<TargetCode> =
            // if we refer to some port of the child as a bank, we need to surface its width here
            portRefs.filter { it.childLfName == this.lfName && it.isGeneratedAsMultiport }.map {
                val portWidthExpr = if (it.isMultiport) "${it.childLfName}.${it.rustFieldOnChildName}.len()"
                else "1" // that's a single port

                // if we're in a bank, the total length is the sum
                val sumExpr =
                    if (it.isContainedInBank) "${it.childLfName}.iter().map(|${it.childLfName}| $portWidthExpr).sum()"
                    else portWidthExpr

                "let ${it.widthParamName} = $sumExpr;"
            }


        return buildString {
            for (inst in nestedInstances) {
                append(inst.childDeclaration()).append("\n")
                inst.portWidthDecls().joinTo(this, "\n").append("\n")
            }

            append(assembleSelf).append("\n")

            for (inst in nestedInstances) {
                append("})")
            }
        }
    }

    private fun ReactorInfo.assembleSelfCall(): String {
        val reactionIds = reactions.map { it.invokerId } +
                timers.map { it.rescheduleReactionId } +
                timers.map { it.startReactionId }

        val debugLabels = reactions.map { it.debugLabel?.withDQuotes().toRustOption() } +
                timers.map { "Some(\"reschedule_${it.lfName}\")" } +
                timers.map { "Some(\"bootstrap_${it.lfName}\")" }


        val pattern = reactionIds.joinToString(prefix = "[", separator = ", ", postfix = "]")
        val debugLabelArray = debugLabels.joinToString(", ", "[", "]")

        val privateParamsCtor = extraConstructionParams.joinWithCommas(prefix = "$privateParamStruct { ", postfix = " }") {
            it.ident.escapeRustIdent()
        }

        return """
                |__ctx.assemble_self(
                |    |cc, id| Self::user_assemble(cc, id, $ctorParamsDeconstructor, $privateParamsCtor),
                |    // number of non-synthetic reactions
                |    ${reactions.size},
                |    // reaction debug labels
                |    $debugLabelArray,
                |    // dependency declarations
                |    |__assembler, __self, $pattern| {
                |        #[allow(unused)]
                |        use reactor_rt::unsafe_iter_bank;
${"             |        "..graphDependencyDeclarations()}
${"             |        "..declareChildConnections()}
                |
                |        Ok(())
                |    }
                |)
            """.trimMargin()
    }


    private fun ReactorInfo.declareChildConnections(): String = with(PortEmitter) {
        connections.joinToString("\n", prefix = "// Declare connections\n") {
            it.declareConnection("__assembler")
        } + "\n" + portReferences.joinToString("\n", prefix = "// Declare port references\n") {
            it.declarePortRef("__assembler")
        }
    }

    /** Renders calls to worker functions from within the react_erased function. */
    private fun ReactorInfo.workerFunctionCalls(): TargetCode {

        fun joinDependencies(reaction: ReactionInfo): String = sequence {
            for ((kind, deps) in reaction.allDependencies) {
                for (comp in deps) {
                    if (comp.isNotInjectedInReaction(kind, reaction)) continue

                    val borrow = comp.toBorrow(kind)
                    this.yield(borrow)
                }
            }
        }.toList().let {
            if (it.isEmpty()) ""
            else it.joinWithCommas(prefix = ", ")
        }

        return reactions.joinWithCommasLn(trailing = true) { n: ReactionInfo ->
            "${n.idx} => self.__impl.${n.workerId}(ctx${joinDependencies(n)})"
        }
    }

    /**
     * Number of reactions, including synthetic reactions.
     * Timers each have a reschedule and a bootstrap reaction.
     */
    private val ReactorInfo.totalNumReactions
        get() = 1 + reactions.size + 2 * timers.size

    /** Renders the branches corresponding to synthetic timer reactions in react_erased. */
    private fun ReactorInfo.syntheticTimerReactions(): String {
        fun ReactorInfo.timerReactionId(timer: TimerData, synthesisNum: Int) =
            reactions.size +
                    timers.indexOf(timer).also { assert(it != -1) } +
                    synthesisNum * timers.size // offset it by a block

        val branches = timers.map {
            "${timerReactionId(it, 0)} => ctx.reschedule_timer(&mut self.${it.rustFieldName})"
        } + timers.map {
            "${timerReactionId(it, 1)} => ctx.bootstrap_timer(&mut self.${it.rustFieldName})"
        }
        return branches.joinWithCommasLn(trailing = true)
    }

    /** Build the dependency graph using the assembler. */
    private fun ReactorInfo.graphDependencyDeclarations(): String {
        val reactions = reactions.map { n ->
            val deps: List<String> = mutableListOf<String>().apply {
                this += n.triggers.map { trigger -> "__assembler.declare_triggers(__self.${trigger.rustFieldName}.get_id(), ${n.invokerId})?;" }
                if (n.isStartup)
                    this += "__assembler.declare_triggers($rsRuntime::assembly::TriggerId::STARTUP, ${n.invokerId})?;"
                if (n.isShutdown)
                    this += "__assembler.declare_triggers($rsRuntime::assembly::TriggerId::SHUTDOWN, ${n.invokerId})?;"
                this += n.uses.map { trigger -> "__assembler.declare_uses(${n.invokerId}, __self.${trigger.rustFieldName}.get_id())?;" }
                this += n.effects.filterIsInstance<PortLike>().map { port ->
                    if (port.isGeneratedAsMultiport) {
                        "__assembler.effects_multiport(${n.invokerId}, &__self.${port.rustFieldName})?;"
                    } else {
                        "__assembler.effects_port(${n.invokerId}, &__self.${port.rustFieldName})?;"
                    }
                }
            }

            n.loc.lfTextComment() + "\n" + deps.joinLn()
        }.joinLn()

        val timers = timers.flatMap {
            listOf(
                "__assembler.declare_triggers(__self.${it.rustFieldName}.get_id(), ${it.rescheduleReactionId})?;",
                // start reactions may "trigger" the timer, otherwise it schedules it
                "__assembler.declare_triggers($rsRuntime::assembly::TriggerId::STARTUP, ${it.startReactionId})?;",
                "__assembler.effects_timer(${it.startReactionId}, &__self.${it.rustFieldName})?;",
            )
        }.joinLn()

        return (reactions + "\n\n" + timers).trimEnd()
    }

    /** Name of the reschedule reaction for this timer. */
    private val TimerData.rescheduleReactionId: String
        get() = "__timer_schedule_$lfName"

    /** Name of the bootstrap reaction for this timer. */
    private val TimerData.startReactionId: String
        get() = "__timer_start_$lfName"

    /**
     * Rust pattern that deconstructs a ctor param tuple into individual variables.
     * Note that if all variables are in scope, it may also be used as a constructor.
     */
    private val ReactorInfo.ctorParamsDeconstructor: TargetCode
        get() {
            val fields = ctorParams.joinWithCommas { it.lfName.escapeRustIdent() }
            return "${names.paramStructName} {  __phantom, $fields }"
        }

    /** Renders the field of the reactor struct that will contain this component. */
    private fun ReactorComponent.toStructField(): TargetCode {
        val fieldVisibility = if (this is PortData) "pub " else ""
        return "$fieldVisibility$rustFieldName: ${toType()}"
    }

    /** The owned type of this reactor component (type of the struct field). */
    private fun ReactorComponent.toType(): TargetCode = when (this) {
        is ActionData ->
            if (isLogical) "$rsRuntime::LogicalAction<${dataType ?: "()"}>"
            else "$rsRuntime::PhysicalActionRef<${dataType ?: "()"}>"
        is PortLike   -> with(this) {
            if (isGeneratedAsMultiport) "$rsRuntime::Multiport<$dataType>"
            else "$rsRuntime::Port<$dataType>"
        }
        is TimerData  -> "$rsRuntime::Timer"
    }

    /** Initial expression for the field. */

    private fun ReactorComponent.initialExpression(): TargetCode = when (this) {
        is ActionData         -> {
            val delay = minDelay.toRustOption()
            val ctorName = if (isLogical) "new_logical_action" else "new_physical_action"
            "__assembler.$ctorName::<$dataType>(\"$lfName\", $delay)"
        }
        is TimerData          -> "__assembler.new_timer(\"$lfName\", $offset, $period)"
        is PortData           -> {
            if (widthSpec != null) {
                "__assembler.new_multiport::<$dataType>(\"$lfName\", $portKind, $widthSpec)?"
            } else {
                "__assembler.new_port::<$dataType>(\"$lfName\", $portKind)"
            }
        }
        is ChildPortReference -> {
            if (isGeneratedAsMultiport) {
                "__assembler.new_multiport::<$dataType>(\"$childLfName.$lfName\", $portKind, $privateParamsVarName.$widthParamName)?"
            } else {
                "__assembler.new_port::<$dataType>(\"$childLfName.$lfName\", $portKind)"
            }
        }
    }

    private val PortLike.portKind: String
        get() {
            val kind = if (this is ChildPortReference) {
                if (isInput) "ChildInputReference"
                else "ChildOutputReference"
            } else {
                if (isInput) "Input"
                else "Output"
            }
            return "$rsRuntime::assembly::PortKind::$kind"
        }

    /** The type of the parameter injected into a reaction for the given dependency. */
    private fun ReactorComponent.toBorrowedType(kind: DepKind): TargetCode =
        when (kind) {
            DepKind.Effects -> "&mut ${toType()}"
            else            -> "&${toType()}"
        }

    /**
     * Produce an instance of the borrowed type ([toBorrowedType]) to inject
     * into a reaction. This conceptually just borrows the field.
     */
    private fun ReactorComponent.toBorrow(kind: DepKind): TargetCode =
        when (kind) {
            DepKind.Effects -> "&mut self.$rustFieldName"
            else            -> "&self.$rustFieldName"
        }

    private fun ReactorComponent.isNotInjectedInReaction(depKind: DepKind, n: ReactionInfo): Boolean =
        // Item is both in inputs and outputs.
        // We must not generate 2 parameters, instead we generate the
        // one with the most permissions (Effects means &mut).
    
        // eg `reaction(act) -> act` must not generate 2 parameters for act,
        // we skip the Trigger one and generate the Effects one.
        depKind != DepKind.Effects && this in n.effects

    /**
     * Whether this component may be unused in a reaction.
     * E.g. actions on which we have just a trigger dependency
     * are fine to ignore.
     */
    private fun ReactorComponent.mayBeUnusedInReaction(depKind: DepKind): Boolean =
        depKind == DepKind.Triggers && this !is PortData


    /** Action that must be taken to cleanup this component within the `cleanup_tag` procedure. */
    private fun ReactorComponent.cleanupAction(): TargetCode? = when (this) {
        is ActionData         ->
            if (isLogical) "ctx.cleanup_logical_action(&mut self.$rustFieldName);"
            else "ctx.cleanup_physical_action(&mut self.$rustFieldName);"
        is PortLike           ->
            if (isGeneratedAsMultiport) "ctx.cleanup_multiport(&mut self.$rustFieldName);"
            else "ctx.cleanup_port(&mut self.$rustFieldName);"
        else                  -> null
    }

    /** Renders the definition of the worker function generated for this reaction. */
    private fun ReactionInfo.toWorkerFunction(): String {
        fun ReactionInfo.reactionParams(): List<String> = sequence {
            for ((kind, comps) in allDependencies) {
                for (comp in comps) {
                    if (comp.isNotInjectedInReaction(kind, this@reactionParams)) continue

                    val param = "${comp.rustRefName}: ${comp.toBorrowedType(kind)}"

                    if (comp.mayBeUnusedInReaction(kind)) {
                        yield("#[allow(unused)] $param")
                    } else {
                        yield(param)
                    }
                }
            }
        }.toList()

        val indent = " ".repeat("fn $workerId(".length)
        return with(PrependOperator) {
            """
                |${loc.lfTextComment()}
                |fn $workerId(&mut self,
                |$indent#[allow(unused)] ctx: &mut $rsRuntime::ReactionCtx,
${"             |$indent"..reactionParams().joinWithCommasLn { it }}) {
${"             |    "..body}
                |}
            """.trimMargin()
        }
    }

    /**
     * A list of parameters that are required for construction
     * but are of internal use to the generator.
     * The widths of port banks referred to by a reactor are
     * saved in here, so that we use the actual runtime value.
     */
    private val ReactorInfo.extraConstructionParams: List<PrivateParamSpec>
        get() {
            val result = mutableListOf<PrivateParamSpec>()

            for (ref in this.portReferences) {
                if (ref.isGeneratedAsMultiport) {
                    result += PrivateParamSpec(
                        ident = ref.widthParamName,
                        type = "usize"
                    )
                }
            }

            return result
        }

    private data class PrivateParamSpec(
        val ident: String,
        val type: TargetCode
    )

    private const val privateParamStruct: String = "PrivateParams"

    private const val privateParamsVarName = "__more_params"
}
