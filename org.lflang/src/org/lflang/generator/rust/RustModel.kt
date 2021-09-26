/*
 * Copyright (c) 2021, TU Dresden.

 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:

 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.

 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.

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
import org.lflang.generator.*
import org.lflang.lf.*
import org.lflang.lf.Timer
import java.util.*

private typealias Ident = String

/** Root model class for the entire generation. */
data class GenerationInfo(
    val crate: CrateInfo,
    val runtime: RuntimeInfo,
    val reactors: List<ReactorInfo>,
    val mainReactor: ReactorInfo, // it's also in the list
    val executableName: Ident,
    val properties: RustTargetProperties
) {

    private val byId = reactors.associateBy { it.globalId }

    fun getReactor(id: ReactorId): ReactorInfo =
        byId[id] ?: throw IllegalArgumentException("No such recorded ID: $id, I know ${byId.keys}")
}

data class RustTargetProperties(
    val keepAlive: Boolean = false,
    val timeout: TargetCode? = null,
    val singleFile: Boolean = false
)

/**
 * Model class for a reactor class. This will be emitted as
 * several structs in a Rust module.
 */
data class ReactorInfo(
    /** Name of the reactor in LF. By LF conventions, this is a PascalCase identifier. */
    val lfName: Ident,
    /** Global ID. */
    val globalId: ReactorId,
    /** Whether this is the main reactor. */
    val isMain: Boolean,
    /** A list of reactions, in order of their [ReactionInfo.idx]. */
    val reactions: List<ReactionInfo>,
    /** List of state variables. */
    val stateVars: List<StateVarInfo>,
    /** Other reactor components, like actions & timers. */
    val otherComponents: List<ReactorComponent>,
    /** List of ctor parameters. */
    val ctorParams: List<CtorParamInfo>,
    /** List of preambles, will be outputted at the top of the file. */
    val preambles: List<TargetCode>,

    val typeParamList: List<TypeParamInfo>,

    /**
     * List of reactor instances that are directly created by this reactor.
     * Each of them is a named item accessible from the assemble method (well,
     * their ports are anyway).
     */
    val nestedInstances: List<NestedReactorInstance>,

    /**
     * List of connections between ports that are made within
     * the body of this reactor instance.
     *
     * todo Connection doesn't have its own model class
     */
    val connections: List<Connection>,
    val loc: LocationInfo,

    ) {
    /** Identifiers for the different Rust constructs that this reactor class generates. */
    val names = ReactorNames(lfName)
    val timers: List<TimerData> = otherComponents.filterIsInstance<TimerData>()

    /**
     * Port references that are used by reactions of this reactor,
     * those are synthesized as fields of the current reactor.
     */
    val portReferences: Set<ChildPortReference> =
        reactions.flatMap { it.allDependencies.values }.flatten().filterIsInstance<ChildPortReference>().toSet()


    /**
     * Returns the type of the port with the given name, when
     * this reactor's type parameters are instantiated with
     * the given type arguments. For instance, in the following:
     *
     *     reactor Generic<T> { input port: Vec<T>; }
     *     reactor Container { gen = new Generic<String>(); }
     *
     * the type of `gen.port` is `Vec<String>`. If a reaction of
     * `Container` depends on `gen.port`, it needs to be injected
     * with the correct type. The correct call to this method
     * would be `generic.typeOfPort("port", listOf("String"))`.
     *
     */
    fun typeOfPort(portName: Ident, typeArgs: List<TargetCode>): TargetCode {
        assert(typeArgs.size == typeParamList.size)
        val port = otherComponents.filterIsInstance<PortData>().first { it.lfName == portName }

        return if (typeArgs.isEmpty()) port.dataType
        else port.dataType.replace(IDENT_REGEX) { match ->
            typeParamList.indexOfFirst { it.lfName == match.value }
                .takeIf { it >= 0 }
                ?.let { typeArgs[it] }
                ?: match.value
        }
    }
}

class TypeParamInfo(
    val targetCode: TargetCode,
    val lfName: Ident,
    val loc: LocationInfo
) {
}

class ReactorNames(
    /** Name of the reactor in LF. By LF conventions, this is a PascalCase identifier. */
    private val lfName: Ident
) {

    /** Name of the rust module, unescaped.. */
    val modFileName: Ident = lfName.camelToSnakeCase()

    /** Name of the rust module as it can be referenced in rust code. */
    val modName: Ident = modFileName.escapeRustIdent()

    /**
     * Name of the "user struct", which contains state
     * variables as fields, and which the user manipulates in reactions.
     */
    val structName: Ident = lfName.capitalize().escapeRustIdent()

    // Names of other implementation-detailistic structs.

    val paramStructName: Ident = "${structName}Params"
    val wrapperName: Ident = "${structName}Adapter"
}

data class NestedReactorInstance(
    val lfName: Ident,
    val reactorLfName: String,
    val args: Map<String, TargetCode>,
    val loc: LocationInfo,
    val typeArgs: List<TargetCode>
) {
    val rustLocalName = lfName.escapeRustIdent()

    val names = ReactorNames(reactorLfName)
}

/**
 * Model class for the parameters of a reactor constructor.
 */
data class CtorParamInfo(
    val lfName: Ident,
    val type: TargetCode,
    val defaultValue: TargetCode?
)

/** Model class for a state variable. */
data class StateVarInfo(
    /**
     * Identifier of the state var. From within a reaction
     * body, the state var is accessible as a field with type
     * [type] declared on the `self` struct.
     */
    val lfName: String,
    /** Rust static type of the struct field. Must be `Sized`. */
    val type: TargetCode,
    /**
     * The field initializer, a Rust expression. If null,
     * will default to `Default::default()`.
     */
    val init: TargetCode?
)

enum class DepKind { Triggers, Uses, Effects }

/**
 * Model class for a single reaction.
 */
data class ReactionInfo(
    /** Index in the containing reactor. */
    val idx: Int,
    /** Target code for the reaction body. */
    val body: TargetCode,

    val allDependencies: Map<DepKind, Set<ReactorComponent>>,

    /** Whether the reaction is triggered by the startup event. */
    val isStartup: Boolean,
    /** Whether the reaction is triggered by the shutdown event. */
    val isShutdown: Boolean,

    /** Location metadata. */
    val loc: LocationInfo,

    val debugLabel: String? = null
) {

    /** Dependencies that trigger the reaction. */
    val triggers: Set<ReactorComponent> get() = allDependencies[DepKind.Triggers].orEmpty()

    /** Dependencies that the reaction may use, but which do not trigger it. */
    val uses: Set<ReactorComponent> get() = allDependencies[DepKind.Uses].orEmpty()

    /** Dependencies that the reaction may write to/schedule. */
    val effects: Set<ReactorComponent> get() = allDependencies[DepKind.Effects].orEmpty()


    // those are implementation details

    /** The name of the worker function for this reaction. */
    val workerId: String = "react_$idx"

    /** The name of the `ReactionInvoker` field for this reaction. */
    val invokerId: String = "react_$idx"
}

/**
 * Metadata about the generated crate. Mostly doesn't matter.
 */
data class CrateInfo(
    /** Name of the crate. According to Rust conventions this should be a snake_case name. */
    val name: String,
    /** Version of the crate. */
    val version: String,
    /** List of names of the credited authors. */
    val authors: List<String>,
)

data class RuntimeInfo(
    val version: String?,
    val localPath: String?
)

/*
TODO do we really need the following classes?
  - I quite like that they are much simpler than the corresponding AST nodes (Input, Action, etc).
  - Also we have control over them. AST classes are generated by Xtend.
  - OTOH they're just a subset of the functionality of AST nodes for now. We could as well use them.
   But this would make this "IR" a weird mix between data classes and AST nodes.
  - Maybe the right data structures to use are the ReactorInstance, etc instead. But I feel like we're
   generating code not for the instance tree, but for now, just generic code for each reactor (like the C++
   generator)
 */


sealed class ReactorComponent {
    /** Simple name of the component in LF. */
    abstract val lfName: Ident

    /**
     * Simple name by which the component can be referred to
     * within target code blocks (usually [lfName]).
     */
    val rustRefName: Ident
        get() =
            if (this is ChildPortReference) "${childName}__$lfName"
            else lfName.escapeRustIdent()

    /** Simple name of the field in Rust. */
    val rustFieldName: Ident
        get() = when (this) {
            is TimerData          -> "__$lfName"
            is PortData           -> "__$lfName" // sync with ChildPortReference.rustFieldOnChildName
            is ChildPortReference -> "__${childName}__$lfName"
            is ActionData         -> "__$lfName"
        }

    companion object {

        private val DEFAULT_TIME_UNIT_IN_TIMER: TimeUnit = TimeUnit.MSEC

        /**
         * Convert an AST node for a reactor component to the corresponding dependency type.
         * Since there's no reasonable common supertype we use [Variable], but maybe we should
         * have another interface.
         */
        fun from(v: Variable): ReactorComponent? = when (v) {
            is Port   -> PortData.from(v)
            is Action -> ActionData(
                lfName = v.name,
                isLogical = v.isLogical,
                dataType = RustTypes.getTargetType(v.type),
                minDelay = v.minDelay?.time?.let(RustTypes::getTargetTimeExpr)
            )
            is Timer  -> TimerData(
                lfName = v.name,
                offset = v.offset.toTimerTimeValue(),
                period = v.period.toTimerTimeValue()
            )
            else      -> throw UnsupportedGeneratorFeatureException("Dependency on ${v.javaClass.simpleName} $v")
        }

        private fun Value?.toTimerTimeValue(): TargetCode =
            when {
                this == null      -> "Duration::from_millis(0)"
                parameter != null -> "${parameter.name}.clone()"
                literal != null   ->
                    literal.toIntOrNull()
                        ?.let { toRustTimeExpr(it.toLong(), DEFAULT_TIME_UNIT_IN_TIMER) }
                        ?: throw InvalidSourceException("Not an integer literal", this)
                time != null      -> time.toRustTimeExpr()
                code != null      -> code.toText()
                else              -> RustTypes.getTargetExpr(this, InferredType.time())
            }
    }
}


/**
 * @property dataType A piece of target code
 */
data class PortData(
    override val lfName: Ident,
    val isInput: Boolean,
    /** Rust data type of this component */
    val dataType: TargetCode,
    val isMultiport: Boolean = false
) : ReactorComponent() {
    companion object {
        fun from(port: Port) =
            PortData(
                lfName = port.name,
                isInput = port.isInput,
                dataType = RustTypes.getTargetType(port.type)
            )
    }
}

/**
 * A reference to a port of a child reactor.
 * This is mostly relevant to [ReactionInfo.allDependencies]
 * and such.
 */
data class ChildPortReference(
    /** Name of the child instance. */
    val childName: Ident,
    override val lfName: Ident,
    val isInput: Boolean,
    /** ID of the containing reactor. */
    val containerId: ReactorId,
    val reactorTypeArgs: List<TargetCode>,
    /** */
    val dataType: TargetCode
) : ReactorComponent() {
    val rustFieldOnChildName: String get() = "__$lfName"


    fun dataType(gen: GenerationInfo): TargetCode {
        val reactor = gen.getReactor(containerId)
        return reactor.typeOfPort(lfName, reactorTypeArgs)
    }
}

data class ActionData(
    override val lfName: Ident,
    // if null, defaults to unit
    val dataType: TargetCode?,
    val isLogical: Boolean,
    val minDelay: TargetCode?,
) : ReactorComponent()

data class TimerData(
    override val lfName: Ident,
    val offset: TargetCode,
    val period: TargetCode,
) : ReactorComponent()

private fun TimeValue.toRustTimeExpr(): TargetCode = toRustTimeExpr(time, unit)
private fun Time.toRustTimeExpr(): TargetCode = toRustTimeExpr(interval.toLong(), unit)

private fun toRustTimeExpr(interval: Long, unit: TimeUnit): TargetCode =
    RustTypes.getTargetTimeExpression(interval, unit)

/** Regex to match a target code block, captures the insides as $1. */
private val TARGET_BLOCK_R = Regex("\\{=(.*)=}", RegexOption.DOT_MATCHES_ALL)
/** Regex to match a simple (C) code block, captures the insides as $1. */
private val BLOCK_R = Regex("\\{(.*)}", RegexOption.DOT_MATCHES_ALL)

/**
 * Produce model classes from the AST.
 */
object RustModelBuilder {

    /**
     * Given the input to the generator, produce the model classes.
     */
    fun makeGenerationInfo(targetConfig: TargetConfig, reactors: List<Reactor>): GenerationInfo {
        val reactorsInfos = makeReactorInfos(reactors)
        // todo how do we pick the main reactor? it seems like super.doGenerate sets that field...
        val mainReactor = reactorsInfos.lastOrNull { it.isMain } ?: reactorsInfos.last()

        return GenerationInfo(
            crate = CrateInfo(
                name = mainReactor.lfName.camelToSnakeCase(),
                version = "1.0.0",
                authors = listOf(System.getProperty("user.name"))
            ),
            runtime = RuntimeInfo(
                version = targetConfig.runtimeVersion,
                localPath = targetConfig.externalRuntimePath
            ),
            reactors = reactorsInfos,
            mainReactor = mainReactor,
            // Rust exec names are snake case, otherwise we get a cargo warning
            // https://github.com/rust-lang/rust/issues/45127
            executableName = mainReactor.lfName.camelToSnakeCase(),
            properties = targetConfig.toRustProperties()
        )
    }

    private fun TargetConfig.toRustProperties(): RustTargetProperties =
        RustTargetProperties(
            keepAlive = this.keepalive,
            timeout = this.timeout?.toRustTimeExpr(),
            singleFile = this.singleFileProject
        )

    private fun makeReactorInfos(reactors: List<Reactor>): List<ReactorInfo> =
        reactors.map { reactor ->
            val components = mutableMapOf<String, ReactorComponent>()
            val allComponents: List<Variable> = reactor.allComponents()
            for (component in allComponents) {
                val irObj = ReactorComponent.from(component) ?: continue
                components[irObj.lfName] = irObj
            }

            val reactions = reactor.reactions.map { n: Reaction ->
                fun makeDeps(depKind: Reaction.() -> List<VarRef>): Set<ReactorComponent> =
                    n.depKind().mapTo(mutableSetOf()) {
                        val variable = it.variable
                        val container = it.container
                        if (container is Instantiation && variable is Port) {
                            ChildPortReference(
                                childName = container.name,
                                containerId = container.reactor.globalId,
                                lfName = variable.name,
                                isInput = variable is Input,
                                reactorTypeArgs = container.typeParms.map { it.toText() },
                                dataType = RustTypes.getTargetType(variable.type),
                            )
                        } else {
                            components[variable.name] ?: throw UnsupportedGeneratorFeatureException("Dependency on $it")
                        }
                    }

                ReactionInfo(
                    idx = n.indexInContainer,
                    allDependencies = EnumMap<DepKind, Set<ReactorComponent>>(DepKind::class.java).apply {
                        this[DepKind.Triggers] = makeDeps { triggers.filterIsInstance<VarRef>() }
                        this[DepKind.Uses] = makeDeps { sources }
                        this[DepKind.Effects] = makeDeps { effects }
                    },
                    body = n.code.toText(),
                    isStartup = n.triggers.any { it.isStartup },
                    isShutdown = n.triggers.any { it.isShutdown },
                    loc = n.locationInfo().let {
                        // remove code block
                        it.copy(lfText = it.lfText.replace(TARGET_BLOCK_R, "{= ... =}"))
                    }
                )
            }

            ReactorInfo(
                lfName = reactor.name,
                loc = reactor.locationInfo().let {
                    // remove body
                    it.copy(lfText = it.lfText.replace(BLOCK_R, "{ ... }"))
                },
                globalId = reactor.globalId,
                reactions = reactions,
                otherComponents = components.values.toList(),
                isMain = reactor.isMain,
                typeParamList = reactor.typeParms.mapIndexed { i, tp ->
                    val targetCode = tp.toText()
                    val ident = IDENT_REGEX.find(targetCode.trimStart())?.value
                        ?: throw InvalidSourceException("No identifier in type param `$targetCode`", tp)
                    TypeParamInfo(targetCode = tp.toText(), ident, tp.locationInfo())
                },
                preambles = reactor.preambles.map { it.code.toText() },
                stateVars = reactor.stateVars.map {
                    StateVarInfo(
                        lfName = it.name,
                        type = RustTypes.getTargetType(it.type, it.init),
                        init = RustTypes.getTargetInitializer(it.init, it.type, initWithBraces = it.braces.isNotEmpty())
                    )
                },
                nestedInstances = reactor.instantiations.map { it.toModel() },
                connections = reactor.connections,
                ctorParams = reactor.parameters.map {
                    CtorParamInfo(
                        lfName = it.name,
                        type = RustTypes.getTargetType(it.type, it.init),
                        defaultValue = RustTypes.getTargetInitializer(it.init, it.type, initWithBraces = it.braces.isNotEmpty())
                    )
                }
            )
        }

    private fun Instantiation.toModel(): NestedReactorInstance {

        val byName = parameters.associateBy { it.lhs.name }
        val args = reactor.parameters.associate { ithParam ->
            // use provided argument
            val value = byName[ithParam.name]?.let {
                RustTypes.getTargetInitializer(it.rhs, ithParam.type, it.isInitWithBraces)
            }
                ?: ithParam?.let { RustTypes.getTargetInitializer(it.init, it.type, it.isInitWithBraces) }
                ?: throw InvalidSourceException("Cannot find value of parameter ${ithParam.name}", this)
            ithParam.name to value
        }

        return NestedReactorInstance(
            lfName = this.name,
            args = args,
            reactorLfName = this.reactorClass.name,
            loc = this.locationInfo(),
            typeArgs = typeParms.map { it.toText() }
        )
    }
}

/**
 * An identifier for a [Reactor]. Used to associate [Reactor]
 * with model objects and vice versa.
 */
class ReactorId(private val id: String) {
    override fun equals(other: Any?): Boolean {
        if (this === other) return true
        if (javaClass != other?.javaClass) return false
        other as ReactorId
        return id == other.id
    }

    override fun hashCode(): Int = id.hashCode()
    override fun toString(): String = id
}

/**
 * Returns a string that is distinct from the globalId
 * of any other reactor. Equal reactors yield the same
 * globalID. This does not need to be stable across runs.
 */
val Reactor.globalId: ReactorId
    get() = ReactorId(this.eResource().toPath().toString() + "/" + this.name + "/" + "/" + this.isMain)

