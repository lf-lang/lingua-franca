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
import org.lflang.TargetProperty.BuildType
import org.lflang.generator.*
import org.lflang.inBlock
import org.lflang.indexInContainer
import org.lflang.inferredType
import org.lflang.isBank
import org.lflang.isInput
import org.lflang.isLogical
import org.lflang.isMultiport
import org.lflang.lf.*
import org.lflang.lf.Timer
import java.nio.file.Path
import java.util.*
import kotlin.text.capitalize

private typealias Ident = String
const val PARALLEL_RT_FEATURE = "parallel-runtime"

/** Root model class for the entire generation. */
data class GenerationInfo(
    val crate: CrateInfo,
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
    /** How the timeout looks like as a Rust expression, eg `Duration::from_millis(40)`. */
    val timeout: TargetCode? = null,
    val timeoutLf: TimeValue? = null,
    val singleFile: Boolean = false,
    /** note: zero means "1 per core" */
    val workers: Int = 0,
    val dumpDependencyGraph: Boolean = false,
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
    /** Other reactor components, like actions, timers, port references, ports. */
    val otherComponents: Set<ReactorComponent>,
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

    /**
     * All timers. This is a subset of [otherComponents].
     */
    val timers: List<TimerData> = otherComponents.filterIsInstance<TimerData>()

    /**
     * Port references that are used by reactions of this reactor,
     * those are synthesized as fields of the current reactor.
     *
     * This is a subset of [otherComponents].
     */
    val portReferences: Set<ChildPortReference> = otherComponents.filterIsInstance<ChildPortReference>().toSet()
}

class TypeParamInfo(
    val targetCode: TargetCode,
    val lfName: Ident,
    val loc: LocationInfo
)

class ReactorNames(
    /** Name of the reactor in LF. By LF conventions, this is a PascalCase identifier. */
    lfName: Ident
) {

    /** Name of the rust module, unescaped.. */
    val modFileName: Ident = lfName.camelToSnakeCase()

    /** Name of the rust module as it can be referenced in rust code. */
    val modName: Ident = modFileName.escapeRustIdent()

    /**
     * Name of the "user struct", which contains state
     * variables as fields, and which the user manipulates in reactions.
     */
    val structName: Ident = lfName.replaceFirstChar { it.uppercase() }.escapeRustIdent()

    // Names of other implementation-detailistic structs.

    val paramStructName: Ident = "${structName}Params"
    val wrapperName: Ident = "${structName}Adapter"
}

data class NestedReactorInstance(
    val lfName: Ident,
    val reactorLfName: String,
    /**
     * Contains arguments for _all_ parameters.
     * The special parameter `bank_index` has the value `"bank_index"`.
     * The map iteration order must be the order in which
     * parameters are declared.
     */
    val args: Map<String, TargetCode>,
    val loc: LocationInfo,
    val typeArgs: List<TargetCode>,
    /** If non-null, this is a reactor bank. */
    val bankWidth: WidthSpec?
) {
    /** Sync with [ChildPortReference.rustChildName]. */
    val rustLocalName = lfName.escapeRustIdent()

    val names = ReactorNames(reactorLfName)
}

/**
 * A reference to a port of a child reactor.
 * This is mostly relevant to [ReactionInfo.allDependencies]
 * and such.
 */
data class ChildPortReference(
    /** Name of the child instance. */
    val childLfName: Ident,
    override val lfName: Ident,
    override val isInput: Boolean,
    override val dataType: TargetCode,
    val widthSpecMultiport: TargetCode?,
    val widthSpecChild: TargetCode?,
) : PortLike() {
    override val isMultiport: Boolean
        get() = widthSpecMultiport != null
    override val isContainedInBank: Boolean get() = widthSpecChild != null
    val rustFieldOnChildName: String = lfName.escapeRustIdent()

    /** Sync with [NestedReactorInstance.rustLocalName]. */
    val rustChildName: TargetCode = childLfName.escapeRustIdent()

    val widthParamName: TargetCode = (rustFieldName + "__width").escapeRustIdent()
}

/**
 * Model class for the parameters of a reactor constructor.
 */
data class CtorParamInfo(
    val lfName: Ident,
    val type: TargetCode,
    val defaultValue: TargetCode?,
    val isTime: Boolean,
    val isList: Boolean,
    val defaultValueAsTimeValue: TimeValue?,
    val documentation: String?
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
    /** The field initializer, a Rust expression. */
    val init: TargetCode
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
    /** Label without quotes. */
    val debugLabel: String?
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
    /** Dependencies of the crate. Note that this includes an entry for the runtime. */
    val dependencies: Map<String, CargoDependencySpec>,
    /** Paths to modules that should be copied to the output & linked into `main.rs`. */
    val modulesToIncludeInMain: List<Path>,
    /** Features to enable in the build command. */
    val enabledCargoFeatures: Set<String>
)


sealed class ReactorComponent {
    /** Simple name of the component in LF. */
    abstract val lfName: Ident

    /**
     * Simple name by which the component can be referred to
     * within target code blocks (usually [lfName]).
     */
    val rustRefName: Ident
        get() =
            if (this is ChildPortReference) "${childLfName}__$lfName"
            else lfName.escapeRustIdent()

    /** Simple name of the field in Rust. */
    val rustFieldName: Ident
        get() = when (this) {
            is TimerData          -> lfName.escapeRustIdent()
            is PortData           -> lfName.escapeRustIdent() // sync with ChildPortReference.rustFieldOnChildName
            is ChildPortReference -> "__${childLfName}__$lfName"
            is ActionData         -> lfName.escapeRustIdent()
        }

    companion object {

        private val DEFAULT_TIME_UNIT_IN_TIMER: TimeUnit = TimeUnit.MILLI

        /**
         * Convert an AST node for a reactor component to the corresponding dependency type.
         * Since there's no reasonable common supertype we use [Variable], but maybe we should
         * have another interface.
         */
        fun from(v: Variable): ReactorComponent = when (v) {
            is Port   -> PortData.from(v)
            is Action -> ActionData(
                lfName = v.name,
                isLogical = v.isLogical,
                dataType = RustTypes.getTargetType(v.type),
                minDelay = (v.minDelay as? Time)?.let(RustTypes::getTargetTimeExpr)
            )
            is Timer  -> TimerData(
                lfName = v.name,
                offset = v.offset.toTimerTimeValue(),
                period = v.period.toTimerTimeValue()
            )
            else      -> throw UnsupportedGeneratorFeatureException("Dependency on ${v.javaClass.simpleName} $v")
        }

        private fun Expression?.toTimerTimeValue(): TargetCode =
            when {
                this == null               -> "Duration::from_millis(0)"
                this is ParameterReference -> "${parameter.name}.clone()"
                this is Literal            -> literal.toIntOrNull()
                    ?.let { TimeValue(it.toLong(), DEFAULT_TIME_UNIT_IN_TIMER).toRustTimeExpr() }
                    ?: throw InvalidLfSourceException("Not an integer literal", this)
                this is Time               -> toRustTimeExpr()
                this is Code               -> toText().inBlock()
                else                       -> RustTypes.getTargetExpr(this, InferredType.time())
            }
    }
}

sealed class PortLike : ReactorComponent() {
    abstract val isInput: Boolean

    abstract val dataType: TargetCode
    val isGeneratedAsMultiport: Boolean
        get() = isMultiport || isContainedInBank
    abstract val isMultiport: Boolean
    abstract val isContainedInBank: Boolean
}

/**
 * @property dataType A piece of target code
 */
data class PortData(
    override val lfName: Ident,
    override val isInput: Boolean,
    /** Rust data type of this component */
    override val dataType: TargetCode,
    // may be a compile-time constant
    val widthSpec: TargetCode?,
) : PortLike() {
    override val isMultiport: Boolean get() = widthSpec != null
    override val isContainedInBank = false

    companion object {
        fun from(port: Port) =
            PortData(
                lfName = port.name,
                isInput = port.isInput,
                dataType = RustTypes.getTargetType(port.type),
                widthSpec = port.widthSpec?.toRustExpr()
            )
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

/** Get the textual representation of a width in Rust code */
fun WidthSpec.toRustExpr(): String = terms.joinToString(" + ") {
    when {
        it.parameter != null -> it.parameter.name
        it.port != null      -> throw UnsupportedGeneratorFeatureException("Width specs that use a port")
        it.code != null      -> it.code.toText().inBlock()
        else                 -> it.width.toString()
    }
}

fun TimeValue.toRustTimeExpr(): TargetCode = RustTypes.getTargetTimeExpr(this)
private fun Time.toRustTimeExpr(): TargetCode = this.toTimeValue().toRustTimeExpr()

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
    fun makeGenerationInfo(targetConfig: TargetConfig, reactors: List<Reactor>, errorReporter: ErrorReporter): GenerationInfo {
        val reactorsInfos = makeReactorInfos(reactors)
        // todo how do we pick the main reactor? it seems like super.doGenerate sets that field...
        val mainReactor = reactorsInfos.lastOrNull { it.isMain } ?: reactorsInfos.last()


        val dependencies = targetConfig.rust.cargoDependencies.toMutableMap()
        dependencies.compute(RustEmitterBase.runtimeCrateFullName) { _, spec ->
            computeDefaultRuntimeConfiguration(spec, targetConfig, errorReporter)
        }

        return GenerationInfo(
            crate = CrateInfo(
                name = mainReactor.lfName.camelToSnakeCase(),
                version = "1.0.0",
                authors = listOf(System.getProperty("user.name")),
                dependencies = dependencies,
                modulesToIncludeInMain = targetConfig.rust.rustTopLevelModules,
                enabledCargoFeatures = targetConfig.rust.cargoFeatures.toSet()
            ),
            reactors = reactorsInfos,
            mainReactor = mainReactor,
            // Rust exec names are snake case, otherwise we get a cargo warning
            // https://github.com/rust-lang/rust/issues/45127
            executableName = mainReactor.lfName.camelToSnakeCase(),
            properties = targetConfig.toRustProperties()
        )
    }

    /**
     * Compute the configuration of the runtime crate, possibly
     * using a user-provided configuration.
     */
    private fun computeDefaultRuntimeConfiguration(
        userSpec: CargoDependencySpec?,
        targetConfig: TargetConfig,
        errorReporter: ErrorReporter
    ): CargoDependencySpec {
        fun CargoDependencySpec.useDefaultRuntimePath() {
            this.localPath = System.getenv("LOCAL_RUST_REACTOR_RT")?.also {
                // Print info to reduce surprise. If the env var is not set,
                // the runtime will be fetched from the internet by Cargo. If
                // the value is incorrect, Cargo will crash.
                errorReporter.reportInfo("Using the Rust runtime from environment variable LOCAL_RUST_REACTOR_RT=$it")
            }

            if (localPath == null) {
                this.gitRepo = RustEmitterBase.runtimeGitUrl
                this.rev = LanguageRuntimeVersions.rustRuntimeVersion
            }
        }

        if (userSpec == null) {
            // default configuration for the runtime crate

            val userRtVersion: String? = targetConfig.runtimeVersion
            // enable parallel feature if asked
            val parallelFeature = listOf(PARALLEL_RT_FEATURE).takeIf { targetConfig.threading }

            val spec = newCargoSpec(
                features = parallelFeature,
            )

            if (targetConfig.externalRuntimePath != null) {
                spec.localPath = targetConfig.externalRuntimePath
            } else if (userRtVersion != null) {
                spec.gitRepo = RustEmitterBase.runtimeGitUrl
                spec.rev = userRtVersion
            } else {
                spec.useDefaultRuntimePath()
            }

            return spec
        } else {
            if (targetConfig.externalRuntimePath != null) {
                userSpec.localPath = targetConfig.externalRuntimePath
            }

            if (userSpec.localPath == null && userSpec.gitRepo == null) {
                userSpec.useDefaultRuntimePath()
            }

            // enable parallel feature if asked
            if (targetConfig.threading) {
                userSpec.features += PARALLEL_RT_FEATURE
            }

            if (!targetConfig.threading && PARALLEL_RT_FEATURE in userSpec.features) {
                errorReporter.reportWarning("Threading cannot be disabled as it was enabled manually as a runtime feature.")
            }

            return userSpec
        }
    }

    private fun TargetConfig.toRustProperties(): RustTargetProperties =
        RustTargetProperties(
            keepAlive = this.keepalive,
            timeout = this.timeout?.toRustTimeExpr(),
            timeoutLf = this.timeout,
            singleFile = this.singleFileProject,
            workers = this.workers,
            dumpDependencyGraph = this.exportDependencyGraph,
        )

    private fun makeReactorInfos(reactors: List<Reactor>): List<ReactorInfo> =
        reactors.map { reactor ->
            val components = mutableMapOf<String, ReactorComponent>()
            val allComponents: List<Variable> = reactor.allComponents()
            for (component in allComponents) {
                val irObj = ReactorComponent.from(component)
                components[irObj.lfName] = irObj
            }

            val reactions = reactor.reactions.map { n: Reaction ->
                fun makeDeps(depKind: Reaction.() -> List<VarRef>): Set<ReactorComponent> =
                    n.depKind().mapTo(mutableSetOf()) {
                        val variable = it.variable
                        val container = it.container
                        if (container is Instantiation && variable is Port) {
                            val formalType = RustTypes.getTargetType(variable.type)
                            ChildPortReference(
                                childLfName = container.name,
                                lfName = variable.name,
                                isInput = variable is Input,
                                dataType = container.reactor.instantiateType(formalType, it.container.typeParms),
                                widthSpecMultiport = variable.widthSpec?.toRustExpr(),
                                widthSpecChild = container.widthSpec?.toRustExpr(),
                            )
                        } else {
                            components[variable.name] ?: throw UnsupportedGeneratorFeatureException(
                                "Dependency on $it"
                            )
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
                    isStartup = n.triggers.any { it is BuiltinTriggerRef && it.type == BuiltinTrigger.STARTUP },
                    isShutdown = n.triggers.any { it is BuiltinTriggerRef && it.type == BuiltinTrigger.SHUTDOWN },
                    debugLabel = AttributeUtils.getLabel(n),
                    loc = n.locationInfo().let {
                        // remove code block
                        it.copy(lfText = it.lfText.replace(TARGET_BLOCK_R, "{= ... =}"))
                    }
                )
            }

            val portReferences =
                reactions.flatMap { it.allDependencies.values }.flatten()
                    .filterIsInstance<ChildPortReference>().toSet()

            ReactorInfo(
                lfName = reactor.name,
                loc = reactor.locationInfo().let {
                    // remove body
                    it.copy(lfText = it.lfText.replace(BLOCK_R, "{ ... }"))
                },
                globalId = reactor.globalId,
                reactions = reactions,
                otherComponents = components.values.toSet() + portReferences,
                isMain = reactor.isMain,
                typeParamList = reactor.typeParms.map {
                    TypeParamInfo(targetCode = it.toText(), it.identifier, it.locationInfo())
                },
                preambles = reactor.preambles.map { it.code.toText() },
                stateVars = reactor.stateVars.map {
                    StateVarInfo(
                        lfName = it.name,
                        type = RustTypes.getTargetType(it.type, it.init),
                        init = RustTypes.getTargetInitializer(it.init, it.type)
                    )
                },
                nestedInstances = reactor.instantiations.map { it.toModel() },
                connections = reactor.connections,
                ctorParams = reactor.parameters.map {
                    CtorParamInfo(
                        lfName = it.name,
                        type = RustTypes.getTargetType(it.type, it.init),
                        defaultValue = RustTypes.getTargetInitializer(it.init, it.type),
                        documentation = null, // todo
                        isTime = it.inferredType.isTime,
                        isList = it.inferredType.isList,
                        defaultValueAsTimeValue = ASTUtils.getDefaultAsTimeValue(it),
                    )
                }
            )
        }

    private fun Instantiation.toModel(): NestedReactorInstance {

        val byName = parameters.associateBy { it.lhs.name }
        val args = reactor.parameters.associate { ithParam ->
            // use provided argument
            val value = byName[ithParam.name]?.let { RustTypes.getTargetInitializer(it.rhs, ithParam.type) }
                ?: if (ithParam.name == "bank_index" && this.isBank) "bank_index" else null // special value
                ?: ithParam?.let { RustTypes.getTargetInitializer(it.init, it.type) }
                ?: throw InvalidLfSourceException(
                    "Cannot find value of parameter ${ithParam.name}",
                    this
                )
            ithParam.name to value
        }

        return NestedReactorInstance(
            lfName = this.name,
            args = args,
            reactorLfName = this.reactorClass.name,
            loc = this.locationInfo(),
            typeArgs = typeParms.map { it.toText() },
            bankWidth = this.widthSpec
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
fun Reactor.instantiateType(formalType: TargetCode, typeArgs: List<TypeParm>): TargetCode {
    val typeParams = typeParms
    assert(typeArgs.size == typeParams.size)

    return if (typeArgs.isEmpty()) formalType
    else {
        val typeArgsByName = typeParams.mapIndexed { i, tp -> Pair(tp.identifier, typeArgs[i].toText()) }.toMap()

        CodeMap.fromGeneratedCode(formalType).generatedCode.replace(IDENT_REGEX) {
            typeArgsByName[it.value] ?: it.value
        }
    }
}

/**
 * Returns the identifier of this type param.
 */
private val TypeParm.identifier: String
    get() {
        val targetCode = CodeMap.fromGeneratedCode(toText()).generatedCode
        return IDENT_REGEX.find(targetCode.trimStart())?.value
            ?: throw InvalidLfSourceException(
                "No identifier in type param `$targetCode`",
                this
            )
    }

/**
 * Returns the name of the profile for Cargo (how it is
 * declared in `Cargo.toml`).
 */
val BuildType.cargoProfileName: String
    get() = when (this) {
        BuildType.DEBUG             -> "debug"
        BuildType.TEST              -> "test"
        BuildType.RELEASE           -> "release"
        BuildType.REL_WITH_DEB_INFO -> "release-with-debug-info"
        BuildType.MIN_SIZE_REL      -> "release-with-min-size"
    }

/** Just the constructor of [CargoDependencySpec], but allows using named arguments. */
fun newCargoSpec(
    version: String? = null,
    gitRepo: String? = null,
    rev: String? = null,
    gitTag: String? = null,
    localPath: String? = null,
    features: List<String>? = null,
) = CargoDependencySpec(version, gitRepo, rev, gitTag, localPath, features)
