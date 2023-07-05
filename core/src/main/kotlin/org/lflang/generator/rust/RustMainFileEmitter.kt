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

import org.lflang.camelToSnakeCase
import org.lflang.escapeStringLiteral
import org.lflang.generator.PrependOperator
import org.lflang.generator.PrependOperator.rangeTo
import org.lflang.generator.UnsupportedGeneratorFeatureException
import org.lflang.joinWithCommasLn
import org.lflang.joinWithLn
import org.lflang.withoutQuotes


/**
 * Emits the entry point for the generated program.
 *
 * @author Clément Fournier - TU Dresden, INSA Rennes
 */
object RustMainFileEmitter : RustEmitterBase() {

    fun Emitter.makeMainFile(gen: GenerationInfo) {
        val mainReactor = gen.mainReactor
        val mainReactorNames = mainReactor.names
        this += with(PrependOperator) {
            """
            |${generatedByComment("//")}
            |#![allow(unused_imports)]
            |#![allow(non_snake_case)]
            |
            |extern crate env_logger;
            |#[macro_use]
            |extern crate log;
            |
            |// user dependencies
${"         |"..gen.crate.dependencies.keys.joinWithLn { "extern crate ${it.replace('-', '_')};" }}
            |
            |// user-defined modules
${"         |"..gen.crate.modulesToIncludeInMain.joinWithLn { "mod ${it.fileName.toString().removeSuffix(".rs")};" }}
            |
            |use $rsRuntime::*;
            |use log::LevelFilter;
            |pub use self::reactors::${mainReactorNames.wrapperName} as __MainReactor;
            |pub use self::reactors::${mainReactorNames.paramStructName} as __MainParams;
            |
            |struct CliParseResult(SchedulerOptions, __MainParams, LevelFilter);
            |
            |const DEFAULT_LOG_LEVEL: LevelFilter = LevelFilter::Warn;
            |
            |fn main() {
            |    let CliParseResult(options, main_args, log_level) = cli::parse();
            |
            |    init_logger(log_level);
            |
            |    SyncScheduler::run_main::<__MainReactor>(options, main_args);
            |}
            |
            |fn init_logger(level: LevelFilter) {
            |    if !cfg!(debug_assertions) && level < LevelFilter::Info {
            |        warn!("Log level {} is not active because this application was built in release mode.", level);
            |        warn!("Use a debug build to enable this level.");
            |        warn!("In Lingua France, use the target property `build-type: Debug` (the default).");
            |    }
            |
            |    let mut builder = env_logger::Builder::from_env(env_logger::Env::default());
            |    builder.format_target(false);
            |    builder.filter_level(level);
            |    builder.init();
            |}
            |
        """.trimMargin()
        }

        skipLines(2)

        makeCliModule(gen)

        skipLines(2)

        if (gen.properties.singleFile) {
            makeSingleFileProject(gen)
        } else {
            this += "mod reactors;\n"
        }
    }

    private fun Emitter.makeCliModule(gen:GenerationInfo) {
        val mainReactor = gen.mainReactor

        val defaultTimeOutAsStr = gen.properties.timeoutLf?.toString() ?: "0"
        val defaultTimeOutAsRust = gen.properties.timeoutLf?.toRustTimeExpr().toRustOption()

        this += """
            |#[cfg(not(feature="cli"))]
            |mod cli {
            |    use $rsRuntime::*;
            |    use super::*;
            |    use std::str::FromStr;
            |
            |    /// Fallback implementation which doesn't parse parameters.
            |    pub(super) fn parse() -> CliParseResult {
            |        if std::env::args().len() > 1 {
            |           warn!("CLI arguments are ignored, as the program was built without the \"cli\" feature.");
            |           warn!("In Lingua Franca, use the target property `cargo-features: [\"cli\"]`.");
            |           warn!("Proceeding with defaults defined at compile time.");
            |        }
            |
            |        let mut options = SchedulerOptions::default();
            |        options.timeout = $defaultTimeOutAsRust;
            |        options.keep_alive = ${gen.properties.keepAlive};
            |        options.threads = ${gen.properties.workers}; // note: zero means "1 per core"
            |        options.dump_graph = ${gen.properties.dumpDependencyGraph};

            |        // main params are entirely defaulted
            |        let main_args = __MainParams::new(
${"         |           "..mainReactor.ctorParams.joinWithCommasLn { (it.defaultValue ?: "Default::default()") }}
            |        );
            |
            |        let level_by_env = std::env::var("RUST_LOG").ok().and_then(|e| e.as_str().parse::<::log::LevelFilter>().ok());
            |        let log_level = level_by_env.unwrap_or(DEFAULT_LOG_LEVEL);
            |
            |        CliParseResult(options, main_args, log_level)
            |    }
            |}
            |
            |#[cfg(feature="cli")]
            |mod cli {
            |    use $rsRuntime::*;
            |    use super::*;
            |    use clap::Parser;
            |    use std::str::FromStr;
            |
            |
            |    // these aliases are needed because clap interprets literal
            |    // occurrences of bool and Option.
            |    type BoolAlias = bool;
            |    type OptionAlias<T> = Option<T>;
            |
            |    #[derive(Debug, Parser)]
            |    #[clap(name = "${gen.executableName}")]
            |    struct Opt {
            |
            |        /// Whether to keep the program alive when the event queue is empty.
            |        /// This is only useful when physical actions are used, as they may
            |        /// push new asynchronous events.
            |        #[clap(long, default_value="${gen.properties.keepAlive}", help_heading=Some("RUNTIME OPTIONS"), value_name("bool"),)]
            |        keep_alive: BoolAlias,
            |
            |        /// Timeout for the program. A value of zero means no timeout. The
            |        /// timeout is in logical time, it means, no event past this tag will
            |        /// be processed. Notice that the program may shutdown earlier because
            |        /// of a call to request_stop.
            |        #[clap(long, default_value="$defaultTimeOutAsStr", parse(try_from_str = try_parse_duration), help_heading=Some("RUNTIME OPTIONS"), value_name("time"),)]
            |        timeout: OptionAlias<Duration>,
            |
            |        /// Number of workers to use to execute reactions in parallel. A value
            |        /// of zero means that the runtime will select a value depending on the
            |        /// number of cores available on the machine.
            |        /// This option is **ignored** unless the runtime crate has been built
            |        /// with the feature `parallel-runtime`.
            |        #[clap(long, default_value="${gen.properties.workers}", help_heading=Some("RUNTIME OPTIONS"), value_name("usize"),)]
            |        workers: usize,
            |
            |        /// Export the dependency graph in DOT format before starting execution.
            |        #[clap(long, help_heading=Some("RUNTIME OPTIONS"),)]
            |        export_graph: bool,
            |
            |        /// Minimum logging level for the runtime. Specifying the log level on the
            |        /// command-line overrides the environment variable RUST_LOG. Note that release
            |        /// builds of the runtime are stripped of trace and debug logs, so only up
            |        /// to info can be enabled then. To trace programs, use a debug build.
            |        #[clap(long,
            |           possible_values(&["trace", "debug", "info", "warn", "error", "off"]),
            |           env = "RUST_LOG",
            |           hide_env_values = true,
            |           default_value="warn",
            |           value_name("level"),
            |           help_heading=Some("RUNTIME OPTIONS"),
            |        )]
            |        log_level: LevelFilter,
            |
${"         |        "..mainReactor.ctorParams.joinWithCommasLn { it.toCliParam() }}
            |    }
            |
            |    pub(super) fn parse() -> CliParseResult {
            |        let opts = Opt::parse();
            |
            |        let mut options = SchedulerOptions::default();
            |        options.timeout = opts.timeout;
            |        options.keep_alive = opts.keep_alive;
            |        options.threads = opts.workers;
            |        options.dump_graph = opts.export_graph;
            |
            |        let main_args = __MainParams::new(
${"         |           "..mainReactor.ctorParams.joinWithCommasLn { "opts." + it.cliParamName }}
            |        );
            |
            |        // Note here we return always Some(LogLevel), because clap supports reading RUST_LOG env var
            |        // Note that there is an inconsistency here, as clap will just parse the simple values,
            |        // while env_logger supports more complicated patterns for this env var.
            |
            |        CliParseResult(options, main_args, opts.log_level)
            |    }
            |
            |    fn try_parse_duration(t: &str) -> ::std::result::Result<Option<Duration>, String> {
            |        reactor_rt::try_parse_duration(t).map(|d| {
            |            if d.is_zero() { None } else { Some(d) }
            |        })
            |    }
            |}
        """.trimMargin()
    }

    private val CtorParamInfo.cliParamName get() = "main_" + lfName.camelToSnakeCase()

    private fun CtorParamInfo.toCliParam() = buildString {
        documentation?.lines()?.map { "///$it" }?.forEach { appendLine(it) }
        append("#[clap(long, help_heading=Some(\"MAIN REACTOR PARAMETERS\"), ")

        if (isList) // todo this should be supported but test it
            throw UnsupportedGeneratorFeatureException("main parameters with list types")

        if (defaultValueAsTimeValue != null)
            append("default_value=\"").append(defaultValueAsTimeValue).append("\", ")
        else if (defaultValue != null)
            append("default_value_t=").append(defaultValue.withoutQuotes()).append(", ")
        else
            append("required=true, ")

        if (isTime) {
            append("parse(try_from_str = $rsRuntime::try_parse_duration), value_name(\"time\"),")
        } else {
            append("value_name(\"${type.escapeStringLiteral()}\"),")
        }

        appendLine(")]")
        append(cliParamName).append(": ").append(type)
    }

    private fun Emitter.makeSingleFileProject(gen: GenerationInfo) {
        this += """
            |//-------------------//
            |//---- REACTORS -----//
            |//-------------------//
            |
        """.trimMargin()

        this.writeInBlock("mod reactors {") {
            for (reactor in gen.reactors) {
                this += with(reactor.names) {
                    """
                pub use self::$modName::$wrapperName;
                pub use self::$modName::$paramStructName;
                """.trimIndent()
                }
                skipLines(1)
            }

            for (reactor in gen.reactors) {
                this += """
                    |//--------------------------------------------//
                    |//------------ ${reactor.lfName} -------//
                    |//-------------------//
                    """.trimMargin()

                this.writeInBlock("mod ${reactor.names.modName} {") {
                    with(RustReactorEmitter) { emitReactorModule(reactor) }
                }
                this.skipLines(2)
            }
        }
    }

}
