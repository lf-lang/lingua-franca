package org.lflang.generator.rust

import org.lflang.generator.PrependOperator
import org.lflang.generator.rust.RustBoardInfo.getBoardInfo
import org.lflang.joinWithCommasLn
import org.lflang.joinWithLn

/**
 * Emits `main.rs` for programs that run on a microcontroller (`platform: { board: ... }`).
 *
 * The program is `no_std` and runs on [Embassy](https://embassy.dev):
 * `main` is an embassy task that sets up the board, then awaits the
 * runtime's async scheduler until the program shuts down.
 */
object RustCoreMainFileEmitter : RustEmitterBase() {

    fun Emitter.makeCoreMainFile(gen: GenerationInfo) {
        val mainReactor = gen.mainReactor
        val mainReactorNames = mainReactor.names
        val board = getBoardInfo(gen.platform)
        val timeout = gen.properties.timeoutLf?.toRustTimeExpr().toRustOption()

        this += with(PrependOperator) {
            """
            |${generatedByComment("//")}
            |#![no_std]
            |#![no_main]
            |#![allow(unused_imports)]
            |#![allow(non_snake_case)]
            |
            |extern crate alloc;
            |
            |use panic_halt as _;
            |
            |// user dependencies
${"         |"..gen.crate.dependencies.keys.joinWithLn { "extern crate ${it.replace('-', '_')};" }}
            |
            |// user-defined modules
${"         |"..gen.crate.modulesToIncludeInMain.joinWithLn { "mod ${it.fileName.toString().removeSuffix(".rs")};" }}
            |
            |use $rsRuntime::*;
            |pub use self::reactors::${mainReactorNames.wrapperName} as __MainReactor;
            |pub use self::reactors::${mainReactorNames.paramStructName} as __MainParams;
            |
            |// board: ${board.name}
${"         |"..board.preamble}
            |
            |#[${board.mainAttribute}]
            |async fn main(_spawner: embassy_executor::Spawner) {
${"         |    "..board.init}
            |
            |    let mut options = SchedulerOptions::default();
            |    options.timeout = $timeout;
            |    options.keep_alive = ${gen.properties.keepAlive};
            |
            |    // main params are entirely defaulted
            |    let main_args = __MainParams::new(
${"         |        "..mainReactor.ctorParams.joinWithCommasLn { it.defaultValue ?: "Default::default()" }}
            |    );
            |
            |    SyncScheduler::run_main_async::<__MainReactor>(options, main_args).await;
            |
            |    // The program has shut down. There is nothing to return to, so idle forever.
            |    loop {
            |        core::future::pending::<()>().await;
            |    }
            |}
            |
        """.trimMargin()
        }
        skipLines(2)

        if (gen.properties.singleFile) {
            with(RustMainFileEmitter) { makeSingleFileProject(gen) }
        } else {
            this += "mod reactors;\n"
        }
    }
}
