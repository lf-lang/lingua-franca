package org.lflang.generator.rust

import org.lflang.generator.PrependOperator
import org.lflang.generator.rust.RustBoardInfo.getBoardInfo
import org.lflang.joinWithLn

object RustCoreMainFileEmitter : RustEmitterBase() {

    // TODO: remember to re add after testing run_main
    fun Emitter.makeCoreMainFile(gen: GenerationInfo) {
        val mainReactor = gen.mainReactor
        val mainReactorNames = mainReactor.names
        val boardInfo = getBoardInfo(gen.platform)
        this += with(PrependOperator) {
            """
            |${generatedByComment("//")}
            |#![no_std]
            |#![no_main]
            |#![allow(unused_imports)]
            |#![allow(non_snake_cases)]
            |
            |extern crate core;
            |
            |use ${boardInfo.panicHandler} as _;
            |
            |// Board HAL
            |use ${boardInfo.boardHal};
            |
            |// Board bootloader (required to boot some devices)
${"         |"..boardInfo.bootloader.joinWithLn { it }}
            |
            |// Application descriptor / tooling metadata
${"         |"..boardInfo.applicationDescriptor.joinWithLn { it }};
            |
            |// user dependencies
${"         |"..gen.crate.dependencies.keys.joinWithLn { "use ${it.replace('-', '_')};" }}
            |
            |// user-defined modules
${"         |"..gen.crate.modulesToIncludeInMain.joinWithLn { "mod ${it.fileName.toString().removeSuffix(".rs")};" }}
            |
            |use $rsRuntime;
            |pub use self::reactors::${mainReactorNames.wrapperName} as __MainReactor;
            |pub use self::reactors::${mainReactorNames.paramStructName} as __MainParams;
            |
            |#[${boardInfo.entryMacro}]
            |fn main() -> !  {
            |//   SyncScheduler::run_main::<__MainReactor>(options, main_args);
            |  loop {
            |    let _ = 1 + 1;
            |  }
            |}
            |
        """.trimMargin()
        }
        skipLines(2)

        if (gen.properties.singleFile) {
            makeSingleFileProject(gen)
        } else {
            this += "mod reactors;\n"
        }
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