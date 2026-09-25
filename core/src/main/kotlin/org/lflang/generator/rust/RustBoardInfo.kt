package org.lflang.generator.rust

import org.lflang.target.property.PlatformProperty

data class BoardInfo(
    /**
     * This implementation is based around
     * [Embedded-HAL](https://github.com/rust-embedded/embedded-hal).
     */
    val boardHal: String = "",
    val entryMacro: String = "entry",
    val panicHandler: String = "panic_halt",
    val bootloader: List<String> = emptyList(),
    val applicationDescriptor: List<String> = emptyList(),
    val dependencies: Map<String, CargoDependencySpec> = emptyMap(),
    val targetTriple: String = "",
    val linkerScriptTemplate: String? = null,
    val runner: String,
    val rustFlags: List<String> = emptyList(),
    val rustToolChainChannel: String = "Stable"
)

object RustBoardInfo {
    fun getBoardInfo(platformProperty: PlatformProperty.PlatformOptions): BoardInfo {
        return when (platformProperty.board().value()) {
            "esp32"  -> esp32BoardInfo(platformProperty.board().value())
            "rp2350" -> rp2350BoardInfo()
            "rp2040" -> rp2040BoardInfo()
            else     -> throw IllegalArgumentException("Unsupported board type: ${platformProperty.board().value()}")
        }
    }

    private fun esp32BoardInfo(board: String): BoardInfo =
        BoardInfo(
            boardHal = "esp_hal::{self as hal, main}",
            entryMacro = "main",
            panicHandler = "panic_halt",
            bootloader = emptyList(),
            applicationDescriptor = listOf(
                "esp_bootloader_esp_idf::esp_app_desc!()",
            ),
            dependencies = mapOf(
                "esp-hal" to CargoDependencySpec(
                    "~1.1.0",
                    null,
                    null,
                    null,
                    null,
                    listOf(board)
                ),
                "panic-halt" to CargoDependencySpec(
                    "1.0.0",
                    null,
                    null,
                    null,
                    null,
                    emptyList()
                ),
                "esp-bootloader-esp-idf" to CargoDependencySpec(
                    "0.5.0",
                    null,
                    null,
                    null,
                    null,
                    listOf(board)
                ),
            ),
            targetTriple = "xtensa-esp32-none-elf",
            linkerScriptTemplate = null,
            runner = "espflash flash --monitor --chip $board",
            rustFlags = listOf(
                "-C", "link-arg=-nostartfiles"
            ),
            rustToolChainChannel = "esp"
        )

    private fun rp2350BoardInfo(): BoardInfo =
        BoardInfo(
            boardHal = "rp235x_hal::{self as hal, entry};",
            entryMacro = "entry",
            panicHandler = "panic_halt",
            bootloader = listOf(
                "#[link_section = .start_block]",
                "#[used]",
                "pub static IMAGE_DEF: hal::block::ImageDef = hal::block::ImageDef::secure_exe()",
            ),
            applicationDescriptor = listOf(
                "#[link_section = \".bi_entries\"]",
                "#[used]",
                "pub static PICOTOOL_ENTRIES: [hal::binary_info::EntryAddr; 2] = [",
                "    hal::binary_info::rp_cargo_bin_name!(),",
                "    hal::binary_info::rp_cargo_version!(),",
                "]",
            ),
            dependencies = mapOf(),
            targetTriple = "thumbv8m.main-none-eabihf",
            linkerScriptTemplate = null,
            runner = "sudo picotool load -x -t elf",
            rustFlags = listOf(
                "-C", "link-arg=-Tlink.x",
                "-C", "link-arg=--nmagic"
            ),
            rustToolChainChannel = "stable"
        )

    // TODO: test this board once reactor-rt is core
    private fun rp2040BoardInfo(): BoardInfo =
        BoardInfo(
            boardHal = "rp2040_hal::{self as hal, entry}",
            entryMacro = "entry",
            panicHandler = "panic_halt",
            bootloader = listOf(
                "#[link_section = .boot2]",
                "#[used]",
                "pub static BOOT2: [u8; 256] = rp2040_boot2::BOOT_LOADER_GENERIC_03H",
            ),
            applicationDescriptor = emptyList(),
            runner = "sudo picotool load -x -t elf",
            rustFlags = emptyList(),
            rustToolChainChannel = "stable"
        )
}