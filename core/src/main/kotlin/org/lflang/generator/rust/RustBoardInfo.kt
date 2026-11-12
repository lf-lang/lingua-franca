package org.lflang.generator.rust

import org.lflang.target.property.PlatformProperty

/**
 * Everything the generator needs to know to build a reactor program
 * for one microcontroller board.
 *
 * Boards run on [Embassy](https://embassy.dev): the runtime is built
 * with its `embassy` feature, and the generated `main` is an async
 * embassy task that awaits `SyncScheduler::run_main_async`. Each board
 * must provide an embassy executor, an embassy time driver, a global
 * allocator and a critical-section implementation.
 */
data class BoardInfo(
    /** Board name, as written in the `platform: { board: ... }` target property. */
    val name: String,
    /** Rust target triple to build for. */
    val targetTriple: String,
    /** Channel written to `rust-toolchain.toml`. */
    val toolchainChannel: String,
    /** Targets written to `rust-toolchain.toml`. Empty for custom toolchains like `esp`. */
    val toolchainTargets: List<String> = emptyList(),
    /** Crates for `-Z build-std`. Empty if the target has a prebuilt `core`. */
    val buildStd: List<String> = emptyList(),
    /** Cargo runner, used by `cargo run` to flash the board. */
    val runner: String,
    val rustFlags: List<String> = emptyList(),
    /** Board crates, as raw TOML values (e.g. `{ version = "1", features = ["x"] }`). */
    val dependencies: Map<String, String>,
    /** Top-level items of `main.rs` (app descriptors, global allocator, ...). */
    val preamble: String = "",
    /** Attribute that turns `async fn main(spawner: Spawner)` into the entry point. */
    val mainAttribute: String,
    /** Statements that run first in `main`: init the HAL, the heap and the time driver. */
    val init: String,
    /** Extra files to write in the project, by path relative to the project root. */
    val extraFiles: Map<String, String> = emptyMap(),
)

enum class Arch { XTENSA, RISCV }

data class StmChip(val part: String, val family: StmFamily) {
    companion object {
        fun fromId(id: String): StmChip? {
            val part = id.lowercase()
            // Require more than the bare family prefix: embassy-stm32 needs the full part.
            val family = StmFamily.forPart(part) ?: return null
            return if (part.length > family.prefix.length) StmChip(part, family) else null
        }
    }
}

enum class StmFamily(val prefix: String, val targetTriple: String) {
    // ARMv6-M (Cortex-M0/M0+)
    C0("stm32c0", "thumbv6m-none-eabi"),
    F0("stm32f0", "thumbv6m-none-eabi"),
    G0("stm32g0", "thumbv6m-none-eabi"),
    L0("stm32l0", "thumbv6m-none-eabi"),
    U0("stm32u0", "thumbv6m-none-eabi"),
    WB0("stm32wb0", "thumbv6m-none-eabi"),
    // ARMv7-M (Cortex-M3)
    F1("stm32f1", "thumbv7m-none-eabi"),
    F2("stm32f2", "thumbv7m-none-eabi"),
    L1("stm32l1", "thumbv7m-none-eabi"),
    // ARMv7E-M, no FPU
    WL("stm32wl", "thumbv7em-none-eabi"),
    // ARMv7E-M with FPU (Cortex-M4F/M7F)
    F3("stm32f3", "thumbv7em-none-eabihf"),
    F4("stm32f4", "thumbv7em-none-eabihf"),
    F7("stm32f7", "thumbv7em-none-eabihf"),
    G4("stm32g4", "thumbv7em-none-eabihf"),
    H7("stm32h7", "thumbv7em-none-eabihf"),
    L4("stm32l4", "thumbv7em-none-eabihf"),   // includes L4+ (l4r/l4s/l4p/l4q)
    WB("stm32wb", "thumbv7em-none-eabihf"),
    // ARMv8-M Mainline with FPU (Cortex-M33)
    H5("stm32h5", "thumbv8m.main-none-eabihf"),
    L5("stm32l5", "thumbv8m.main-none-eabihf"),
    U3("stm32u3", "thumbv8m.main-none-eabihf"),
    U5("stm32u5", "thumbv8m.main-none-eabihf"),
    WBA("stm32wba", "thumbv8m.main-none-eabihf"),
    // ARMv8.1-M (Cortex-M55)
    N6("stm32n6", "thumbv8.1m.main-none-eabihf");

    val isArmV6: Boolean get() = targetTriple.startsWith("thumbv6m")

    companion object {
        /** Longest prefix wins, so "stm32wba52cg" matches WBA rather than WB. */
        fun forPart(part: String): StmFamily? {
            val p = part.lowercase()
            return entries.filter { p.startsWith(it.prefix) }.maxByOrNull { it.prefix.length }
        }
    }
}

enum class EspChip(val id: String, val arch: Arch, val targetTriple: String) {
    ESP32 ("esp32", Arch.XTENSA, "xtensa-esp32-none-elf"),
    ESP32S2 ("esp32s2", Arch.XTENSA, "xtensa-esp32s2-none-elf"),
    ESP32S3 ("esp32s3", Arch.XTENSA, "xtensa-esp32s3-none-elf"),
    ESP32C2 ("esp32c2", Arch.RISCV, "riscv32imc-unknown-none-elf"),
    ESP32C3 ("esp32c3", Arch.RISCV, "riscv32imc-unknown-none-elf"),
    ESP32C6 ("esp32c6", Arch.RISCV, "riscv32imac-unknown-none-elf"),
    ESP32H2 ("esp32h2", Arch.RISCV, "riscv32imac-unknown-none-elf");

    companion object {
        fun fromId(id: String): EspChip? = entries.find { it.id == id.lowercase() }
    }
}

object RustBoardInfo {

    /** Heap given to the global allocator, in KiB. The runtime allocates its reactors and event queue on it. */
    private const val HEAP_KIB = 64

    /** Versions shared by all boards. They must match the ones `reactor_rt` uses with its `embassy` feature. */
    private const val EMBASSY_EXECUTOR = "0.10"
    private const val EMBASSY_TIME = "0.5"

    val supportedBoards: List<String> = buildList {
        EspChip.entries.mapTo(this) { it.id }
        StmFamily.entries.mapTo(this) { "${it.prefix}xxxx" }
        add("rp2040")
    }

    /** The board info for [board], or null if it is not supported. */
    fun boardInfoOrNull(board: String): BoardInfo? {
        EspChip.fromId(board)?.let { return esp(it) }
        StmChip.fromId(board)?.let { return stm32(it) }
        return when (board.lowercase()) {
            "rp2040" -> rp2040()
            else     -> null
        }
    }

    fun isSupported(board: String): Boolean = boardInfoOrNull(board) != null

    fun getBoardInfo(platformProperty: PlatformProperty.PlatformOptions): BoardInfo {
        val board = platformProperty.board().value()
        return boardInfoOrNull(board) ?: throw IllegalArgumentException(
            "Unsupported board for the Rust target: '$board'. Supported boards are: ${supportedBoards.joinToString()}."
        )
    }

    private fun stm32(chip: StmChip, heapKib: Int = HEAP_KIB): BoardInfo = BoardInfo(
        name = chip.part,
        targetTriple = chip.family.targetTriple,
        toolchainChannel = "stable",
        toolchainTargets = listOf(chip.family.targetTriple),
        runner = "probe-rs run --chip ${chip.part.uppercase().substringBefore("-")}",
        rustFlags = listOf("-C", "link-arg=-Tlink.x"),
        dependencies = buildMap {
            put("embassy-stm32", """{ version = "0.4", features = ["${chip.part}", "time-driver-any", "memory-x", "exti"] }""")
            put("embassy-executor", """{ version = "$EMBASSY_EXECUTOR", features = ["platform-cortex-m", "executor-thread"] }""")
            put("embassy-time", """{ version = "$EMBASSY_TIME" }""")
            put("cortex-m", """{ version = "0.7", features = ["critical-section-single-core"] }""")
            put("cortex-m-rt", """{ version = "0.7" }""")
            put("embedded-alloc", """{ version = "0.6" }""")
            put("panic-halt", """{ version = "1.0" }""")
            if (chip.family.isArmV6) {
                // No CAS atomics on ARMv6-M
                put("portable-atomic", """{ version = "1", features = ["critical-section"] }""")
            }
        },
        preamble = """
        |#[global_allocator]
        |static HEAP: embedded_alloc::LlffHeap = embedded_alloc::LlffHeap::empty();
    """.trimMargin(),
        mainAttribute = "embassy_executor::main",
        init = """
        |let peripherals = embassy_stm32::init(Default::default());
        |{
        |    use core::mem::MaybeUninit;
        |    const HEAP_SIZE: usize = $heapKib * 1024;
        |    static mut HEAP_MEM: [MaybeUninit<u8>; HEAP_SIZE] = [MaybeUninit::uninit(); HEAP_SIZE];
        |    unsafe { HEAP.init(core::ptr::addr_of_mut!(HEAP_MEM) as usize, HEAP_SIZE) }
        |}
    """.trimMargin(),
        extraFiles = mapOf(),
    )

    /**
     * ESP32 (Xtensa|riscv), using `esp-hal` with `esp-rtos` as the embassy
     * executor and time driver. Needs the `esp` toolchain (install it with `espup`).
     */
    private fun esp(chip: EspChip): BoardInfo {
        val c = chip.id
        val xtensa = chip.arch == Arch.XTENSA

        return BoardInfo(
            name = c,
            targetTriple = chip.targetTriple,
            toolchainChannel = if (xtensa) "esp" else "stable",
            buildStd = if (xtensa) listOf("core", "alloc") else emptyList(),
            runner = "espflash flash --monitor --chip $c",
            rustFlags = buildList {
                if (xtensa) addAll(listOf("-C", "link-arg=-nostartfiles"))
                addAll(listOf("-C", "link-arg=-Tlinkall.x"))
            },
            dependencies = mapOf(
                "esp-hal" to """{ version = "~1.2", features = ["$c", "unstable"] }""",
                "esp-rtos" to """{ version = "0.4", features = ["$c", "embassy"] }""",
                "esp-alloc" to """{ version = "0.11", features = ["$c"] }""",
                "esp-bootloader-esp-idf" to """{ version = "0.6", features = ["$c"] }""",
                "embassy-executor" to """{ version = "$EMBASSY_EXECUTOR" }""",
                "embassy-time" to """{ version = "$EMBASSY_TIME" }""",
                "panic-halt" to """{ version = "1.0" }""",
            ),
            preamble = """
                |// Application descriptor, read by the ESP-IDF bootloader.
                |esp_bootloader_esp_idf::esp_app_desc!();
            """.trimMargin(),
            mainAttribute = "esp_rtos::main",
            init = """
                |let peripherals = esp_hal::init(esp_hal::Config::default().with_cpu_clock(esp_hal::clock::CpuClock::max()));
                |esp_alloc::heap_allocator!(size: $HEAP_KIB * 1024);
                |// esp-rtos is the embassy time driver
                |let timg0 = esp_hal::timer::timg::TimerGroup::new(peripherals.TIMG0);
                |let sw_int = esp_hal::interrupt::software::SoftwareInterruptControl::new(peripherals.SW_INTERRUPT);
                |esp_rtos::start(timg0.timer0, sw_int.software_interrupt0);
            """.trimMargin(),
        )
    }

    /**
     * RP2040 (Cortex-M0+), using `embassy-rp`, which also provides the
     * time driver and critical-section implementation.
     */
    private fun rp2040(): BoardInfo = BoardInfo(
        name = "rp2040",
        targetTriple = "thumbv6m-none-eabi",
        toolchainChannel = "stable",
        toolchainTargets = listOf("thumbv6m-none-eabi"),
        runner = "picotool load -u -v -x -t elf",
        rustFlags = listOf(
            "-C", "link-arg=--nmagic",
            "-C", "link-arg=-Tlink.x",
            "-C", "link-arg=-Tlink-rp.x",
        ),
        dependencies = mapOf(
            "embassy-rp" to """{ version = "0.10", features = ["rp2040", "time-driver", "critical-section-impl"] }""",
            "embassy-executor" to """{ version = "$EMBASSY_EXECUTOR", features = ["platform-cortex-m", "executor-thread"] }""",
            "embassy-time" to """{ version = "$EMBASSY_TIME" }""",
            "cortex-m-rt" to """{ version = "0.7" }""",
            "embedded-alloc" to """{ version = "0.7" }""",
            "panic-halt" to """{ version = "1.0" }""",
        ),
        preamble = """
            |#[global_allocator]
            |static HEAP: embedded_alloc::LlffHeap = embedded_alloc::LlffHeap::empty();
        """.trimMargin(),
        mainAttribute = "embassy_executor::main",
        init = """
            |let _peripherals = embassy_rp::init(Default::default());
            |{
            |    use core::mem::MaybeUninit;
            |    const HEAP_SIZE: usize = $HEAP_KIB * 1024;
            |    static mut HEAP_MEM: [MaybeUninit<u8>; HEAP_SIZE] = [MaybeUninit::uninit(); HEAP_SIZE];
            |    unsafe { HEAP.init(core::ptr::addr_of_mut!(HEAP_MEM) as usize, HEAP_SIZE) }
            |}
        """.trimMargin(),
        extraFiles = mapOf(
            "memory.x" to """
                |MEMORY {
                |    BOOT2 : ORIGIN = 0x10000000, LENGTH = 0x100
                |    FLASH : ORIGIN = 0x10000100, LENGTH = 2048K - 0x100
                |    RAM   : ORIGIN = 0x20000000, LENGTH = 256K
                |}
                |""".trimMargin(),
            // Put memory.x on the linker search path, for cortex-m-rt's link.x.
            "build.rs" to """
                |use std::{env, fs, path::PathBuf};
                |
                |fn main() {
                |    let out = PathBuf::from(env::var_os("OUT_DIR").unwrap());
                |    fs::write(out.join("memory.x"), include_bytes!("memory.x")).unwrap();
                |    println!("cargo:rustc-link-search={}", out.display());
                |    println!("cargo:rerun-if-changed=memory.x");
                |}
                |""".trimMargin(),
        ),
    )
}
