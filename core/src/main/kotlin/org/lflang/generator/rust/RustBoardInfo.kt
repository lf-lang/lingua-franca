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

object RustBoardInfo {

    /** Heap given to the global allocator, in KiB. The runtime allocates its reactors and event queue on it. */
    private const val HEAP_KIB = 64

    /** Versions shared by all boards. They must match the ones `reactor_rt` uses with its `embassy` feature. */
    private const val EMBASSY_EXECUTOR = "0.10"
    private const val EMBASSY_TIME = "0.5"

    val supportedBoards = listOf("esp32", "rp2040")

    fun getBoardInfo(platformProperty: PlatformProperty.PlatformOptions): BoardInfo =
        when (val board = platformProperty.board().value()) {
            "esp32"  -> esp32()
            "rp2040" -> rp2040()
            else     -> throw IllegalArgumentException(
                "Unsupported board for the Rust target: '$board'. Supported boards are: ${supportedBoards.joinToString()}."
            )
        }

    /**
     * ESP32 (Xtensa), using `esp-hal` with `esp-rtos` as the embassy
     * executor and time driver. Needs the `esp` toolchain (install it with `espup`).
     */
    private fun esp32(): BoardInfo = BoardInfo(
        name = "esp32",
        targetTriple = "xtensa-esp32-none-elf",
        toolchainChannel = "esp",
        buildStd = listOf("core", "alloc"),
        runner = "espflash flash --monitor --chip esp32",
        rustFlags = listOf(
            "-C", "link-arg=-nostartfiles",
            "-C", "link-arg=-Tlinkall.x",
        ),
        dependencies = mapOf(
            "esp-hal" to """{ version = "~1.2", features = ["esp32", "unstable"] }""",
            "esp-rtos" to """{ version = "0.4", features = ["esp32", "embassy"] }""",
            "esp-alloc" to """{ version = "0.11", features = ["esp32"] }""",
            "esp-bootloader-esp-idf" to """{ version = "0.6", features = ["esp32"] }""",
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
            |esp_rtos::start(timg0.timer0, peripherals.FROM_CPU_INTR0);
        """.trimMargin(),
    )

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
