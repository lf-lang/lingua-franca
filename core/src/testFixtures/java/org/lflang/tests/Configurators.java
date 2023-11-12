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

package org.lflang.tests;

import org.lflang.target.TargetConfig;
import org.lflang.target.property.LoggingProperty;
import org.lflang.target.property.PlatformProperty;
import org.lflang.target.property.PlatformProperty.PlatformOptions;
import org.lflang.target.property.SingleThreadedProperty;
import org.lflang.target.property.WorkersProperty;
import org.lflang.target.property.type.LoggingType.LogLevel;
import org.lflang.target.property.type.PlatformType.Platform;

/**
 * Configuration procedures for {@link TestBase} methods.
 *
 * @author Clément Fournier
 * @author Marten Lohstroh
 */
public class Configurators {

  /** Function to adapts a given target configuration. */
  @FunctionalInterface
  public interface Configurator {

    /**
     * Apply a side effect to the given test case to change its default configuration. Return true
     * if configuration succeeded, false otherwise.
     */
    boolean configure(TargetConfig config);
  }

  /**
   * Configure the given test to use single-threaded execution.
   *
   * <p>For targets that provide a threaded and an unthreaded runtime, this configures using the
   * unthreaded runtime. For targets that do not distinguish threaded and unthreaded runtime, the
   * number of workers is set to 1.
   *
   * @param config The target configuration to alter.
   * @return True if successful, false otherwise.
   */
  public static boolean disableThreading(TargetConfig config) {
    SingleThreadedProperty.INSTANCE.override(config, true);
    WorkersProperty.INSTANCE.override(config, 1);
    return true;
  }

  public static boolean makeZephyrCompatibleUnthreaded(TargetConfig config) {

    // NOTE: Zephyr emulations fails with debug log-levels.
    disableThreading(config);
    LoggingProperty.INSTANCE.override(config, LogLevel.WARN);

    var platform = config.get(PlatformProperty.INSTANCE);
    PlatformProperty.INSTANCE.override(
        config,
        new PlatformOptions(
            Platform.ZEPHYR,
            "qemu_cortex_m3",
            platform.port(),
            platform.baudRate(),
            false,
            platform.userThreads()));
    return true;
  }

  public static boolean makeZephyrCompatible(TargetConfig config) {
    // NOTE: Zephyr emulations fails with debug log-levels.
    LoggingProperty.INSTANCE.override(config, LogLevel.WARN);

    var platform = config.get(PlatformProperty.INSTANCE);
    PlatformProperty.INSTANCE.override(
        config,
        new PlatformOptions(
            Platform.ZEPHYR,
            "qemu_cortex_m3",
            platform.port(),
            platform.baudRate(),
            false,
            platform.userThreads()));
    return true;
  }
  /**
   * Make no changes to the configuration.
   *
   * @param config The target configuration.
   * @return True
   */
  public static boolean noChanges(TargetConfig config) {
    return true;
  }
}
