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

package org.lflang;

import java.util.Arrays;
import java.util.LinkedHashSet;
import java.util.List;
import java.util.Set;
import java.util.stream.Collectors;

/**
 * A unit of time for a {@link TimeValue}.
 *
 * @author Marten Lohstroh
 * @author Clément Fournier, TU Dresden, INSA Rennes
 */
public enum TimeUnit {
  /** Nanoseconds. */
  NANO("nsec", "ns", "nsecs"),
  /** Microseconds. */
  MICRO("usec", "us", "usecs"),
  /** Milliseconds. */
  MILLI("msec", "ms", "msecs"),
  /** Seconds. */
  SECOND("sec", "s", "secs", "second", "seconds"),
  /** Minute. */
  // NOTE: Do not use MIN as the first entry. Common macro for minimum.
  MINUTE("minute", "min", "mins", "minutes"),
  /** Hour. */
  HOUR("hour", "h", "hours"),
  /** Day. */
  DAY("day", "d", "days"),
  WEEK("week", "wk", "weeks"),
  ;

  private final Set<String> allNames;
  private final String canonicalName;
  private final String siName;

  /**
   * Construct a time unit.
   *
   * @param canonicalName The name used in the generated code for the unit.
   * @param siName The SI unit name, if there is one, and otherwise a short name.
   * @param aliases Any number of alternative names for the unit.
   */
  TimeUnit(String canonicalName, String siName, String... aliases) {
    this.canonicalName = canonicalName;
    this.siName = siName;
    var all = new LinkedHashSet<String>();
    all.add(canonicalName);
    all.add(siName);
    all.addAll(Arrays.asList(aliases));
    this.allNames = all;
  }

  /** Returns the name that is preferred when displaying this unit. */
  public String getCanonicalName() {
    return canonicalName;
  }

  /** Returns the name that is preferred when displaying this unit. */
  public static String staticGetCanonicalName(TimeUnit unit) {
    if (unit == null) {
      return null;
    }
    return unit.getCanonicalName();
  }

  /** Returns true if the given name is one of the aliases of this unit. */
  public boolean hasAlias(String name) {
    return allNames.contains(name);
  }

  /**
   * Returns the constant corresponding to the given name. The comparison is case-sensitive.
   *
   * @return Null if the parameter is null, otherwise a non-null constant
   * @throws IllegalArgumentException If the name doesn't correspond to any constant
   */
  public static TimeUnit fromName(String name) {
    if (name == null) {
      return null;
    }
    return Arrays.stream(values())
        .filter(it -> it.hasAlias(name))
        .findFirst()
        .orElseThrow(() -> new IllegalArgumentException("invalid name '" + name + "'"));
  }

  /** Returns true if the parameter is null, or it is the alias of a valid time unit. */
  public static boolean isValidUnit(String name) {
    if (name == null) {
      return false;
    }
    return Arrays.stream(values()).anyMatch(it -> it.hasAlias(name));
  }

  /** Returns a list of all possible aliases for time values. */
  public static List<String> list() {
    return Arrays.stream(values()).flatMap(it -> it.allNames.stream()).collect(Collectors.toList());
  }

  @Override
  public String toString() {
    return this.siName;
  }
}
