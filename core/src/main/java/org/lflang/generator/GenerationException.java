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

package org.lflang.generator;

import org.eclipse.emf.ecore.EObject;

// import org.jetbrains.annotations.Nullable;

/** An exception that occurred during code generation. May also wrap another exception. */
public class GenerationException
    extends RuntimeException { // note that this is an unchecked exception.

  /* @Nullable */
  private final EObject location;

  public GenerationException(String message) {
    this(null, message, null);
  }

  public GenerationException(/* @Nullable */ EObject location, String message) {
    this(location, message, null);
  }

  public GenerationException(String message, Throwable cause) {
    this(null, message, cause);
  }

  public GenerationException(/* @Nullable */ EObject location, String message, Throwable cause) {
    super(message, cause);
    this.location = location;
  }

  public GenerationException(Throwable cause) {
    this(null, null, cause);
  }

  /* @Nullable */
  public EObject getLocation() {
    return location;
  }
}
