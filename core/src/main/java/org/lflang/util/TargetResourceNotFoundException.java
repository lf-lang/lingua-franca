package org.lflang.util;

import java.io.IOException;

public class TargetResourceNotFoundException extends IOException {
  public TargetResourceNotFoundException(String resourcePath) {
    super(
        String.format(
            """
            A required resource could not be found on the classpath or is an empty directory: %s
            Perhaps a git submodule is missing or not up to date.
            Try running 'git submodule update --init --recursive' and rebuild.
            Also see https://www.lf-lang.org/docs/handbook/developer-setup.
            """,
            resourcePath));
  }
}
