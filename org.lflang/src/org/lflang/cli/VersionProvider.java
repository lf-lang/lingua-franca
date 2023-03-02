package org.lflang.cli;

import picocli.CommandLine.IVersionProvider;
import picocli.CommandLine.Model.CommandSpec;
import picocli.CommandLine.Spec;

import org.lflang.LocalStrings;

/*
 * Dynamically provides version information to the Lingua Franca CLI.
 *
 * @author Atharva Patil
 */
class VersionProvider implements IVersionProvider {
    @Spec CommandSpec spec;

    public String[] getVersion() {
        return new String[] {
            spec.qualifiedName()  + " " + LocalStrings.VERSION
        };
    }
}
