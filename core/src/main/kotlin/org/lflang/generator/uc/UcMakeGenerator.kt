package org.lflang.generator.uc

import org.lflang.FileConfig
import org.lflang.target.TargetConfig
import org.lflang.generator.PrependOperator
import org.lflang.joinWithLn
import org.lflang.target.property.BuildTypeProperty
import org.lflang.target.property.CmakeIncludeProperty
import org.lflang.target.property.ExternalRuntimePathProperty
import org.lflang.target.property.RuntimeVersionProperty
import org.lflang.toUnixString
import java.nio.file.Path

class UcMakeGenerator(private val targetConfig: TargetConfig, private val fileConfig: FileConfig) {
    private val S = '$' // a little trick to escape the dollar sign with $S
    fun generateMake(sources: List<Path>) = with(PrependOperator) {
        """
            | # Makefile genrated for ${fileConfig.name}
            |LF_GEN_SOURCES = \
        ${" |    "..sources.joinWithLn { it.toUnixString() + " \\ "}}
            |
        """.trimMargin()
    }
}
