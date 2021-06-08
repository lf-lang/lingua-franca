package org.lflang.generator.rust

import org.eclipse.emf.ecore.resource.Resource
import org.eclipse.xtext.generator.IFileSystemAccess2
import org.eclipse.xtext.generator.IGeneratorContext
import org.lflang.FileConfig
import org.lflang.generator.cpp.name
import org.lflang.lf.Reactor
import java.io.Closeable
import java.io.IOException
import java.nio.file.Path

class RustFileConfig(resource: Resource, fsa: IFileSystemAccess2, context: IGeneratorContext) :
    FileConfig(resource, fsa, context) {

    /**
     * Clean any artifacts produced by the C++ code generator.
     */
    @Throws(IOException::class)
    override fun doClean() {
        super.doClean()
        deleteDirectory(outPath.resolve("target"))
    }

    /** Relative path to the directory where all source files for this resource should be generated in. */
    private fun getGenDir(r: Resource): Path = getDirectory(r).resolve(r.name)

    fun emit(p: Path, f:Emitter.()-> Unit) = Emitter(p)
    fun emit(pathRelativeToOutDir: String) = emit(srcGenPath.resolve(pathRelativeToOutDir))


    /** Path to the source file corresponding to this reactor (needed for non generic reactors)  */
    fun getReactorSourcePath(r: Reactor): Path = getGenDir(r.eResource()).resolve("${r.name}.cc")
}

class Emitter(
    private val output: Path,
) : Closeable {

    private val sb = StringBuilder()

    operator fun plusAssign(s: String) {
        sb.append(s)
    }

    override fun close() {
        output.toFile().bufferedWriter().use {
            it.write(sb.toString())
        }
    }
}

