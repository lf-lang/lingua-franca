package org.lflang.generator.cpp

interface ConnectionGenerator {
    abstract fun generateDeclarations() : String
    abstract fun generateInitializers() : String
}