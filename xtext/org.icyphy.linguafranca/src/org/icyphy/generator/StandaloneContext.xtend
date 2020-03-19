package org.icyphy.generator

import org.eclipse.xtext.generator.GeneratorContext
import java.util.Properties
import org.eclipse.xtend.lib.annotations.Accessors

@Accessors
class StandaloneContext extends GeneratorContext {

    Properties  args
    
}