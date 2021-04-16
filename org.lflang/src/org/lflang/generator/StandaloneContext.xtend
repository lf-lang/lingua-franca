package org.lflang.generator

import java.nio.file.Path
import java.util.Properties
import org.eclipse.xtend.lib.annotations.Accessors
import org.eclipse.xtext.generator.GeneratorContext

@Accessors
class StandaloneContext extends GeneratorContext {

    Properties args = new Properties();
    
    Path packageRoot
    
    boolean hierarchicalBin = false;
    
}