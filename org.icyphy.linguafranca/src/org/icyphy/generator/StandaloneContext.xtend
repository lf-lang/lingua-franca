package org.icyphy.generator

import org.eclipse.xtext.generator.GeneratorContext
import java.util.Properties
import org.eclipse.xtend.lib.annotations.Accessors
import java.nio.file.Path

@Accessors
class StandaloneContext extends GeneratorContext {

    Properties args = new Properties();
    
    Path packageRoot
    
    boolean hierarchicalBin = false;
    
}