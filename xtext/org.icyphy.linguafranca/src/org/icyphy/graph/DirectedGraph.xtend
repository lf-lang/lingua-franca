package org.icyphy.graph

import java.util.List

interface DirectedGraph<T> extends Graph<T> {
    
    def List<T> getOrigins(T node)
 
    def List<T> getEffects(T node)
    
    def List<T> rootNodes()

    def List<T> leafNodes()
        
}