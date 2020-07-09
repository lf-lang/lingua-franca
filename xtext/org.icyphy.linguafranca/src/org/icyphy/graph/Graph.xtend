package org.icyphy.graph

import java.util.List

interface Graph<T> {
    
    def boolean hasNode(T node)
    
    def void addNode(T node)
    
    def void removeNode(T node)
    
    def void addEdge(T to, T from)
    
    def void addEdges(T to, List<T> from) 
    
    def void removeEdge(T to, T from)
    
    def int nodeCount()
    
    def int edgeCount()
    
    def List<T> nodes()
    
}