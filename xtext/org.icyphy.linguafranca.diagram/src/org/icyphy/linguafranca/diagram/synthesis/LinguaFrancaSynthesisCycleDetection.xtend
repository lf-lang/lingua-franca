package org.icyphy.linguafranca.diagram.synthesis

import com.google.common.collect.HashBiMap
import com.google.inject.Inject
import de.cau.cs.kieler.klighd.internal.util.KlighdInternalProperties
import de.cau.cs.kieler.klighd.kgraph.KGraphElement
import de.cau.cs.kieler.klighd.kgraph.KNode
import de.cau.cs.kieler.klighd.kgraph.KPort
import de.cau.cs.kieler.klighd.krendering.ViewSynthesisShared
import de.cau.cs.kieler.klighd.krendering.extensions.KPortExtensions
import java.util.Map
import java.util.function.Consumer
import java.util.function.Supplier
import org.eclipse.elk.core.options.CoreOptions
import org.eclipse.elk.graph.properties.Property
import org.eclipse.xtend.lib.annotations.Data
import org.icyphy.AnnotatedDependencyGraph
import org.icyphy.AnnotatedNode
import org.icyphy.linguaFranca.Action

import static extension de.cau.cs.kieler.klighd.kgraph.util.KGraphIterators.*
import static extension java.lang.String.*

/**
 * Dependency cycle detection for Lingua Franca diagrams based on the diagram topology.
 * 
 * @author{Alexander Schulz-Rosengarten <als@informatik.uni-kiel.de>}
 */
@ViewSynthesisShared
class LinguaFrancaSynthesisCycleDetection extends AbstractSynthesisExtensions {
	
	// Properties for marking diagram elements
	public static val RECURSIVE_INSTANTIATION = new Property<Boolean>("org.icyphy.linguafranca.diagram.synthesis.recursive.instantiation", false)
	public static val DEPENDENCY_CYCLE = new Property<Boolean>("org.icyphy.linguafranca.diagram.synthesis.dependency.cycle", false)
	
	private static val DEBUG_CYCLE_DETECTION = false
	
	@Inject extension KPortExtensions
	
	/**
	 * Performs cycle detection based on the diagram's graph structure and applies given highlighting to the included elements
	 */
	def DetectionResult detectAndHighlightCycles(KNode node, Consumer<KGraphElement> highlighter) {
		var containsRecursion = false
		var depGraph = new AnnotatedDependencyGraph()
		
    	// Some edges do not specify ports hence we introduce virtual ports to correctly handle dependency separation
    	val virtualPorts = HashBiMap.<KNode,KPort>create
		
		// Build graph and find cycles
        for (childNode : node.getKNodeIterator(false).toIterable) {
        	if (childNode.getProperty(RECURSIVE_INSTANTIATION)) {
        		containsRecursion = true
        	}
        	
        	if (!childNode.outgoingEdges.empty && !childNode.incomingEdges.empty) { // simplify dependency graph
        		val vPort = virtualPorts.getOrInit(childNode)[createPort()]
        		val breaksCylce = switch(childNode.getProperty(KlighdInternalProperties.MODEL_ELEMEMT)) {
        			Action: true
        			default: false
        		}
            	if (childNode.ports.empty) {
            		// If node has no ports ignore outgoing if it breaks cycles
	            	if (!breaksCylce) {
		                depGraph.addEdges(new AnnotatedNode(vPort), childNode.outgoingEdges.map[
		                	new AnnotatedNode(targetPort?:virtualPorts.getOrInit(target)[createPort()])
		                ].toSet)
	            	}
            	} else {
            		// If node has port prefer dependencies on ports
            		for (edge : childNode.outgoingEdges.filter[!target.outgoingEdges.empty]) { // also simplify dependency graph
            			depGraph.addEdge(
	                		new AnnotatedNode(edge.sourcePort?:vPort),
	                		new AnnotatedNode(edge.targetPort?:virtualPorts.getOrInit(edge.target)[createPort()])
            			)
            		}
            		// If node breaks cycles or has children -> do not introduce dependencies from input to output
	            	if (!breaksCylce && childNode.children.empty) {
	            		val vPortNode = new AnnotatedNode(vPort)
	            		val outputs = childNode.outgoingEdges.map[sourcePort].filterNull.toSet.map[new AnnotatedNode(it)]
	            		val inputs = childNode.incomingEdges.map[targetPort].filterNull.toSet.map[new AnnotatedNode(it)]
	            		
	            		for (input : inputs) {
            				depGraph.addEdge(input, vPortNode)
	            		}
            			depGraph.addEdges(vPortNode, outputs.toSet)
	            	}
            	}
        	}
        }
        depGraph.detectCycles()
        
        if (DEBUG_CYCLE_DETECTION) {
            println("-- DEBUG CYCLE DETECTION --")
            for (n : depGraph.nodes) {
            	for(d : depGraph.getOrigins(n)) {
            		val sN = (n.contents.eContainer?:virtualPorts.inverse.get(n.contents)) as KNode
            		val tN = (d.contents.eContainer?:virtualPorts.inverse.get(d.contents)) as KNode
            		val sO = sN?.getProperty(KlighdInternalProperties.MODEL_ELEMEMT).toString
            		val tO = tN?.getProperty(KlighdInternalProperties.MODEL_ELEMEMT).toString
            		val sP = n.contents.eContainer === null ? "virtual" : n.contents.getProperty(CoreOptions.PORT_SIDE).toString
            		val tP = d.contents.eContainer === null ? "virtual" : d.contents.getProperty(CoreOptions.PORT_SIDE).toString
            		println("%s[%s] -> %s[%s]".format(sO, sP, tO, tP))
            	}
            }
		}
		
        if (!depGraph.cycles.empty && highlighter !== null) {
			// Highlight cycles
            for (cycle : depGraph.cycles) {
            	val cyclePorts = cycle.toSet
            	val cycleNodes = cycle.map[
            		if (it.node !== null) {
            			it.node
            		} else {
            			virtualPorts.inverse.get(it)
            		}
            	].filterNull.toSet
            	for (cycleNode : cycleNodes) {
            		cycleNode.setProperty(DEPENDENCY_CYCLE, true)
            		highlighter.accept(cycleNode)
            		for (cycleEgde : cycleNode.outgoingEdges.filter[
            			cycleNodes.contains(target) &&
            			sourcePort === null ? true : cyclePorts.contains(sourcePort)
            		]) {
            			cycleEgde.setProperty(DEPENDENCY_CYCLE, true)
            			highlighter.accept(cycleEgde)
            		}
            		for (cyclePort : cycleNode.ports.filter[cyclePorts.contains(it)]) {
            			cyclePort.setProperty(DEPENDENCY_CYCLE, true)
            			highlighter.accept(cyclePort)
            		}
            	}
            }
         }
         
         return new DetectionResult(!depGraph.cycles.empty, containsRecursion)
	}
	
	/**
	 * Utility to get the value in a map or put an initial value and get that one.
	 */
	private def <K,V> V getOrInit(Map<K, V> map, K key, Supplier<V> init) {
		if (map === null) {
			return null
		} else if (map.containsKey(key)) {
			return map.get(key)
		} else {
			val value = init.get()
			map.put(key, value)
			return value
		}
	}

}

@Data
class DetectionResult {
	val boolean cycle
	val boolean recursion
}
