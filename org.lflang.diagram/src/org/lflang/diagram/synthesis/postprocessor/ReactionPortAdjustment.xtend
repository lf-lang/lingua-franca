/*************
* Copyright (c) 2020, Kiel University.
*
* Redistribution and use in source and binary forms, with or without modification,
* are permitted provided that the following conditions are met:
*
* 1. Redistributions of source code must retain the above copyright notice,
*    this list of conditions and the following disclaimer.
*
* 2. Redistributions in binary form must reproduce the above copyright notice,
*    this list of conditions and the following disclaimer in the documentation
*    and/or other materials provided with the distribution.
*
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND 
* ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED 
* WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE 
* DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR
* ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES 
* (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; 
* LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON 
* ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT 
* (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS 
* SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
***************/
package org.lflang.diagram.synthesis.postprocessor

import de.cau.cs.kieler.klighd.IStyleModifier
import de.cau.cs.kieler.klighd.IViewer
import de.cau.cs.kieler.klighd.kgraph.KGraphFactory
import de.cau.cs.kieler.klighd.kgraph.KNode
import de.cau.cs.kieler.klighd.kgraph.KPoint
import de.cau.cs.kieler.klighd.kgraph.KPort
import org.eclipse.elk.core.options.CoreOptions
import org.eclipse.elk.core.options.PortSide
import org.eclipse.elk.graph.properties.Property
import org.lflang.diagram.synthesis.styles.LinguaFrancaShapeExtensions

/**
 * Adjusts the port position of reactions node AFTER layout, to allow free port order but also adapt (snuggle) to pointy shape of reaction node.
 * 
 * @author{Alexander Schulz-Rosengarten <als@informatik.uni-kiel.de>}
 */
class ReactionPortAdjustment implements IStyleModifier {

	public static val ID = "org.lflang.diagram.synthesis.postprocessor.ReactionPortAdjustment"
	static val PROCESSED = new Property<Boolean>("org.lflang.diagram.synthesis.postprocessor.reaction.ports.processed", false)
	
	extension KGraphFactory = KGraphFactory.eINSTANCE

	override modify(StyleModificationContext context) {
		try {
			val node = context.graphElement
			if (node instanceof KNode) {
				// Find root node
				var parent = node
				while(parent.eContainer !== null) {
					parent = parent.eContainer as KNode
				}
				// Get viewer
				val viewer = parent.allProperties.entrySet.findFirst[key.id.equals("de.cau.cs.kieler.klighd.viewer")]?.value
				val recorder = if (viewer instanceof IViewer) {
					viewer.viewContext?.layoutRecorder
				}
				
				if (!node.ports.empty) {
					if (node.ports.head.ypos !== 0 && !node.getProperty(PROCESSED)) { // Only adjust if layout is already applied
						recorder?.startRecording // important for incremental update animation
						
						val in = node.ports.filter[getProperty(CoreOptions.PORT_SIDE) === PortSide.WEST && !hasProperty(CoreOptions.PORT_BORDER_OFFSET)].sortBy[ypos].toList
						val out = node.ports.filter[getProperty(CoreOptions.PORT_SIDE) === PortSide.EAST && !hasProperty(CoreOptions.PORT_BORDER_OFFSET)].sortBy[ypos].toList
	
						// Adjust
						in.indexed.adjustPositions(in.size, true)
						out.indexed.adjustPositions(out.size, false)
						node.setProperty(PROCESSED, true)
						
						recorder?.stopRecording(0)
					} else if (node.ports.head.ypos === 0) {
						node.setProperty(PROCESSED, false)
					}
				}
			}
		} catch (Exception e) {
			e.printStackTrace
			// do not disturb rendering process
		}
		return false
	}

	def void adjustPositions(Iterable<Pair<Integer, KPort>> indexedPorts, int count, boolean input) {
		val segments = (LinguaFrancaShapeExtensions.REACTION_POINTINESS * 2) / (count + 1)
		for (indexedPort : indexedPorts) {
			val port = indexedPort.value
			val idx = indexedPort.key
			
			var float offset = 0
			if (count % 2 !== 0 && idx === count / 2) {
				offset += LinguaFrancaShapeExtensions.REACTION_POINTINESS
			} else if (idx < count / 2) {
				offset += segments * (idx + 1)
			} else {
				offset += segments * (count - idx)
			}
			
			if (!input) { // reverse
				offset -= LinguaFrancaShapeExtensions.REACTION_POINTINESS
			}
			
			// apply
			port.setPos(port.xpos + offset, port.ypos)
			for (edge : port.edges) {
				if (input) {
					edge.targetPoint = edge.targetPoint.adjustedKPoint(offset)
				} else {
					edge.sourcePoint = edge.sourcePoint.adjustedKPoint(offset)
				}
			}
		}
	}

	def KPoint adjustedKPoint(KPoint point, float xOffest) {
		return createKPoint => [
			x = point.x + xOffest
			y = point.y
		]
	}

}
