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
package org.lflang.diagram.synthesis.postprocessor;

import de.cau.cs.kieler.klighd.IStyleModifier;
import de.cau.cs.kieler.klighd.IViewer;
import de.cau.cs.kieler.klighd.internal.ILayoutRecorder;
import de.cau.cs.kieler.klighd.kgraph.KEdge;
import de.cau.cs.kieler.klighd.kgraph.KGraphElement;
import de.cau.cs.kieler.klighd.kgraph.KGraphFactory;
import de.cau.cs.kieler.klighd.kgraph.KNode;
import de.cau.cs.kieler.klighd.kgraph.KPoint;
import de.cau.cs.kieler.klighd.kgraph.KPort;
import java.util.List;
import java.util.Map;
import org.eclipse.elk.core.options.CoreOptions;
import org.eclipse.elk.core.options.PortSide;
import org.eclipse.elk.graph.properties.IProperty;
import org.eclipse.elk.graph.properties.Property;
import org.eclipse.xtext.xbase.lib.Extension;
import org.eclipse.xtext.xbase.lib.IterableExtensions;
import org.eclipse.xtext.xbase.lib.Pair;
import org.lflang.diagram.synthesis.styles.LinguaFrancaShapeExtensions;

/**
 * Adjusts the port position of reactions node AFTER layout, to allow free port order but also adapt (snuggle) to pointy shape of reaction node.
 * 
 * @author{Alexander Schulz-Rosengarten <als@informatik.uni-kiel.de>}
 */
public class ReactionPortAdjustment implements IStyleModifier {

    public static final String ID = "org.lflang.diagram.synthesis.postprocessor.ReactionPortAdjustment";
    
    private static final Property<Boolean> PROCESSED = new Property<Boolean>("org.lflang.diagram.synthesis.postprocessor.reaction.ports.processed", false);
    
    @Extension
    private KGraphFactory _kGraphFactory = KGraphFactory.eINSTANCE;

    @Override
    public boolean modify(IStyleModifier.StyleModificationContext context) {
		try {
		    KGraphElement node = context.getGraphElement();
			if (node instanceof KNode) {
			    KNode knode = (KNode) node;
			    
				// Find root node
			    KNode parent = knode;
				while (parent.eContainer() != null) {
					parent = (KNode) parent.eContainer();
				}
				
				// Get viewer (this is a bit brittle because it fetches the viewer from some internal property)
				Map.Entry<IProperty<?>, Object> first = IterableExtensions.findFirst(
    			        parent.getAllProperties().entrySet(), 
    			        it -> {
    			            return it.getKey().getId().equals("de.cau.cs.kieler.klighd.viewer") || 
    			                   it.getKey().getId().equals("klighd.layout.viewer");
    			        }
		        );
				Object viewer = first != null ? first.getValue() : null;
				
				ILayoutRecorder recorder = null;
				if (viewer instanceof IViewer) {
				    recorder = ((IViewer) viewer).getViewContext().getLayoutRecorder();
				}
				
				
				if (!knode.getPorts().isEmpty()) {
					if (IterableExtensions.head(knode.getPorts()).getYpos() != 0 && 
					        !knode.getProperty(ReactionPortAdjustment.PROCESSED)) { // Only adjust if layout is already applied
						// important for incremental update animation
					    if (recorder != null) {
					        recorder.startRecording();
					    }
					    
					    List<KPort> in = IterableExtensions.toList(
				            IterableExtensions.sortBy(
			                    IterableExtensions.filter(
			                            knode.getPorts(),
                                        it -> {
                                            return it.getProperty(CoreOptions.PORT_SIDE) == PortSide.WEST && 
                                                  !it.hasProperty(CoreOptions.PORT_BORDER_OFFSET);
                                        }), 
			                    it -> { return it.getYpos(); })
			            );
					    
					    List<KPort> out = IterableExtensions.toList(
                            IterableExtensions.sortBy(
                                IterableExtensions.filter(
                                    knode.getPorts(),
                                    it -> {
                                        return it.getProperty(CoreOptions.PORT_SIDE) == PortSide.EAST && 
                                              !it.hasProperty(CoreOptions.PORT_BORDER_OFFSET);
                                    }), 
                                it -> { return it.getYpos(); })
                        );

						// Adjust
					    adjustPositions(IterableExtensions.indexed(in), in.size(), true);
			            adjustPositions(IterableExtensions.indexed(out), out.size(), false);
						knode.setProperty(ReactionPortAdjustment.PROCESSED, true);
						
						if (recorder!=null) {
			              recorder.stopRecording(0);
			            }
						
					} else if (IterableExtensions.head(knode.getPorts()).getYpos() == 0) {
						knode.setProperty(PROCESSED, false);
					}
				}
			}
		} catch (Exception e) {
		    e.printStackTrace();
			// do not disturb rendering process
		}
		return false;
	}

    public void adjustPositions(Iterable<Pair<Integer, KPort>> indexedPorts, int count, boolean input) {
        float segments = LinguaFrancaShapeExtensions.REACTION_POINTINESS * 2 / (count + 1);
		for (Pair<Integer, KPort> indexedPort : indexedPorts) {
		    KPort port = indexedPort.getValue();
	        int idx = indexedPort.getKey();
	        float offset = 0;
	        
	        if (count % 2 != 0 && idx == count / 2) {
	            offset += LinguaFrancaShapeExtensions.REACTION_POINTINESS;
	        } else if (idx < count / 2) {
	            offset += segments * (idx + 1);
	        } else {
	            offset += segments * (count - idx);
	        }
			
			if (!input) { // reverse
				offset -= LinguaFrancaShapeExtensions.REACTION_POINTINESS;
			}
			
			// apply
			port.setPos(port.getXpos() + offset, port.getYpos());
			for (KEdge edge : port.getEdges()) {
				if (input) {
				    edge.setTargetPoint(adjustedKPoint(edge.getTargetPoint(), offset));
				} else {
				    edge.setSourcePoint(adjustedKPoint(edge.getSourcePoint(), offset));
				}
			}
		}
	}

    public KPoint adjustedKPoint(KPoint point, float xOffset) {
        KPoint kPoint = _kGraphFactory.createKPoint();
        kPoint.setX(point.getX() + xOffset);
        kPoint.setY(point.getY());
        return kPoint;
	}

}
