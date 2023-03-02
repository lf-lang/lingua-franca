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

import java.util.stream.Collectors;

import org.eclipse.elk.core.options.CoreOptions;
import org.eclipse.elk.core.options.PortSide;
import org.eclipse.elk.graph.properties.Property;
import org.eclipse.xtext.xbase.lib.Extension;
import org.eclipse.xtext.xbase.lib.IterableExtensions;
import org.eclipse.xtext.xbase.lib.Pair;
import org.lflang.diagram.synthesis.styles.LinguaFrancaShapeExtensions;

import de.cau.cs.kieler.klighd.IStyleModifier;
import de.cau.cs.kieler.klighd.IViewer;
import de.cau.cs.kieler.klighd.internal.ILayoutRecorder;
import de.cau.cs.kieler.klighd.kgraph.KEdge;
import de.cau.cs.kieler.klighd.kgraph.KGraphElement;
import de.cau.cs.kieler.klighd.kgraph.KGraphFactory;
import de.cau.cs.kieler.klighd.kgraph.KNode;
import de.cau.cs.kieler.klighd.kgraph.KPoint;
import de.cau.cs.kieler.klighd.kgraph.KPort;
import de.cau.cs.kieler.klighd.krendering.KRendering;
import de.cau.cs.kieler.klighd.krendering.KRenderingFactory;

/**
 * Adjusts the port position of reactions node AFTER layout, to allow free port order but also adapt (snuggle) to pointy shape of reaction node.
 * 
 * @author Alexander Schulz-Rosengarten
 */
public class ReactionPortAdjustment implements IStyleModifier {

    public static final String ID = "org.lflang.diagram.synthesis.postprocessor.ReactionPortAdjustment";
    
    /** 
     * INTERNAL property to mark node as processed.
     */
    public static final Property<Boolean> PROCESSED = new Property<>("org.lflang.diagram.synthesis.postprocessor.reaction.ports.processed", false);
    
    @Extension
    private KGraphFactory _kGraphFactory = KGraphFactory.eINSTANCE;
    private static KRenderingFactory _kRenderingFactory = KRenderingFactory.eINSTANCE;
    
    /**
     * Register this modifier on a reaction rendering.
     */
    public static void apply(KNode node, KRendering rendering) {
        // Add modifier that fixes port positions such that edges are properly attached to the shape
        var invisible = _kRenderingFactory.createKInvisibility();
        invisible.setInvisible(false); // make it ineffective (just for purpose of holding modifier)
        invisible.setModifierId(ReactionPortAdjustment.ID); // Add modifier to receive callback after layout
        rendering.getStyles().add(invisible);
        node.setProperty(PROCESSED, false);
    }

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
                Object viewer = 
                        parent.getAllProperties().entrySet().stream().filter(entry -> 
                                    entry.getKey().getId().equals("de.cau.cs.kieler.klighd.viewer")
                                    || entry.getKey().getId().equals("klighd.layout.viewer"))
                                .findAny().map(entry -> entry.getValue()).orElse(null);
                                
                ILayoutRecorder recorder = null;
                if (viewer instanceof IViewer) {
                    recorder = ((IViewer) viewer).getViewContext().getLayoutRecorder();
                }
                
                if (!knode.getPorts().isEmpty()) {
                    if (IterableExtensions.head(knode.getPorts()).getYpos() != 0 && 
                             // Only adjust if layout is already applied important for incremental update animation
                            !knode.getProperty(ReactionPortAdjustment.PROCESSED)) {
                        if (recorder != null) {
                            recorder.startRecording();
                        }
                        
                        var in = knode.getPorts().stream().filter(p -> 
                            p.getProperty(CoreOptions.PORT_SIDE) == PortSide.WEST).sorted((p1, p2) ->  
                                Float.compare(p1.getYpos(), p2.getYpos())).collect(Collectors.toList());
                        
                        var out = knode.getPorts().stream().filter(p -> 
                            p.getProperty(CoreOptions.PORT_SIDE) == PortSide.EAST).sorted((p1, p2) ->  
                                Float.compare(p1.getYpos(), p2.getYpos())).collect(Collectors.toList());

                        // Adjust
                        if (in.stream().anyMatch(p -> !p.hasProperty(CoreOptions.PORT_BORDER_OFFSET))) {
                            adjustPositions(IterableExtensions.indexed(in), in.size(), true);
                        }
                        if (out.stream().anyMatch(p -> !p.hasProperty(CoreOptions.PORT_BORDER_OFFSET))) {
                            adjustPositions(IterableExtensions.indexed(out), out.size(), false);
                        }
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
            
            // Save for future layout
            port.setProperty(CoreOptions.PORT_BORDER_OFFSET, (double) (input ? -offset : offset));
        }
    }

    public KPoint adjustedKPoint(KPoint point, float xOffset) {
        KPoint kPoint = _kGraphFactory.createKPoint();
        kPoint.setX(point.getX() + xOffset);
        kPoint.setY(point.getY());
        return kPoint;
    }

}
