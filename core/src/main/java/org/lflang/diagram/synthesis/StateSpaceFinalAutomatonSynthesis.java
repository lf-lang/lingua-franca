package org.lflang.diagram.synthesis;

import java.util.EnumSet;

import org.eclipse.elk.alg.layered.options.LayeredOptions;
import org.eclipse.elk.core.math.ElkPadding;
import org.eclipse.elk.core.options.CoreOptions;
import org.eclipse.elk.core.options.Direction;
import org.eclipse.elk.core.options.NodeLabelPlacement;
import org.eclipse.elk.core.options.SizeConstraint;
import org.eclipse.xtext.xbase.lib.Extension;

import org.lflang.lf.Model;

import com.google.inject.Inject;
import de.cau.cs.kieler.klighd.KlighdConstants;
import de.cau.cs.kieler.klighd.kgraph.KEdge;
import de.cau.cs.kieler.klighd.kgraph.KNode;
import de.cau.cs.kieler.klighd.krendering.extensions.KContainerRenderingExtensions;
import de.cau.cs.kieler.klighd.krendering.extensions.KEdgeExtensions;
import de.cau.cs.kieler.klighd.krendering.extensions.KLabelExtensions;
import de.cau.cs.kieler.klighd.krendering.extensions.KNodeExtensions;
import de.cau.cs.kieler.klighd.krendering.extensions.KPolylineExtensions;
import de.cau.cs.kieler.klighd.krendering.extensions.KPortExtensions;
import de.cau.cs.kieler.klighd.krendering.extensions.KRenderingExtensions;
import de.cau.cs.kieler.klighd.syntheses.AbstractDiagramSynthesis;

public class StateSpaceFinalAutomatonSynthesis extends AbstractDiagramSynthesis<Model> {
    @Inject
    @Extension
    private KNodeExtensions _kNodeExtensions;
    @Inject @Extension private KEdgeExtensions _kEdgeExtensions;
    @Inject @Extension private KPortExtensions _kPortExtensions;
    @Inject @Extension private KLabelExtensions _kLabelExtensions;
    @Inject @Extension private KRenderingExtensions _kRenderingExtensions;
    @Inject @Extension private KContainerRenderingExtensions _kContainerRenderingExtensions;
    @Inject @Extension private KPolylineExtensions _kPolylineExtensions;

    public static final String ID = "org.lflang.diagram.synthesis.StateSpaceDiagram";

    @Override
    public KNode transform(Model model) {
        KNode rootNode = _kNodeExtensions.createNode();
        setLayoutOption(rootNode, CoreOptions.ALGORITHM, LayeredOptions.ALGORITHM_ID);
        setLayoutOption(rootNode, CoreOptions.DIRECTION, Direction.RIGHT);
        setLayoutOption(rootNode, CoreOptions.PADDING, new ElkPadding(0));
        setLayoutOption(rootNode, LayeredOptions.NODE_LABELS_PLACEMENT, NodeLabelPlacement.insideCenter());
        setLayoutOption(rootNode, LayeredOptions.NODE_SIZE_CONSTRAINTS, EnumSet.of(SizeConstraint.MINIMUM_SIZE, SizeConstraint.NODE_LABELS));
        KNode stateSpace = _kNodeExtensions.createNode();
        associateWith(stateSpace, model);
        rootNode.getChildren().add(stateSpace);
        _kRenderingExtensions.addRectangle(stateSpace);
        _kLabelExtensions.addInsideCenteredNodeLabel(stateSpace, "TEst",
            1,
            KlighdConstants.DEFAULT_FONT_NAME);


        KNode stateSpace2 = _kNodeExtensions.createNode();
        rootNode.getChildren().add(stateSpace2);
        _kRenderingExtensions.addRectangle(stateSpace2);
        _kLabelExtensions.addInsideCenteredNodeLabel(stateSpace2, "TEst");

        KEdge edge = _kEdgeExtensions.createEdge();
        edge.setSource(stateSpace);
        edge.setTarget(stateSpace2);
        return rootNode;
    }
}
