package org.lflang.graph;

import java.util.List;
import java.util.Set;

public interface Graph<T> {

    /**
     * Return an unmodifiable set of nodes in this graph.
     */
    Set<T> nodes();


    boolean hasNode(T node);


    /**
     * Add the given node to the graph.
     *
     * @param node The node to add to the graph.
     */
    void addNode(T node);


    /**
     * Remove the given node from the graph. This also eliminates any
     * edges from upstream and to downstream neighbors of this node.
     *
     * @param node The node to remove.
     */
    void removeNode(T node);


    void addEdge(T to, T from);


    void addEdges(T to, List<T> from);


    void removeEdge(T to, T from);


    /**
     * Return the number of nodes in this graph.
     */
    int nodeCount();


    /** Return the number of directed edges in this graph. */
    int edgeCount();
}
