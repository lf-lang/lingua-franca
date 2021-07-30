package org.lflang.graph;

import java.util.List;
import java.util.Set;

public interface Graph<T> {

    Set<T> nodes();

    boolean hasNode(T node);

    void addNode(T node);

    void removeNode(T node);

    void addEdge(T to, T from);

    void addEdges(T to, List<T> from);

    void removeEdge(T to, T from);

    int nodeCount();

    int edgeCount();
}
