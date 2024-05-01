package org.lflang.diagram.lsp;

public class NodePosition {
    private int start;
    private int end;

    public NodePosition(int start, int end){
        this.start = start;
        this.end = end;
    }

    public int getStart() {
        return start;
    }

    public int getEnd() {
        return end;
    }
}
