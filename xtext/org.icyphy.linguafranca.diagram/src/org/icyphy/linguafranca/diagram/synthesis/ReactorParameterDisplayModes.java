package org.icyphy.linguafranca.diagram.synthesis;

/**
 * Enumeration of different display options for reactor parameters.
 * 
 * @author{Alexander Schulz-Rosengarten <als@informatik.uni-kiel.de>}
 */
public enum ReactorParameterDisplayModes {
    NONE,
    TITLE,
    TABLE;
    
    @Override
    public String toString() {
        switch(this) {
            case NONE: return "None";
            case TITLE: return "List in Title";
            case TABLE: return "Table in Body";
            default: return "";
        }
    }
}
