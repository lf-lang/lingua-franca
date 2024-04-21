// package org.lflang.analyses.opt;

// import java.util.*;

// public class VF2 {
//     private Map<Integer, Integer> mappingM; // Maps nodes of G1 to nodes of G2
//     private Map<Integer, Integer> inverseMappingM; // Inverse mapping
//     private Set<Integer> in1; // Nodes in the mapping in G1
//     private Set<Integer> in2; // Nodes in the mapping in G2
//     private Set<Integer> out1; // Nodes not yet in the mapping in G1
//     private Set<Integer> out2; // Nodes not yet in the mapping in G2

//     public VF2() {
//         mappingM = new HashMap<>();
//         inverseMappingM = new HashMap<>();
//         in1 = new HashSet<>();
//         in2 = new HashSet<>();
//         out1 = new HashSet<>();
//         out2 = new HashSet<>();
//     }

//     public boolean isIsomorphic(ProgramDependenceGraph g1, ProgramDependenceGraph g2) {
//         return match(g1, g2, 0);
//     }

//     private boolean match(ProgramDependenceGraph g1, ProgramDependenceGraph g2, int depth) {
//         if (mappingM.size() == g1.nodeCount()) { // All nodes are mapped
//             return true;
//         }

//         Pair<Integer, Integer> p = selectCandidate(g1, g2);
//         int n = p.getFirst();
//         int m = p.getSecond();

//         if (feasible(g1, g2, n, m)) {
//             addMapping(n, m);
//             if (match(g1, g2, depth + 1)) {
//                 return true;
//             }
//             removeMapping(n, m);
//         }

//         return false;
//     }

//     private Pair<Integer, Integer> selectCandidate(ProgramDependenceGraph g1, ProgramDependenceGraph g2) {
//         for (Integer n : g1.getNodes()) {
//             if (!mappingM.containsKey(n)) {
//                 for (Integer m : g2.getNodes()) {
//                     if (!inverseMappingM.containsKey(m)) {
//                         return new Pair<>(n, m);
//                     }
//                 }
//             }
//         }
//         return null; // Should never happen if used correctly
//     }

//     private boolean feasible(ProgramDependenceGraph g1, ProgramDependenceGraph g2, Integer n, Integer m) {
//         // Check if 'n' and 'm' are compatible, i.e., have the same type of operation or statement
//         if (!g1.getNodeType(n).equals(g2.getNodeType(m))) {
//             return false;
//         }

//         // Further checks can be added here to verify control and data dependencies
//         return true;
//     }

//     private void addMapping(int n, int m) {
//         mappingM.put(n, m);
//         inverseMappingM.put(m, n);
//         updateInOutSets(n, m);
//     }

//     private void removeMapping(int n, int m) {
//         mappingM.remove(n);
//         inverseMappingM.remove(m);
//         revertInOutSets(n, m);
//     }

//     private void updateInOutSets(int n, int m) {
//         in1.add(n);
//         in2.add(m);
//         out1.remove(n);
//         out2.remove(m);
//     }

//     private void revertInOutSets(int n, int m) {
//         in1.remove(n);
//         in2.remove(m);
//         out1.add(n);
//         out2.add(m);
//     }
// }

// class Pair<T, U> {
//     private T first;
//     private U second;

//     public Pair(T first, U second) {
//         this.first = first;
//         this.second = second;
//     }

//     public T getFirst() {
//         return first;
//     }

//     public U getSecond() {
//         return second;
//     }
// }
