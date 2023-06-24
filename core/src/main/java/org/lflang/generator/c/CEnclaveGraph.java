package org.lflang.generator.c;
import java.util.List;
import java.util.Map;
import java.util.HashMap;
import java.util.HashSet;
import java.util.Set;
import java.util.Stack;

import org.lflang.TimeValue;
import org.lflang.generator.PortInstance;
import org.lflang.generator.ReactorInstance;
import org.lflang.generator.RuntimeRange;
import org.lflang.generator.SendRange;

public class CEnclaveGraph {
    private Map<ReactorInstance, Set<EnclaveConnection>> graph;

    private Stack<ReactorInstance> zeroDelayCycle;

    public CEnclaveGraph(List<ReactorInstance> enclaves) {
        graph = new HashMap<>();
        addNodes(enclaves);

        for (ReactorInstance enclave: enclaves) {
            for (PortInstance input: enclave.inputs) {
                if (input.eventualSources().size() == 1) {
                    ReactorInstance source = input.eventualSources().get(0).parentReactor();
                    ReactorInstance sourceEnclave = CUtil.getClosestEnclave(source);

                    TimeValue delay = TimeValue.ZERO;
                    // Get the delay by inspecting the ConnectionReactor
                    if (input.getDependentPorts().size() == 1) {
                        ReactorInstance connReactor = input.getDependentPorts().get(0).destinations.get(0).parentReactor();
                        delay = connReactor.actions.get(0).getMinDelay();
                        // FIXME: Also check for isPhysical here
                    } else {
                        // FIXME: Can eventual source be more than 1 for an input port? It is an error condition for us, at least
                    }

                    addEdge(sourceEnclave, enclave, delay, false);
                } else if (input.eventualSources().size() > 1) {
                    // FIXME: This is an error. Enclave inputs can only be driven by a single source
                }
            }

            for (PortInstance output: enclave.outputs) {
                for (SendRange sendRange: output.eventualDestinations()) {
                    for (RuntimeRange runtimeRange: sendRange.destinations) {
                        ReactorInstance destReactor = runtimeRange.parentReactor();
                        ReactorInstance destEnclave = CUtil.getClosestEnclave(destReactor);
                        // destReactor is actually the ConnectionReactor within the target enclave.
                        if (destReactor.actions.size() != 1) {
                            // FIXME: Error. This should be the ConnectionReactor which has a single Action
                        }
                        TimeValue delay = destReactor.actions.get(0).getMinDelay();
                        // FIXME: Also check for isPhysical here

                        addEdge(enclave, destEnclave, delay, false);
                    }
                }
            }
        }
    }

    // To find zero delay cycles in the enclave graph. We do a Depth First Search from each node and
    // look for backedges. However, since we are interested in zero-delay cycles. We only consider edges
    // with zero delay.

    public boolean hasZeroDelayCycles() {
        Set<ReactorInstance> visited = new HashSet<>();
        for (ReactorInstance node : graph.keySet()) {
            if (hasZeroDelayCycle(node, visited, new Stack<>())) {
                return true;
            }
        }
        return false;
    }

    private boolean hasZeroDelayCycle(ReactorInstance current, Set<ReactorInstance> visited, Stack<ReactorInstance> path) {
        visited.add(current);
        path.push(current);

        // Loop through all zero-delay outgoing edges
        for (EnclaveConnection edge : graph.get(current).stream().filter(e -> e.getDelay().toNanoSeconds() == 0).toList()) {
            if (!visited.contains(edge.target)) {
                if (hasZeroDelayCycle(edge.target, visited, path)) {
                    return true;
                }
            } else if (path.contains(edge.target)) {
                zeroDelayCycle = path;
                return true;
            }
        }

        path.pop();
        visited.remove(current);
        return false;
    }

    public ReactorInstance getUserEnclave(ReactorInstance enclave) {
        if (enclave.isMainOrFederated()) {
            return enclave;
        } else {
            return enclave.children.get(0);
        }
    }

    public String buildCycleString() {
        StringBuilder cycle = new StringBuilder();
        boolean recording = false;
        ReactorInstance start = getUserEnclave(zeroDelayCycle.get(0));
        for (ReactorInstance node : zeroDelayCycle) {
            cycle.append(getUserEnclave(node).getFullName()).append(" -> ");
        }
        cycle.append(start.getFullName());
        return cycle.toString();
    }

    public void addNode(ReactorInstance node) {
        if (!graph.containsKey(node)) {
            graph.put(node, new HashSet<>());
        }
    }
    public void addNodes(List<ReactorInstance> nodes) {
        for (ReactorInstance node: nodes) {
            addNode(node);
        }
    }

    public void addEdge(ReactorInstance source, ReactorInstance target, TimeValue delay, boolean isPhysical) {
        if (graph.containsKey(source) && graph.containsKey(target)) {
            EnclaveConnection newEdge = new EnclaveConnection(source, target, delay, isPhysical);
            graph.get(source).add(newEdge);
        } else {
            // FIXME: Error
        }
    }

    public Set<EnclaveConnection> getDirectUpstreams(ReactorInstance node) {
        Set<EnclaveConnection> upstreams = new HashSet<>();
        for (Map.Entry<ReactorInstance, Set<EnclaveConnection>> entry : graph.entrySet()) {
            for (EnclaveConnection edge : entry.getValue()) {
                if (edge.getTarget().equals(node)) {
                    upstreams.add(edge);
                    break;
                }
            }
        }
        return upstreams;
    }

    public Set<EnclaveConnection> getDirectDownstreams(ReactorInstance node) {
        return graph.getOrDefault(node, new HashSet<>());
    }

    public class EnclaveConnection {
        private ReactorInstance target;
        private ReactorInstance source;

        private boolean isPhysical;
        private TimeValue delay;
        public EnclaveConnection(ReactorInstance source, ReactorInstance target, TimeValue delay, boolean isPhysical) {
            this.source = source;
            this.target = target;
            this.delay = delay;
            this.isPhysical = isPhysical;
        }

        public ReactorInstance getTarget() {
            return target;
        }

        public ReactorInstance getSource() {
            return source;
        }

        public TimeValue getDelay() {
            return delay;
        }

        public boolean isPhysical() {
            return isPhysical;
        }
    }

}

