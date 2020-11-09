import ULog from 'ulog';

/**
 * Utilities for the reactor runtime.
 * 
 * @author Marten Lohstroh (marten@berkeley.edu)
 */

export interface PrioritySetElement<P> {

    /**
     * Pointer to the next node in the priority set.
     */
    next: PrioritySetElement<P> | undefined;

    /**
     * Return the priority of this node.
     */
    getPriority(): P;

    /**
     * Determine whether this node has priority over the given node or not.
     * @param node A node to compare the priority of this node to.
     */
    hasPriorityOver: (node: PrioritySetElement<P> | undefined) => boolean;

    /**
     * If the given node is considered a duplicate of this node, then
     * update this node if needed, and return true. Return false otherwise.
     * @param node A node that may or may not be a duplicate of this node.
     */
    updateIfDuplicateOf: (node: PrioritySetElement<P> | undefined) => boolean;
}

export interface Sortable<P> {
    setPriority(priority: P): void;

    // getSTPUntil(): TimeInstant
    // setSTPUntil(): TimeInstant
}

/**
 * A priority queue that overwrites duplicate entries.
 */
export class PrioritySet<P> {

    private head: PrioritySetElement<P> | undefined;
    private count: number = 0;

    push(element: PrioritySetElement<P>) {
        // update linked list
        if (this.head == undefined) {
            // create head
            element.next = undefined;
            this.head = element;
            this.count++;
            return;
        } else if (element.updateIfDuplicateOf(this.head)) {
            // updateIfDuplicateOf returned true, i.e., 
            // it has updated the value of this.head to
            // equal that of element.
            return;
        } else {
            // prepend
            if (element.hasPriorityOver(this.head)) {
                element.next = this.head;
                this.head = element;
                this.count++;
                return;
            }
            // seek
            var curr: PrioritySetElement<P> | undefined = this.head;
            while (curr) {
                let next: PrioritySetElement<P> | undefined = curr.next;
                if (next) {
                    if (element.updateIfDuplicateOf(next)) {
                        // updateIfDuplicateOf returned true, i.e., 
                        // it has updated the value of this.head to
                        // equal that of element.
                        return;
                    } else if (element.hasPriorityOver(next)) {
                        break;
                    } else {
                        curr = next;
                    }
                } else {
                    break;
                }
            }
            if (curr) {
                // insert
                element.next = curr.next; // undefined if last
                curr.next = element;
                this.count++;
                return;
            }
        }
    }

    pop(): PrioritySetElement<P> | undefined {
        if (this.head) {
            let node = this.head;
            this.head = this.head.next;
            node.next = undefined; // unhook from linked list
            this.count--;
            return node;
        }
    }

    peek(): PrioritySetElement<P> | undefined {
        if (this.head) {
            return this.head;
        }
    }

    size(): number {
        return this.count;
    }

    empty(): void {
        this.head = undefined;
        this.count = 0;
    }
}

export class DependencyGraph<T> {
    /**
     * Map nodes to the set of nodes that they depend on.
     **/
    protected adjacencyMap: Map<T, Set<T>> = new Map();

    protected numberOfEdges = 0;

    merge(apg: this) {
        for (const [k, v] of apg.adjacencyMap) {
            let nodes = this.adjacencyMap.get(k);
            if (nodes) {
                for (let n of v) {
                    if (!nodes.has(n)) {
                        nodes.add(n);
                        this.numberOfEdges++;
                    }
                }
            } else {
                this.adjacencyMap.set(k, v);
                this.numberOfEdges += v.size;
            }
        }
    }

    addNode(node: T) {
        if (!this.adjacencyMap.has(node)) {
            this.adjacencyMap.set(node, new Set());
        }
    }

    getEdges(node: T): Set<T> { // FIXME: use different terminology: origins/effects
        let nodes = this.adjacencyMap.get(node)
        if (nodes !== undefined) {
            return nodes
        } else {
            return new Set<T>()
        }
    }

    getBackEdges(node: T): Set<T> {
        let backEdges = new Set<T>()
        this.adjacencyMap.forEach((edges, dep) => edges.forEach((edge) => {if (edge === node) { backEdges.add(dep)}}))
        return backEdges
    }

    /**
     * Return the subset of origins that are reachable from the given effect.
     * @param effect A node in the graph that to search upstream of.
     * @param origins A set of nodes to be found anywhere upstream of effect.
     */
    reachableOrigins(effect: T, origins : Set<T>): Set<T> {
        let visited = new Set<T>()
        let reachable = new Set<T>()
        let self = this
        
        /**
         * Recursively traverse the graph to collect reachable origins.
         * @param current The current node being visited.
         */
        function search(current: T) {
            for (let next of self.getEdges(current)) {
                if (!visited.has(current)) {
                    // Do not visit a node twice.
                    if (origins.has(current)) {
                        // If the current node is among the origins searched
                        // for, add it to the reachable set.
                        reachable.add(current)
                    }
                    // Continue search, depth first.
                    if (reachable.size == origins.size) {
                        search(next)
                    } else {
                        // Preempt search of all origins are reachable.
                        return
                    }
                }
            }
        }
        
        search(effect)

        return reachable
    } 


    hasCycle(): boolean {
        // FIXME
        return false
    }

    removeNode(node: T) {
        let deps: Set<T> | undefined;
        if (deps = this.adjacencyMap.get(node)) {
            this.numberOfEdges -= deps.size;
            this.adjacencyMap.delete(node);
            for (const [v, e] of this.adjacencyMap) {
                if (e.has(node)) {
                    e.delete(node);
                    this.numberOfEdges--;
                }
            }
        }
    }

    // node -> deps
    addEdge(node: T, dependsOn: T) {
        let deps = this.adjacencyMap.get(node);
        if (!deps) {
            this.adjacencyMap.set(node, new Set([dependsOn]));
            this.numberOfEdges++;
        } else {
            if (!deps.has(dependsOn)) {
                deps.add(dependsOn);
                this.numberOfEdges++;
            }
        }
        // Create an entry for `dependsOn` if it doesn't exist.
        // This is so that the keys of the map contain all the
        // nodes in the graph.
        if (!this.adjacencyMap.has(dependsOn)) {
            this.adjacencyMap.set(dependsOn, new Set());
        }
    }

    addBackEdges(node: T, dependentNodes: Set<T>) {
        for (let a of dependentNodes) {
            this.addEdge(a, node);
        }
    }

    addEdges(node: T, dependsOn: Set<T>) {
        let deps = this.adjacencyMap.get(node);
        if (!deps) {
            this.adjacencyMap.set(node, new Set(dependsOn));
            this.numberOfEdges += dependsOn.size;
        } else {
            for (let dependency of dependsOn) {
                if (!deps.has(dependency)) {
                    deps.add(dependency);
                    this.numberOfEdges++;
                }
                if (!this.adjacencyMap.has(dependency)) {
                    this.adjacencyMap.set(dependency, new Set());
                }
            }
        }
    }

    removeEdge(node: T, dependsOn: T) {
        let deps = this.adjacencyMap.get(node);
        if (deps && deps.has(dependsOn)) {
            deps.delete(dependsOn);
            this.numberOfEdges--;
        }
    }

    size() {
        return [this.adjacencyMap.size, this.numberOfEdges];
    }

    nodes() {
        return this.adjacencyMap.keys();
    }

    /**
     * Return a DOT representation of the graph.
     */
    toString() {
        var dot = "";
        var graph = this.adjacencyMap;
        var visited: Set<T> = new Set();

        /**
         * Store the DOT representation of the given chain, which is really
         * just a stack of nodes. The top node of the stack (i.e., the first)
         * element in the chain is given separately.
         * @param node The node that is currently being visited.
         * @param chain The current chain that is being built.
         */
        function printChain(node: T, chain: Array<T>) {
            dot += "\n";
            dot += '"' + node + '"'
            while (chain.length > 0) {
                dot += "->" + '"' + chain.pop() + '"';
            }
            dot += ";";
        }

        /**
         * Recursively build the chains that emanate from the given node.
         * @param node The node that is currently being visited.
         * @param chain The current chain that is being built.
         */
        function buildChain(node: T, chain: Array<T>) {
            let match = false;
            for (let [v, e] of graph) {
                if (e.has(node)) {
                    // Found next link in the chain.
                    let deps = graph.get(node);
                    if (match || !deps || deps.size == 0) {
                        // Start a new line when this is not the first match,
                        // or when the current node is a start node.
                        chain = new Array();
                        Log.global.debug("Starting new chain.")
                    }

                    // Mark current node as visited.
                    visited.add(node);
                    // Add this node to the chain.
                    chain.push(node);

                    if (chain.includes(v)) {
                        Log.global.debug("Cycle detected.");
                        printChain(v, chain);
                    } else if (visited.has(v)) {
                        Log.global.debug("Overlapping chain detected.");
                        printChain(v, chain);
                    } else {
                        Log.global.debug("Adding link to the chain.");        
                        buildChain(v, chain);
                    }
                    // Indicate that a match has been found.
                    match = true;
                }
            }
            if (!match) {
                Log.global.debug("End of chain.");
                printChain(node, chain);
            }
        }
        
        let start: Array<T> = new Array();
        // Build a start set of node without dependencies.
        for (const [v, e] of this.adjacencyMap) {
            if (!e || e.size == 0) {
                start.push(v);
            }
        }

        // Build the chains.
        for (let s of start) {
            buildChain(s, new Array());
        }

        return "digraph G {" + dot + "\n}";
    }

    public rootNodes(): Set<T> {
        var roots: Set<T> = new Set();
        /* Populate start set */
        for (const [v, e] of this.adjacencyMap) {
            if (!e || e.size == 0) {
                roots.add(v); // leaf nodes have no dependencies
                //clone.delete(v); // FIXME add a removeNodes function to factor out the duplicate code below
            }
        }
        return roots
    }

    public leafNodes(): Set<T> {
        var leafs: Set<T> = new Set(this.nodes());
        for (let node of this.nodes()) {
            for (let dep of this.getEdges(node)) {
                leafs.delete(dep)
            }
        }
        return leafs
    }
}

export class SortableDependencyGraph<T extends Sortable<number>> extends DependencyGraph<T> {
    updatePriorities(destructive: boolean, spacing: number = 100) {
        var start: Array<T> = new Array();
        var graph: Map<T, Set<T>>
        var count = 0; 
        if (!destructive) { 
            graph = new Map() 
            /* duplicate the map */ 
            for (const [v, e] of this.adjacencyMap) { 
                graph.set(v, new Set(e)); 
            } 
        } else {
            graph = this.adjacencyMap
        }
        
        /* Populate start set */
        for (const [v, e] of this.adjacencyMap) {
            if (!e || e.size == 0) {
                start.push(v); // start nodes have no dependencies
                graph.delete(v); 
            } 
        }
        /* Sort reactions */
        for (var n: T | undefined; (n = start.shift()); count += spacing) {
            n.setPriority(count);
            // for each node v with an edge e from n to v do 
            for (const [v, e] of graph) { 
                if (e.has(n)) {
                    // v depends on n 
                    e.delete(n);
                } if (e.size == 0) {
                    start.push(v);
                    graph.delete(v);
                }
            }
        } if (graph.size != 0) {
            return false; // ERROR: cycle detected
        } else {
            return true;
        }
    }
}
/**
 * Log levels for `Log`.
 * @see Log
 */
export enum LogLevel {
    ERROR = 1,
    WARN = 2,
    INFO = 3,
    LOG = 4,
    DEBUG = 5
}

/**
 * Global logging facility that has multiple levels of severity.
 */
export class Log {

    /**
     * Available log levels.
     */
    public static levels = LogLevel;

    /**
     * Global instance of ulog that performs the logging.
     */
    public static global = ULog("reactor-ts");

    /**
     * Horizontal rule.
     */
    public static hr = "==============================================================================";

    /**
     * Map that keeps track of active loggers.
     */
    private static loggers: Map<string, ULog> = new Map();

    /**
     * Get the logger instance associated with the given module.
     * If it does not exist, it is created.
     * @param module The name associated with the logger
     */
    public static getInstance(module: string): ULog {
        var logger = Log.loggers.get(module);
        if (!logger) {
            logger = ULog(module);
            Log.loggers.set(module, logger);
        } 
        return logger;
    }

    /**
     * Log a message with severity `LogLevel.DEBUG`. The `message` callback
     * is only invoked if the ulog instance has a log level higher than
     * `LogLevel.DEBUG`.
     * @param obj The object to call the message callback on.
     * @param message Callback that returns a message string.
     * @param module The name associated with the logger.
     * @see LogLevel
     */    
    public static debug(obj:unknown, message: () => string, module?: string) {
        if (module) {
            if (Log.global.level >= LogLevel.DEBUG) {
                Log.getInstance(module).debug(message.call(obj));
            }
        } else {
            if (Log.global.level >= LogLevel.DEBUG) {
                Log.global.debug(message.call(obj));
            } 
        }
    }

    /**
     * Log a message with severity `LogLevel.ERROR`. The `message` callback
     * is only invoked if the ulog instance has a log level higher than
     * `LogLevel.ERROR`.
     * @param obj The object to call the message callback on.
     * @param message Callback that returns a message string.
     * @param module The name associated with the logger.
     * @see LogLevel
     */    
    public static error(obj:unknown, message: () => string, module?: string) {
        if (module) {
            if (Log.global.level >= LogLevel.ERROR) {
                Log.getInstance(module).error(message.call(obj));
            }
        } else {
            if (Log.global.level >= LogLevel.ERROR) {
                Log.global.error(message.call(obj));
            } 
        }
    }

    /**
     * Log a message with severity `LogLevel.INFO`. The `message` callback
     * is only invoked if the ulog instance has a log level higher than
     * `LogLevel.INFO`.
     * @param obj The object to call the message callback on.
     * @param message Callback that returns a message string.
     * @param module The name associated with the logger.
     * @see LogLevel
     */      
    public static info(obj:unknown, message: () => string, module?: string) {
        if (module) {
            if (Log.global.level >= LogLevel.INFO) {
                Log.getInstance(module).info(message.call(obj));
            }
        } else {
            if (Log.global.level >= LogLevel.INFO) {
                Log.global.info(message.call(obj));
            } 
        }
    }

    /**
     * Log a message with severity `LogLevel.LOG`. The `message` callback
     * is only invoked if the ulog instance has a log level higher than
     * `LogLevel.LOG`.
     * @param obj The object to call the message callback on.
     * @param message Callback that returns a message string.
     * @param module The name associated with the logger.
     * @see LogLevel
     */      
    public static log(obj:unknown, message: () => string, module?: string) {
        if (module) {
            if (Log.global.level >= LogLevel.LOG) {
                Log.getInstance(module).log(message.call(obj));
            }
        } else {
            if (Log.global.level >= LogLevel.LOG) {
                Log.global.log(message.call(obj));
            } 
        }
    }

    /**
     * Log a message with severity `LogLevel.WARN`. The `message` callback
     * is only invoked if the ulog instance has a log level higher than
     * `LogLevel.WARN`.
     * @param obj The object to call the message callback on.
     * @param message Callback that returns a message string.
     * @param module The name associated with the logger.
     * @see LogLevel
     */     
    public static warn(obj:unknown, message: () => string, module?: string) {
        if (module) {
            if (Log.global.level >= LogLevel.WARN) {
                Log.getInstance(module).warn(message.call(obj));
            }
        } else {
            if (Log.global.level >= LogLevel.WARN) {
                Log.global.warn(message.call(obj));
            } 
        }
    }    
}