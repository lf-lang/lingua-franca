"use strict";

Object.defineProperty(exports, "__esModule", {
  value: true
});
exports.Log = exports.PrecedenceGraph = exports.PrioritySet = void 0;

function _defineProperty(obj, key, value) { if (key in obj) { Object.defineProperty(obj, key, { value: value, enumerable: true, configurable: true, writable: true }); } else { obj[key] = value; } return obj; }

const ulog = require("ulog");
/**
 * Utilities for the reactor runtime.
 * 
 * @author Marten Lohstroh (marten@berkeley.edu)
 */


/**
 * A priority queue that overwrites duplicate entries.
 */
class PrioritySet {
  constructor() {
    _defineProperty(this, "head", void 0);

    _defineProperty(this, "count", 0);
  }

  push(element) {
    // update linked list
    if (this.head == undefined) {
      // create head
      element.setNext(undefined);
      this.head = element;
      this.count++;
      return;
    } else if (element.updateIfDuplicateOf(this.head)) {
      return;
    } else {
      // prepend
      if (element.hasPriorityOver(this.head)) {
        element.setNext(this.head);
        this.head = element;
        this.count++;
        return;
      } // seek


      var curr = this.head;

      while (curr) {
        let next = curr.getNext();

        if (next) {
          if (element.updateIfDuplicateOf(next)) {
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
        element.setNext(curr.getNext()); // undefined if last

        curr.setNext(element);
        this.count++;
        return;
      }
    }
  }

  pop() {
    if (this.head) {
      let node = this.head;
      this.head = this.head.getNext();
      node.setNext(undefined); // unhook from linked list

      this.count--;
      return node;
    }
  }

  peek() {
    if (this.head) {
      return this.head;
    }
  }

  size() {
    return this.count;
  }

  empty() {
    this.head = undefined;
    this.count = 0;
  }

}

exports.PrioritySet = PrioritySet;

class PrecedenceGraph {
  constructor() {
    _defineProperty(this, "graph", new Map());

    _defineProperty(this, "numberOfEdges", 0);
  }

  //protected startNodes: Set<T> = new Set(); 
  merge(apg) {
    for (const [k, v] of apg.graph) {
      let nodes = this.graph.get(k);

      if (nodes) {
        for (let n of v) {
          if (!nodes.has(n)) {
            nodes.add(n);
            this.numberOfEdges++;
          }
        }
      } else {
        this.graph.set(k, v);
        this.numberOfEdges += v.size;
      }
    }
  }

  addNode(node) {
    if (!this.graph.has(node)) {
      this.graph.set(node, new Set());
    }
  }

  removeNode(node) {
    let deps;

    if (deps = this.graph.get(node)) {
      this.numberOfEdges -= deps.size;
      this.graph.delete(node);

      for (const [v, e] of this.graph) {
        if (e.has(node)) {
          e.delete(node);
          this.numberOfEdges--;
        }
      }
    }
  } // node -> deps


  addEdge(node, dependsOn) {
    let deps = this.graph.get(node);

    if (!deps) {
      this.graph.set(node, new Set([dependsOn]));
      this.numberOfEdges++;
    } else {
      if (!deps.has(dependsOn)) {
        deps.add(dependsOn);
        this.numberOfEdges++;
      }
    }

    if (!this.graph.has(dependsOn)) {
      this.graph.set(dependsOn, new Set());
    }
  }

  addBackEdges(node, dependentNodes) {
    for (let a of dependentNodes) {
      this.addEdge(a, node);
    }
  }

  addEdges(node, dependsOn) {
    let deps = this.graph.get(node);

    if (!deps) {
      this.graph.set(node, new Set(dependsOn));
      this.numberOfEdges += dependsOn.size;
    } else {
      for (let dependency of dependsOn) {
        if (!deps.has(dependency)) {
          deps.add(dependency);
          this.numberOfEdges++;
        }

        if (!this.graph.has(dependency)) {
          this.graph.set(dependency, new Set());
        }
      }
    }
  }

  removeEdge(node, dependsOn) {
    let deps = this.graph.get(node);

    if (deps && deps.has(dependsOn)) {
      deps.delete(dependsOn);
      this.numberOfEdges--;
    }
  }

  size() {
    return [this.graph.size, this.numberOfEdges];
  }

  updatePriorities(spacing = 100) {
    var start = new Array();
    var clone = new Map();
    var count = 0;
    /* duplicate the graph */

    for (const [v, e] of this.graph) {
      clone.set(v, new Set(e));
    }
    /* Populate start set */


    for (const [v, e] of this.graph) {
      if (!e || e.size == 0) {
        start.push(v); // start nodes have no dependencies

        clone.delete(v);
      }
    }
    /* Sort reactions */


    for (var n; n = start.shift(); count += spacing) {
      n.setPriority(count); // for each node v with an edge e from n to v do

      for (const [v, e] of clone) {
        if (e.has(n)) {
          // v depends on n
          e.delete(n);
        }

        if (e.size == 0) {
          start.push(v);
          clone.delete(v);
        }
      }
    }

    if (clone.size != 0) {
      return false; // ERROR: cycle detected
    } else {
      return true;
    }
  }

  nodes() {
    return this.graph.keys();
  }
  /**
   * Return a DOT representation of the graph.
   */


  toString() {
    var dot = "";
    var graph = this.graph;
    var visited = new Set();
    /**
     * Store the DOT representation of the given chain, which is really
     * just a stack of nodes. The top node of the stack (i.e., the first)
     * element in the chain is given separately.
     * @param node The node that is currently being visited.
     * @param chain The current chain that is being built.
     */

    function printChain(node, chain) {
      dot += "\n";
      dot += '"' + node + '"';

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


    function buildChain(node, chain) {
      let match = false;

      for (let [v, e] of graph) {
        if (e.has(node)) {
          // Found next link in the chain.
          let deps = graph.get(node);

          if (match || !deps || deps.size == 0) {
            // Start a new line when this is not the first match,
            // or when the current node is a start node.
            chain = new Array();
            Log.global.debug("Starting new chain.");
          } // Mark current node as visited.


          visited.add(node); // Add this node to the chain.

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
          } // Indicate that a match has been found.


          match = true;
        }
      }

      if (!match) {
        Log.global.debug("End of chain.");
        printChain(node, chain);
      }
    }

    let start = new Array(); // Build a start set of node without dependencies.

    for (const [v, e] of this.graph) {
      if (!e || e.size == 0) {
        start.push(v);
      }
    } // Build the chains.


    for (let s of start) {
      buildChain(s, new Array());
    }

    return "digraph G {" + dot + "\n}";
  }

}
/**
 * Log levels for `Log`.
 * @see Log
 */


exports.PrecedenceGraph = PrecedenceGraph;
var LogLevel;
/**
 * Global logging facility that has multiple levels of severity.
 */

(function (LogLevel) {
  LogLevel[LogLevel["ERROR"] = 1] = "ERROR";
  LogLevel[LogLevel["WARN"] = 2] = "WARN";
  LogLevel[LogLevel["INFO"] = 3] = "INFO";
  LogLevel[LogLevel["LOG"] = 4] = "LOG";
  LogLevel[LogLevel["DEBUG"] = 5] = "DEBUG";
})(LogLevel || (LogLevel = {}));

class Log {
  /**
   * Available log levels.
   */

  /**
   * Horizontal rule.
   */

  /**
   * Global instance of ulog that performs the logging.
   */
  static getInstance(module) {
    return ulog(module);
  } // FIXME: write type declarations for ulog

  /**
   * Set the minimum level of severity a message is required to 
   * have for the logger to display it.
   * @param level The minimum level of severity
   * @see LogLevel
   */


  static setGlobalLevel(level) {
    this.global.level = level;
  }
  /**
   * Get the minimum level of severity a message is required to 
   * have for the logger to display it.
   * @see LogLevel
   */


  static getGlobalLevel() {
    return this.global.level;
  }

}

exports.Log = Log;

_defineProperty(Log, "levels", LogLevel);

_defineProperty(Log, "hr", "==============================================================================");

_defineProperty(Log, "global", ulog("reactor-ts"));