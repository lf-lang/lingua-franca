/**
 * Copyright (c) 2019, The University of California at Berkeley.
 *
 * <p>Redistribution and use in source and binary forms, with or without modification, are permitted
 * provided that the following conditions are met:
 *
 * <p>1. Redistributions of source code must retain the above copyright notice, this list of
 * conditions and the following disclaimer.
 *
 * <p>2. Redistributions in binary form must reproduce the above copyright notice, this list of
 * conditions and the following disclaimer in the documentation and/or other materials provided with
 * the distribution.
 *
 * <p>THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR
 * IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND
 * FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY
 * WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */
package org.lflang.graph;

/**
 * Note annotations used in Tarjan's algorithm for finding strongly connected components.
 *
 * @author Marten Lohstroh
 */
public class NodeAnnotation {
  /**
   * Sequence number that is assigned when this node is discovered. A node with a lower index was
   * discovered later than this one; a node with a higher index was discovered later than this one.
   */
  public int index = -1;

  /** Temporary mark do be used in topological sort algorithm. */
  public boolean hasTempMark = false;

  /** Temporary mark do be used in topological sort algorithm. */
  public boolean hasPermMark = false;

  /** The smallest index of any node known to be reachable from this node. */
  public int lowLink = -1;

  /**
   * Whether or not this node is currently on the stack that keeps track of visited nodes that
   * potentially form a cycle.
   */
  public boolean onStack = false;

  /** Whether or not this node has a dependency on itself. */
  public boolean selfLoop = false;
}
