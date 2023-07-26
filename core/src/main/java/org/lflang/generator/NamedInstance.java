/* Base class for instances with names in Lingua Franca. */

/*************
 * Copyright (c) 2019, The University of California at Berkeley.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 * this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 * this list of conditions and the following disclaimer in the documentation
 * and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY
 * EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL
 * THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
 * STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF
 * THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 ***************/

package org.lflang.generator;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import org.eclipse.emf.ecore.EObject;

/**
 * Base class for compile-time instances with names in Lingua Franca. An instance of concrete
 * subclasses of this class represents one or more runtime instances of a reactor, port, reaction,
 * etc. There will be more than one runtime instance if the object or any of its parents is a bank
 * of reactors.
 *
 * @author Marten Lohstroh
 * @author Edward A. Lee
 */
public abstract class NamedInstance<T extends EObject> {

  /**
   * Construct a new instance with the specified definition and parent. E.g., for a reactor
   * instance, the definition is Instantiation, and for a port instance, it is Port. These are nodes
   * in the AST. This is protected because only subclasses should be constructed.
   *
   * @param definition The definition in the AST for this instance.
   * @param parent The reactor instance that creates this instance.
   */
  protected NamedInstance(T definition, ReactorInstance parent) {
    this.definition = definition;
    this.parent = parent;

    // Calculate the depth.
    this.depth = 0;
    ReactorInstance p = parent;
    while (p != null) {
      p = p.parent;
      this.depth++;
    }
  }

  //////////////////////////////////////////////////////
  //// Public fields.

  /** A limit on the number of characters returned by uniqueID. */
  public static int identifierLengthLimit = 40;

  //////////////////////////////////////////////////////
  //// Public methods.

  /** Return the definition, which is the AST node for this object. */
  public T getDefinition() {
    return definition;
  }

  /**
   * Get the depth of the reactor instance. This is 0 for the main reactor, 1 for reactors
   * immediately contained therein, etc.
   */
  public int getDepth() {
    return depth;
  }

  /**
   * Return the full name of this instance, which has the form "a.b.c", where "c" is the name of
   * this instance, "b" is the name of its container, and "a" is the name of its container, stopping
   * at the container in main. If any reactor in the hierarchy is in a bank of reactors then, it
   * will appear as a[index]. Similarly, if c is a port in a multiport, it will appear as c[index].
   *
   * @return The full name of this instance.
   */
  public String getFullName() {
    return getFullNameWithJoiner(".");
  }

  /**
   * Return a string of the form "a.b.c", where "." is replaced by the specified joiner, "c" is the
   * name of this instance, "b" is the name of its container, and "a" is the name of its container,
   * stopping at the container in main.
   *
   * @return A string representing this instance.
   */
  public String getFullNameWithJoiner(String joiner) {
    // This is not cached because _uniqueID is cached.
    if (parent == null) {
      return this.getName();
    } else if (getMode(true) != null) {
      return parent.getFullNameWithJoiner(joiner)
          + joiner
          + getMode(true).getName()
          + joiner
          + this.getName();
    } else {
      return parent.getFullNameWithJoiner(joiner) + joiner + this.getName();
    }
  }

  /**
   * Return the name of this instance as given in its definition. Note that this is unique only
   * relative to other instances with the same parent.
   *
   * @return The name of this instance within its parent.
   */
  public abstract String getName();

  /** Return the parent or null if this is a top-level reactor. */
  public ReactorInstance getParent() {
    return parent;
  }

  /**
   * Return the parent at the given depth or null if there is no parent at the given depth.
   *
   * @param d The depth.
   */
  public ReactorInstance getParent(int d) {
    if (d >= depth || d < 0) return null;
    ReactorInstance p = parent;
    while (p != null) {
      if (p.depth == d) return p;
      p = p.parent;
    }
    return null;
  }

  /**
   * Return the width of this instance, which in this base class is 1. Subclasses PortInstance and
   * ReactorInstance change this to the multiport and bank widths respectively.
   */
  public int getWidth() {
    return width;
  }

  /**
   * Return true if this instance has the specified parent (possibly indirectly, anywhere up the
   * hierarchy).
   */
  public boolean hasParent(ReactorInstance container) {

    ReactorInstance p = parent;

    while (p != null) {
      if (p == container) return true;
      p = p.parent;
    }
    return false;
  }

  /** Return a list of all the parents starting with the root(). */
  public List<ReactorInstance> parents() {
    List<ReactorInstance> result = new ArrayList<ReactorInstance>(depth + 1);
    if (this instanceof ReactorInstance && parent == null) {
      // This is the top level, so it must be a reactor.
      result.add((ReactorInstance) this);
    }
    ReactorInstance container = parent;
    while (container != null) {
      result.add(container);
      container = container.parent;
    }
    return result;
  }

  /**
   * Return the root reactor, which is the top-level parent.
   *
   * @return The top-level parent.
   */
  public ReactorInstance root() {
    if (parent != null) {
      return parent.root();
    } else {
      return (ReactorInstance) this;
    }
  }

  /**
   * Set the width. This method is here for testing only and should not be used for any other
   * purpose.
   *
   * @param width The new width.
   */
  public void setWidth(int width) {
    this.width = width;
  }

  /**
   * Return an identifier for this instance, which has the form "a_b_c" or "a_b_c_n", where "c" is
   * the name of this instance, "b" is the name of its container, and "a" is the name of its
   * container, stopping at the container in main. All names are converted to lower case. The suffix
   * _n is usually omitted, but it is possible to get name collisions using the above scheme, in
   * which case _n will be an increasing integer until there is no collision. If the length of the
   * root of the name as calculated above (the root is without the _n suffix) is longer than the
   * static variable identifierLengthLimit, then the name will be truncated. The returned name will
   * be the tail of the name calculated above with the prefix '_'.
   *
   * @return An identifier for this instance that is guaranteed to be unique within the top-level
   *     parent.
   */
  public String uniqueID() {
    if (_uniqueID == null) {
      // Construct the unique ID only if it has not been
      // previously constructed.
      String prefix = getFullNameWithJoiner("_").toLowerCase();

      // Replace all non-alphanumeric (Latin) characters with underscore.
      prefix = prefix.replaceAll("[^A-Za-z0-9]", "_");

      // Truncate, if necessary.
      if (prefix.length() > identifierLengthLimit) {
        prefix = '_' + prefix.substring(prefix.length() - identifierLengthLimit + 1);
      }

      // Ensure uniqueness.
      ReactorInstance toplevel = root();
      if (toplevel.uniqueIDCount == null) {
        toplevel.uniqueIDCount = new HashMap<String, Integer>();
      }
      var count = toplevel.uniqueIDCount.get(prefix);
      if (count == null) {
        toplevel.uniqueIDCount.put(prefix, 1);
        _uniqueID = prefix;
      } else {
        toplevel.uniqueIDCount.put(prefix, count + 1);
        // NOTE: The length of this name could exceed
        // identifierLengthLimit. Is this OK?
        _uniqueID = prefix + '_' + (count + 1);
      }
    }
    return _uniqueID;
  }

  /**
   * Returns the directly/indirectly enclosing mode.
   *
   * @param direct flag whether to check only for direct enclosing mode or also consider modes of
   *     parent reactor instances.
   * @return The mode, if any, null otherwise.
   */
  public ModeInstance getMode(boolean direct) {
    ModeInstance mode = null;
    if (parent != null) {
      if (!parent.modes.isEmpty()) {
        mode = parent.modes.stream().filter(it -> it.contains(this)).findFirst().orElse(null);
      }
      if (mode == null && !direct) {
        mode = parent.getMode(false);
      }
    }
    return mode;
  }

  //////////////////////////////////////////////////////
  //// Protected fields.

  /** The Instantiation AST object from which this was created. */
  T definition;

  /** The reactor instance that creates this instance. */
  ReactorInstance parent;

  /**
   * Map from a name of the form a_b_c to the number of unique IDs with that prefix that have been
   * already assigned. If none have been assigned, then there is no entry in this map. This map
   * should be non-null only for the main reactor (the top level).
   */
  HashMap<String, Integer> uniqueIDCount;

  /**
   * The width of this instance. This is 1 for everything except a PortInstance representing a
   * multiport and a ReactorInstance representing a bank.
   */
  int width = 1;

  //////////////////////////////////////////////////////
  //// Protected fields.

  /**
   * The depth in the hierarchy of this instance. This is 0 for main or federated, 1 for the
   * reactors immediately contained, etc.
   */
  protected int depth = 0;

  //////////////////////////////////////////////////////
  //// Private fields.

  /** The full name of this instance. */
  private String _fullName = null;

  /** Unique ID for this instance. This is null until uniqueID() is called. */
  private String _uniqueID = null;
}
