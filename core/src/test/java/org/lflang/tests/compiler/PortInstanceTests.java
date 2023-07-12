package org.lflang.tests.compiler;

import java.util.List;
import org.junit.jupiter.api.Assertions;
import org.junit.jupiter.api.Test;
import org.lflang.DefaultMessageReporter;
import org.lflang.MessageReporter;
import org.lflang.generator.PortInstance;
import org.lflang.generator.ReactionInstance;
import org.lflang.generator.ReactorInstance;
import org.lflang.generator.RuntimeRange;
import org.lflang.generator.SendRange;
import org.lflang.lf.LfFactory;
import org.lflang.lf.Port;
import org.lflang.lf.Reaction;
import org.lflang.lf.Reactor;

public class PortInstanceTests {

  private MessageReporter reporter = new DefaultMessageReporter();
  private static LfFactory factory = LfFactory.eINSTANCE;

  @Test
  public void createRange() throws Exception {
    Reactor main = factory.createReactor();
    ReactorInstance maini = new ReactorInstance(main, reporter);

    ReactorInstance a = newReactor("A", maini);
    ReactorInstance b = newReactor("B", maini);
    ReactorInstance c = newReactor("C", maini);

    PortInstance p = newOutputPort("p", a);
    PortInstance q = newInputPort("q", b);
    PortInstance r = newInputPort("r", c);

    Assertions.assertEquals(".A.p", p.getFullName());

    connect(p, q);
    connect(p, r);

    List<SendRange> sr = p.eventualDestinations();
    // Destinations should be empty because there are no reactions.
    Assertions.assertEquals("[]", sr.toString());

    // Clear caches to make a mutation.
    maini.clearCaches();
    newReaction(q);
    // Re-retrieve destinations.
    sr = p.eventualDestinations();
    Assertions.assertEquals("[.A.p(0,1)->[.B.q(0,1)]]", sr.toString());

    maini.clearCaches();
    newReaction(r);
    // Re-retrieve destinations.
    sr = p.eventualDestinations();
    Assertions.assertEquals("[.A.p(0,1)->[.B.q(0,1), .C.r(0,1)]]", sr.toString());

    // Now test multiports.
    p.setWidth(3);
    r.setWidth(2);
    // Have to redo the connections.
    clearConnections(maini);
    maini.clearCaches();
    connect(p, 0, 1, q, 0, 1);
    connect(p, 1, 2, r, 0, 2);

    // Re-retrieve destinations.
    sr = p.eventualDestinations();
    Assertions.assertEquals("[.A.p(0,1)->[.B.q(0,1)], .A.p(1,2)->[.C.r(0,2)]]", sr.toString());

    // More complicated multiport connection.
    clearConnections(maini);
    maini.clearCaches();

    ReactorInstance d = newReactor("D", maini);
    PortInstance v = newOutputPort("v", d);
    v.setWidth(2);
    q.setWidth(3);
    connect(v, 0, 2, q, 0, 2);
    connect(p, 0, 1, q, 2, 1);
    connect(p, 1, 2, r, 0, 2);

    sr = p.eventualDestinations();
    Assertions.assertEquals("[.A.p(0,1)->[.B.q(2,1)], .A.p(1,2)->[.C.r(0,2)]]", sr.toString());

    // Additional multicast connection.
    maini.clearCaches();
    ReactorInstance e = newReactor("E", maini);
    PortInstance s = newPort("s", e);
    s.setWidth(3);
    newReaction(s);
    connect(p, s);

    sr = p.eventualDestinations();
    Assertions.assertEquals(
        "[.A.p(0,1)->[.B.q(2,1), .E.s(0,1)], .A.p(1,2)->[.C.r(0,2), .E.s(1,2)]]", sr.toString());

    // Add hierarchical reactors that further split the ranges.
    maini.clearCaches();
    ReactorInstance f = newReactor("F", e);
    PortInstance t = newPort("t", f);
    newReaction(t);
    ReactorInstance g = newReactor("G", e);
    PortInstance u = newPort("u", g);
    u.setWidth(2);
    newReaction(u);
    connect(s, 0, 1, t, 0, 1);
    connect(s, 1, 2, u, 0, 2);

    sr = p.eventualDestinations();
    // FIXME: Multicast destinations should be able to be reported in arbitrary order.
    Assertions.assertEquals(
        "[.A.p(0,1)->[.E.F.t(0,1), .E.s(0,1), .B.q(2,1)], .A.p(1,2)->[.E.G.u(0,2), .E.s(1,2),"
            + " .C.r(0,2)]]",
        sr.toString());
  }

  @Test
  public void multiportDestination() throws Exception {
    Reactor main = factory.createReactor();
    ReactorInstance maini = new ReactorInstance(main, reporter);

    ReactorInstance a = newReactor("A", maini);
    ReactorInstance b = newReactor("B", maini);
    b.setWidth(4);

    PortInstance p = newOutputPort("p", a);
    PortInstance q = newInputPort("q", b);

    connect(p, 0, 1, q, 0, 4);

    List<SendRange> sr = p.eventualDestinations();
    // Destination has no reactions, so empty list is right.
    Assertions.assertEquals("[]", sr.toString());

    maini.clearCaches();
    newReaction(q);
    sr = p.eventualDestinations();
    Assertions.assertEquals("[.A.p(0,1)->[.B.q(0,4)]]", sr.toString());
  }

  /** Clear connections. This recursively clears them for all contained reactors. */
  protected void clearConnections(ReactorInstance r) {
    for (PortInstance p : r.inputs) {
      p.getDependentPorts().clear();
    }
    for (PortInstance p : r.outputs) {
      p.getDependentPorts().clear();
    }
    for (ReactorInstance child : r.children) {
      clearConnections(child);
    }
  }

  /**
   * Simple connection of two ports. This should be used only for connections that would be allowed
   * in the syntax (i.e., no cross-hierarchy connections), but this is not checked.
   *
   * @param src The sending port.
   * @param dst The receiving port.
   */
  protected void connect(PortInstance src, PortInstance dst) {
    RuntimeRange<PortInstance> srcRange = new RuntimeRange.Port(src);
    RuntimeRange<PortInstance> dstRange = new RuntimeRange.Port(dst);
    ReactorInstance.connectPortInstances(srcRange, dstRange, null);
  }

  /**
   * Connection between multiports. This should be used only for connections that would be allowed
   * in the syntax (i.e., no cross-hierarchy connections), but this is not checked.
   *
   * @param src The sending port.
   * @param dst The receiving port.
   */
  protected void connect(
      PortInstance src, int srcStart, int srcWidth, PortInstance dst, int dstStart, int dstWidth) {
    RuntimeRange<PortInstance> srcRange = new RuntimeRange.Port(src, srcStart, srcWidth, null);
    RuntimeRange<PortInstance> dstRange = new RuntimeRange.Port(dst, dstStart, dstWidth, null);
    ReactorInstance.connectPortInstances(srcRange, dstRange, null);
  }

  protected PortInstance newPort(String name, ReactorInstance container) {
    Port p = factory.createPort();
    p.setName(name);
    return new PortInstance(p, container, reporter);
  }

  protected PortInstance newInputPort(String name, ReactorInstance container) {
    PortInstance pi = newPort(name, container);
    container.inputs.add(pi);
    return pi;
  }

  protected PortInstance newOutputPort(String name, ReactorInstance container) {
    PortInstance pi = newPort(name, container);
    container.outputs.add(pi);
    return pi;
  }

  /**
   * Return a new reaction triggered by the specified port.
   *
   * @param trigger The triggering port.
   */
  protected ReactionInstance newReaction(PortInstance trigger) {
    Reaction r = factory.createReaction();
    ReactionInstance result =
        new ReactionInstance(r, trigger.getParent(), trigger.getDependentReactions().size());
    trigger.getDependentReactions().add(result);
    trigger.getParent().reactions.add(result);
    return result;
  }

  protected ReactorInstance newReactor(String name, ReactorInstance container) {
    Reactor r = factory.createReactor();
    r.setName(name);
    ReactorInstance ri = new ReactorInstance(r, container, reporter);
    container.children.add(ri);
    return ri;
  }
}
