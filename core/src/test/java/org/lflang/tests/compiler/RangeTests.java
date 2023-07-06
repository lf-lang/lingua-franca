package org.lflang.tests.compiler;

import java.util.List;
import java.util.Set;
import org.junit.jupiter.api.Assertions;
import org.junit.jupiter.api.Test;
import org.lflang.DefaultMessageReporter;
import org.lflang.MessageReporter;
import org.lflang.generator.PortInstance;
import org.lflang.generator.ReactorInstance;
import org.lflang.generator.RuntimeRange;
import org.lflang.generator.SendRange;
import org.lflang.lf.LfFactory;
import org.lflang.lf.Port;
import org.lflang.lf.Reactor;

public class RangeTests {

  private MessageReporter reporter = new DefaultMessageReporter();
  private static LfFactory factory = LfFactory.eINSTANCE;

  @Test
  public void createRange() throws Exception {
    Reactor main = factory.createReactor();
    ReactorInstance maini = new ReactorInstance(main, reporter);

    Reactor a = factory.createReactor();
    a.setName("A");
    ReactorInstance ai = new ReactorInstance(a, maini, reporter);
    ai.setWidth(2);

    Reactor b = factory.createReactor();
    b.setName("B");
    ReactorInstance bi = new ReactorInstance(b, ai, reporter);
    bi.setWidth(2);

    Port p = factory.createPort();
    p.setName("P");
    PortInstance pi = new PortInstance(p, bi, reporter);
    pi.setWidth(2);

    Assertions.assertEquals(".A.B.P", pi.getFullName());

    RuntimeRange<PortInstance> range = new RuntimeRange.Port(pi, 3, 4, null);

    Assertions.assertEquals(8, range.maxWidth);

    Assertions.assertEquals(".A.B.P(3,4)", range.toString());

    // The results expected below are derived from the class comment for RuntimeRange,
    // which includes this example.
    List<Integer> instances = range.instances();
    Assertions.assertEquals(List.of(3, 4, 5, 6), instances);
    Set<Integer> parents = range.parentInstances(1);
    Assertions.assertEquals(Set.of(1, 2, 3), parents);

    parents = range.parentInstances(2);
    Assertions.assertEquals(Set.of(0, 1), parents);

    // Test startMR().getDigits.
    Assertions.assertEquals(List.of(1, 1, 0), range.startMR().getDigits());

    // Create a SendRange sending from and to this range.
    SendRange sendRange = new SendRange(pi, 3, 4, null, null);
    sendRange.destinations.add(range);

    // Test getNumberOfDestinationReactors.
    Assertions.assertEquals(3, sendRange.getNumberOfDestinationReactors());

    // Make first interleaved version.
    range = range.toggleInterleaved(bi);
    instances = range.instances();
    Assertions.assertEquals(List.of(3, 4, 6, 5), instances);

    // Test startMR().getDigits.
    Assertions.assertEquals(List.of(1, 1, 0), range.startMR().getDigits());

    // Make second interleaved version.
    range = range.toggleInterleaved(ai);
    instances = range.instances();
    Assertions.assertEquals(List.of(6, 1, 5, 3), instances);

    // Test startMR().getDigits.
    Assertions.assertEquals(List.of(0, 1, 1), range.startMR().getDigits());

    // Test instances of the parent.
    Assertions.assertEquals(Set.of(3, 0, 2, 1), range.parentInstances(1));

    // Add this range to the sendRange destinations and verify
    // that the number of destination reactors becomes 4.
    sendRange.addDestination(range);
    Assertions.assertEquals(4, sendRange.getNumberOfDestinationReactors());

    // Make third interleaved version.
    range = range.toggleInterleaved(bi);
    instances = range.instances();
    Assertions.assertEquals(List.of(5, 2, 6, 3), instances);

    // Test startMR().getDigits.
    Assertions.assertEquals(List.of(1, 0, 1), range.startMR().getDigits());
  }
}
