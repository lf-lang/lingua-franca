package org.lflang.tests.compiler;

import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import java.io.IOException;
import java.nio.file.Files;
import java.nio.file.Path;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.io.TempDir;
import org.lflang.DefaultMessageReporter;
import org.lflang.MessageReporter;
import org.lflang.TimeUnit;
import org.lflang.TimeValue;
import org.lflang.generator.ReactionInstance;
import org.lflang.generator.ReactorInstance;
import org.lflang.lf.LfFactory;
import org.lflang.lf.Reaction;
import org.lflang.lf.Reactor;
import org.lflang.pretvm.dag.Dag;
import org.lflang.pretvm.dag.IntervalNode;
import org.lflang.pretvm.dag.JobNode;
import org.lflang.pretvm.dag.TimeNode;
import org.lflang.pretvm.scheduler.TaprioScheduleGenerator;

/**
 * Tests for {@link TaprioScheduleGenerator}.
 *
 * @ingroup Tests
 */
public class TaprioScheduleGeneratorTest {

  private static final LfFactory factory = LfFactory.eINSTANCE;
  private final MessageReporter reporter = new DefaultMessageReporter();

  /**
   * Test TAPRIO generation with two federates communicating via one inter-federate edge.
   *
   * <p>DAG structure: start -> job_A (WCET=200ms) -> job_B (WCET=300ms) -> end
   *
   * <p>job_A belongs to federate A (TC 0), job_B belongs to federate B (TC 1). The edge job_A ->
   * job_B is an inter-federate edge. Makespan to job_A = 200ms.
   *
   * <p>Expected schedule:
   *
   * <ul>
   *   <li>TC 0 window: [0, 200ms] (gate mask 01)
   *   <li>Best-effort: [200ms, 1000ms] (gate mask 03, since 2 TCs)
   * </ul>
   */
  @Test
  public void testTwoFederatesTaprio(@TempDir Path tempDir) throws IOException {
    Reactor mainReactor = factory.createReactor();
    ReactorInstance maini = new ReactorInstance(mainReactor, reporter);

    ReactorInstance federateA = newReactor("A", maini);
    ReactorInstance federateB = newReactor("B", maini);

    ReactionInstance reactionA = newReaction(federateA, 0);
    reactionA.wcet = new TimeValue(200, TimeUnit.MILLI);

    ReactionInstance reactionB = newReaction(federateB, 0);
    reactionB.wcet = new TimeValue(300, TimeUnit.MILLI);

    Dag dag = new Dag();

    TimeNode start = new TimeNode(TimeValue.ZERO);
    dag.addNode(start);
    dag.start = start;

    JobNode jobA = new JobNode(reactionA);
    dag.addNode(jobA);

    JobNode jobB = new JobNode(reactionB);
    dag.addNode(jobB);

    TimeNode end = new TimeNode(new TimeValue(1000, TimeUnit.MILLI));
    dag.addNode(end);
    dag.end = end;
    dag.tail = end;

    dag.addEdge(start, jobA);
    dag.addEdge(jobA, jobB);
    dag.addEdge(jobB, end);

    TaprioScheduleGenerator generator = new TaprioScheduleGenerator(maini, tempDir);
    long hyperperiodNs = new TimeValue(1000, TimeUnit.MILLI).toNanoSeconds();
    generator.generate(dag, hyperperiodNs, 0);

    Path scriptPath = tempDir.resolve("taprio_setup.sh");
    assertTrue(Files.exists(scriptPath), "taprio_setup.sh should be generated");

    String script = Files.readString(scriptPath);

    assertTrue(script.startsWith("#!/bin/bash"), "Script should start with shebang");
    assertTrue(script.contains("num_tc 2"), "Should have 2 traffic classes");
    assertTrue(script.contains("queues 1@0 1@1"), "Should have queue assignments");
    assertTrue(script.contains("flags 0x2"), "Should have flags 0x2");

    // jobA is the sender (TC 0) with makespan = 200ms = 200000000 ns.
    assertTrue(
        script.contains("sched-entry S 01 200000000"),
        "Should have sched-entry for TC 0 at 200ms. Script:\n" + script);
    // Remaining: 1000ms - 200ms = 800ms, all gates open (mask 03 for 2 TCs).
    assertTrue(
        script.contains("sched-entry S 03 800000000"),
        "Should have best-effort entry for remaining time. Script:\n" + script);
  }

  /**
   * Test that TAPRIO is skipped when there are no inter-federate edges.
   */
  @Test
  public void testNoInterFederateEdgesSkips(@TempDir Path tempDir) throws IOException {
    Reactor mainReactor = factory.createReactor();
    ReactorInstance maini = new ReactorInstance(mainReactor, reporter);

    ReactorInstance federateA = newReactor("A", maini);

    ReactionInstance reaction1 = newReaction(federateA, 0);
    reaction1.wcet = new TimeValue(100, TimeUnit.MILLI);

    ReactionInstance reaction2 = newReaction(federateA, 1);
    reaction2.wcet = new TimeValue(200, TimeUnit.MILLI);

    Dag dag = new Dag();
    TimeNode start = new TimeNode(TimeValue.ZERO);
    dag.addNode(start);
    dag.start = start;

    JobNode job1 = new JobNode(reaction1);
    dag.addNode(job1);

    JobNode job2 = new JobNode(reaction2);
    dag.addNode(job2);

    TimeNode end = new TimeNode(new TimeValue(500, TimeUnit.MILLI));
    dag.addNode(end);
    dag.end = end;
    dag.tail = end;

    dag.addEdge(start, job1);
    dag.addEdge(job1, job2);
    dag.addEdge(job2, end);

    TaprioScheduleGenerator generator = new TaprioScheduleGenerator(maini, tempDir);
    generator.generate(dag, 500_000_000L, 0);

    Path scriptPath = tempDir.resolve("taprio_setup.sh");
    assertFalse(Files.exists(scriptPath), "taprio_setup.sh should NOT be generated");
  }

  /**
   * Test that TAPRIO is skipped when a reaction has unannotated WCET (MAX_VALUE).
   */
  @Test
  public void testUnannotatedWcetSkips(@TempDir Path tempDir) throws IOException {
    Reactor mainReactor = factory.createReactor();
    ReactorInstance maini = new ReactorInstance(mainReactor, reporter);

    ReactorInstance federateA = newReactor("A", maini);
    ReactorInstance federateB = newReactor("B", maini);

    // Leave wcet as default (MAX_VALUE).
    ReactionInstance reactionA = newReaction(federateA, 0);

    ReactionInstance reactionB = newReaction(federateB, 0);
    reactionB.wcet = new TimeValue(300, TimeUnit.MILLI);

    Dag dag = new Dag();
    TimeNode start = new TimeNode(TimeValue.ZERO);
    dag.addNode(start);
    dag.start = start;

    JobNode jobA = new JobNode(reactionA);
    dag.addNode(jobA);

    JobNode jobB = new JobNode(reactionB);
    dag.addNode(jobB);

    TimeNode end = new TimeNode(new TimeValue(1000, TimeUnit.MILLI));
    dag.addNode(end);
    dag.end = end;
    dag.tail = end;

    dag.addEdge(start, jobA);
    dag.addEdge(jobA, jobB);
    dag.addEdge(jobB, end);

    TaprioScheduleGenerator generator = new TaprioScheduleGenerator(maini, tempDir);
    generator.generate(dag, 1_000_000_000L, 0);

    Path scriptPath = tempDir.resolve("taprio_setup.sh");
    assertFalse(
        Files.exists(scriptPath),
        "taprio_setup.sh should NOT be generated when WCET is unannotated");
  }

  /**
   * Test TAPRIO generation with three federates and multiple inter-federate edges.
   *
   * <p>DAG: start -> jobA (200ms) -> jobB (300ms) -> jobC (100ms) -> end
   *
   * <p>Inter-federate edges: jobA -> jobB, jobB -> jobC. Senders: A (makespan 200ms), B (makespan
   * 500ms).
   *
   * <p>Expected schedule:
   *
   * <ul>
   *   <li>TC 0: [0, 200ms]
   *   <li>TC 1: [200ms, 500ms]
   *   <li>Best-effort: [500ms, 1000ms]
   * </ul>
   */
  @Test
  public void testThreeFederatesTaprio(@TempDir Path tempDir) throws IOException {
    Reactor mainReactor = factory.createReactor();
    ReactorInstance maini = new ReactorInstance(mainReactor, reporter);

    ReactorInstance federateA = newReactor("A", maini);
    ReactorInstance federateB = newReactor("B", maini);
    ReactorInstance federateC = newReactor("C", maini);

    ReactionInstance reactionA = newReaction(federateA, 0);
    reactionA.wcet = new TimeValue(200, TimeUnit.MILLI);

    ReactionInstance reactionB = newReaction(federateB, 0);
    reactionB.wcet = new TimeValue(300, TimeUnit.MILLI);

    ReactionInstance reactionC = newReaction(federateC, 0);
    reactionC.wcet = new TimeValue(100, TimeUnit.MILLI);

    Dag dag = new Dag();

    TimeNode start = new TimeNode(TimeValue.ZERO);
    dag.addNode(start);
    dag.start = start;

    JobNode jobA = new JobNode(reactionA);
    dag.addNode(jobA);

    JobNode jobB = new JobNode(reactionB);
    dag.addNode(jobB);

    JobNode jobC = new JobNode(reactionC);
    dag.addNode(jobC);

    TimeNode end = new TimeNode(new TimeValue(1000, TimeUnit.MILLI));
    dag.addNode(end);
    dag.end = end;
    dag.tail = end;

    dag.addEdge(start, jobA);
    dag.addEdge(jobA, jobB);
    dag.addEdge(jobB, jobC);
    dag.addEdge(jobC, end);

    TaprioScheduleGenerator generator = new TaprioScheduleGenerator(maini, tempDir);
    long hyperperiodNs = new TimeValue(1000, TimeUnit.MILLI).toNanoSeconds();
    generator.generate(dag, hyperperiodNs, 0);

    Path scriptPath = tempDir.resolve("taprio_setup.sh");
    assertTrue(Files.exists(scriptPath), "taprio_setup.sh should be generated");

    String script = Files.readString(scriptPath);

    assertTrue(script.contains("num_tc 3"), "Should have 3 traffic classes");
    // TC 0 (federate A) sends at makespan 200ms.
    assertTrue(
        script.contains("sched-entry S 01 200000000"),
        "Should have sched-entry for TC 0 (A). Script:\n" + script);
    // TC 1 (federate B) sends at makespan 500ms, window: 300ms duration.
    assertTrue(
        script.contains("sched-entry S 02 300000000"),
        "Should have sched-entry for TC 1 (B). Script:\n" + script);
    // Remaining: 1000ms - 500ms = 500ms, all gates open (mask 07 for 3 TCs).
    assertTrue(
        script.contains("sched-entry S 07 500000000"),
        "Should have best-effort entry. Script:\n" + script);
  }

  /**
   * Test with an IntervalNode in the path to verify interval cost is included in makespan.
   *
   * <p>DAG: start -> interval(50ms) -> jobA(200ms) -> jobB(300ms) -> end
   *
   * <p>Makespan to jobA = 50ms + 200ms = 250ms (jobA is the sender).
   */
  @Test
  public void testWithIntervalNode(@TempDir Path tempDir) throws IOException {
    Reactor mainReactor = factory.createReactor();
    ReactorInstance maini = new ReactorInstance(mainReactor, reporter);

    ReactorInstance federateA = newReactor("A", maini);
    ReactorInstance federateB = newReactor("B", maini);

    ReactionInstance reactionA = newReaction(federateA, 0);
    reactionA.wcet = new TimeValue(200, TimeUnit.MILLI);

    ReactionInstance reactionB = newReaction(federateB, 0);
    reactionB.wcet = new TimeValue(300, TimeUnit.MILLI);

    Dag dag = new Dag();

    TimeNode start = new TimeNode(TimeValue.ZERO);
    dag.addNode(start);
    dag.start = start;

    IntervalNode interval = new IntervalNode(new TimeValue(50, TimeUnit.MILLI));
    dag.addNode(interval);

    JobNode jobA = new JobNode(reactionA);
    dag.addNode(jobA);

    JobNode jobB = new JobNode(reactionB);
    dag.addNode(jobB);

    TimeNode end = new TimeNode(new TimeValue(1000, TimeUnit.MILLI));
    dag.addNode(end);
    dag.end = end;
    dag.tail = end;

    dag.addEdge(start, interval);
    dag.addEdge(interval, jobA);
    dag.addEdge(jobA, jobB);
    dag.addEdge(jobB, end);

    TaprioScheduleGenerator generator = new TaprioScheduleGenerator(maini, tempDir);
    long hyperperiodNs = new TimeValue(1000, TimeUnit.MILLI).toNanoSeconds();
    generator.generate(dag, hyperperiodNs, 0);

    Path scriptPath = tempDir.resolve("taprio_setup.sh");
    assertTrue(Files.exists(scriptPath), "taprio_setup.sh should be generated");

    String script = Files.readString(scriptPath);

    // Makespan to jobA = 50ms + 200ms = 250ms = 250000000 ns.
    assertTrue(
        script.contains("sched-entry S 01 250000000"),
        "Should have sched-entry for TC 0 (A) at 250ms. Script:\n" + script);
    // Remaining: 1000ms - 250ms = 750ms, all gates open (mask 03 for 2 TCs).
    assertTrue(
        script.contains("sched-entry S 03 750000000"),
        "Should have best-effort entry. Script:\n" + script);
  }

  private ReactorInstance newReactor(String name, ReactorInstance container) {
    Reactor r = factory.createReactor();
    r.setName(name);
    ReactorInstance ri = new ReactorInstance(r, container, reporter);
    container.children.add(ri);
    return ri;
  }

  private ReactionInstance newReaction(ReactorInstance parent, int index) {
    Reaction r = factory.createReaction();
    ReactionInstance ri = new ReactionInstance(r, parent, index);
    parent.reactions.add(ri);
    return ri;
  }
}
