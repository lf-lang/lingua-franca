package org.lflang.generator.c;

import static org.lflang.AttributeUtils.getEnclaveNumWorkersFromAttribute;
import static org.lflang.AttributeUtils.isEnclave;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.LinkedList;
import java.util.List;
import java.util.Map;
import java.util.Queue;
import org.lflang.generator.ReactorInstance;
import org.lflang.util.Pair;

/**
 * This class contains the CEnclaveInstances and maintains a mapping between them and the
 * ReactorInstance.
 */
public class ReactorEnclaveMap {

  private Map<ReactorInstance, Integer> enclaveIndex = new HashMap<>();
  private List<CEnclaveInstance> enclaves = new ArrayList<>();

  /** Add a new mapping. */
  private void add(ReactorInstance inst, CEnclaveInstance e) {
    if (!enclaves.contains(e)) {
      enclaves.add(e);
    }
    enclaveIndex.put(inst, enclaves.indexOf(e));
  }

  /** Get a CEnclaveInstance. */
  public CEnclaveInstance get(ReactorInstance inst) {
    var idx = enclaveIndex.get(inst);
    return enclaves.get(idx);
  }

  /** Update the CEnclaveInstance. */
  public void set(CEnclaveInstance e) {
    enclaves.set(enclaves.indexOf(e), e);
  }

  public int numEnclaves() {
    return enclaves.size();
  }

  public List<CEnclaveInstance> getEnclaves() {
    return enclaves;
  }

  /**
   * Given the main reactor instance, walk down the containment hierarchy and build the map from
   * ReactorInstance to CEnclaveInstance.
   */
  public void build(ReactorInstance main) {
    Queue<Pair<ReactorInstance, ReactorInstance>> queue = new LinkedList<>();
    CEnclaveInstance enc;
    queue.add(new Pair<>(main, null));
    while (!queue.isEmpty()) {
      Pair<ReactorInstance, ReactorInstance> _current = queue.poll();
      ReactorInstance current = _current.first();
      ReactorInstance parent = _current.second();
      if (parent == null) {
        enc = new CEnclaveInstance(current, 0);
        add(current, enc);
      } else if (isEnclave(current.getDefinition())) {
        enc =
            new CEnclaveInstance(
                current, getEnclaveNumWorkersFromAttribute(current.getDefinition()));
        add(current, enc);
      } else {
        enc = get(parent);
        add(current, enc);
      }
      for (ReactorInstance child : current.children) {
        queue.add(new Pair<>(child, current));
      }
    }
  }
}
