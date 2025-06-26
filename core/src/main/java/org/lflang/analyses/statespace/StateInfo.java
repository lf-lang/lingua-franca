package org.lflang.analyses.statespace;

import java.util.ArrayList;
import java.util.HashMap;
import org.lflang.TimeTag;

/** A class that represents information in a step in a counterexample trace */
public class StateInfo {

  public ArrayList<String> reactions = new ArrayList<>();
  public TimeTag tag;
  public HashMap<String, String> variables = new HashMap<>();
  public HashMap<String, String> triggers = new HashMap<>();
  public HashMap<String, String> scheduled = new HashMap<>();
  public HashMap<String, String> payloads = new HashMap<>();

  public void display() {
    System.out.println(
        String.join(
            "\n",
            "/**************************/",
            "reactions: " + reactions,
            "tag: " + tag,
            "variables: " + variables,
            "triggers: " + triggers,
            "scheduled: " + scheduled,
            "payloads: " + payloads,
            "/**************************/"));
  }
}
