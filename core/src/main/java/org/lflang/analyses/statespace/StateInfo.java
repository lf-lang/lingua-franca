package org.lflang.analyses.statespace;

import java.util.ArrayList;
import java.util.HashMap;

/** A class that represents information in a step in a counterexample trace */
public class StateInfo {

  public ArrayList<String> reactions = new ArrayList<>();
  public Tag tag;
  public HashMap<String, String> variables = new HashMap<>();
  public HashMap<String, String> triggers = new HashMap<>();
  public HashMap<String, String> scheduled = new HashMap<>();
  public HashMap<String, String> payloads = new HashMap<>();

  public void display() {
    System.out.println("reactions: " + reactions);
    System.out.println("tag: " + tag);
    System.out.println("variables: " + variables);
    System.out.println("triggers: " + triggers);
    System.out.println("scheduled: " + scheduled);
    System.out.println("payloads: " + payloads);
  }
}
