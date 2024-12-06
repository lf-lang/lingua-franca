package org.lflang.analyses.uclid;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import org.json.JSONArray;
import org.json.JSONObject;

/**
 * A class that represents the data of a reaction.
 *
 * @name Reaction name (String)
 * @inputs The inputs of the reaction (List of Arugments)
 * @outputs The outputs of the reaction (List of Arguments)
 * @types Types of inputs and outputs of the reaction (Dictionary)
 */
public class ReactionData {
  private String name;
  public List<Argument> inputs = new ArrayList<>();
  public List<Argument> outputs = new ArrayList<>();
  public Map<String, Map<String, Argument>> types = new HashMap<>();
  public List<UclCall> uclCalls = new ArrayList<>();

  public ReactionData(String name) {
    this.name = name;
  }

  public void addInput() {
    this.inputs.add(new Argument());
  }

  public void addOutput() {
    this.outputs.add(new Argument());
  }

  public void addType(String name) {
    this.types.put(name, new HashMap<>());
  }

  public String getName() {
    return this.name;
  }

  public Map<String, Argument> getType(String name) {
    return this.types.get(name);
  }

  public String toJSON() {
    JSONObject json = new JSONObject();
    JSONObject types_json = new JSONObject();
    JSONArray inputs_json = new JSONArray();
    JSONArray outputs_json = new JSONArray();
    JSONArray uclCalls_json = new JSONArray();
    json.put("name", this.name);
    for (Map.Entry<String, Map<String, Argument>> entry : this.types.entrySet()) {
      JSONObject type_json = new JSONObject();
      type_json.put("type", "struct");
      JSONObject fields_json = new JSONObject();
      for (Map.Entry<String, Argument> field : entry.getValue().entrySet()) {
        JSONObject field_json = new JSONObject();
        field_json.put("tgttype", field.getValue().getTgtType());
        field_json.put("ucltype", field.getValue().getUclType());
        fields_json.put(field.getKey(), field_json);
      }
      type_json.put("fields", fields_json);
      types_json.put(entry.getKey(), type_json);
    }
    json.put("types", types_json);
    for (Argument arg : this.inputs) {
      JSONObject input = new JSONObject();
      input.put("tgtname", arg.getTgtName());
      input.put("tgttype", arg.getTgtType());
      input.put("uclname", arg.getUclName());
      input.put("ucltype", arg.getUclType());
      inputs_json.put(input);
    }
    json.put("inputs", inputs_json);
    for (Argument arg : this.outputs) {
      JSONObject output = new JSONObject();
      output.put("tgtname", arg.getTgtName());
      output.put("tgttype", arg.getTgtType());
      output.put("uclname", arg.getUclName());
      output.put("ucltype", arg.getUclType());
      outputs_json.put(output);
    }
    json.put("outputs", outputs_json);
    for (UclCall call : this.uclCalls) {
      JSONObject uclCall = new JSONObject();
      JSONArray inputs = new JSONArray();
      JSONArray outputs = new JSONArray();
      for (String input : call.inputs) {
        inputs.put(input);
      }
      for (String output : call.outputs) {
        outputs.put(output);
      }
      uclCall.put("inputs", inputs);
      uclCall.put("outputs", outputs);
      uclCall.put("flag", call.flag);
      uclCalls_json.put(uclCall);
    }
    json.put("uclcalls", uclCalls_json);
    return json.toString(4);
  }

  /**
   * A class that represent an input or output argument of a reaction
   *
   * @tgtname The name of the argument in target lang (String)
   * @tgttype The type of the argument in target lang (String)
   * @uclname The name of the argument in Uclid5 (String)
   * @ucltype The type of the argument in Uclid5 (String)
   */
  public class Argument {
    private String tgtname = "";
    private String tgttype = "";
    private String uclname = "";
    private String ucltype = "";

    public Argument() {}

    public String getTgtName() {
      return this.tgtname;
    }

    public String getTgtType() {
      return this.tgttype;
    }

    public String getUclName() {
      return this.uclname;
    }

    public String getUclType() {
      return this.ucltype;
    }

    public void setTgtName(String tgtname) {
      this.tgtname = tgtname;
    }

    public void setTgtType(String tgttype) {
      this.tgttype = tgttype;
    }

    public void setUclName(String uclname) {
      this.uclname = uclname;
    }

    public void setUclType(String ucltype) {
      this.ucltype = ucltype;
    }
  }

  public class UclCall {
    public List<String> inputs = new ArrayList<>();
    public List<String> outputs = new ArrayList<>();
    public String flag = "";
  }
}
