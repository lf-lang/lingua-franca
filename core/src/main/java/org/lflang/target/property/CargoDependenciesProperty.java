package org.lflang.target.property;

import java.util.Arrays;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import org.lflang.MessageReporter;
import org.lflang.Target;
import org.lflang.TargetPropertyConfig;
import org.lflang.generator.rust.CargoDependencySpec;
import org.lflang.generator.rust.CargoDependencySpec.CargoDependenciesPropertyType;
import org.lflang.lf.Element;
import org.lflang.lf.KeyValuePair;
import org.lflang.lf.KeyValuePairs;
import org.lflang.lf.LfFactory;

public class CargoDependenciesProperty
    extends TargetPropertyConfig<Map<String, CargoDependencySpec>> {

  public CargoDependenciesProperty() {
    super(CargoDependenciesPropertyType.INSTANCE);
  }

  @Override
  public Map<String, CargoDependencySpec> initialValue() {
    return new HashMap<>();
  }

  @Override
  protected Map<String, CargoDependencySpec> fromAst(Element value, MessageReporter err) {
    return CargoDependencySpec.parseAll(value);
  }

  @Override
  protected Map<String, CargoDependencySpec> fromString(String value, MessageReporter err) {
    throw new UnsupportedOperationException("Not supported yet.");
  }

  @Override
  public List<Target> supportedTargets() {
    return Arrays.asList(Target.Rust);
  }

  @Override
  public Element toAstElement() {
    var deps = this.value;
    if (deps.size() == 0) {
      return null;
    } else {
      Element e = LfFactory.eINSTANCE.createElement();
      KeyValuePairs kvp = LfFactory.eINSTANCE.createKeyValuePairs();
      for (var ent : deps.entrySet()) {
        KeyValuePair pair = LfFactory.eINSTANCE.createKeyValuePair();
        pair.setName(ent.getKey());
        pair.setValue(CargoDependencySpec.extractSpec(ent.getValue()));
        kvp.getPairs().add(pair);
      }
      e.setKeyvalue(kvp);
      return e;
    }
  }
}