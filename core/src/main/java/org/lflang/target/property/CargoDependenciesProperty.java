package org.lflang.target.property;

import java.util.HashMap;
import java.util.Map;
import org.lflang.MessageReporter;
import org.lflang.generator.InvalidLfSourceException;
import org.lflang.generator.rust.CargoDependencySpec;
import org.lflang.generator.rust.CargoDependencySpec.CargoDependenciesPropertyType;
import org.lflang.lf.Element;
import org.lflang.lf.KeyValuePair;
import org.lflang.lf.KeyValuePairs;
import org.lflang.lf.LfFactory;

/**
 * Dependency specifications for Cargo. This property looks like this:
 *
 * <pre>{@code
 * cargo-dependencies: {
 *    // Name-of-the-crate: "version"
 *    rand: "0.8",
 *    // Equivalent to using an explicit map:
 *    rand: {
 *      version: "0.8"
 *    },
 *    // The map allows specifying more details
 *    rand: {
 *      // A path to a local unpublished crate.
 *      // Note 'path' is mutually exclusive with 'version'.
 *      path: "/home/me/Git/local-rand-clone"
 *    },
 *    rand: {
 *      version: "0.8",
 *      // you can specify cargo features
 *      features: ["some-cargo-feature",]
 *    }
 * }
 * }</pre>
 */
public final class CargoDependenciesProperty
    extends TargetProperty<Map<String, CargoDependencySpec>, CargoDependenciesPropertyType> {

  /** Singleton target property instance. */
  public static final CargoDependenciesProperty INSTANCE = new CargoDependenciesProperty();

  private CargoDependenciesProperty() {
    super(new CargoDependenciesPropertyType());
  }

  @Override
  public Map<String, CargoDependencySpec> initialValue() {
    return new HashMap<>();
  }

  @Override
  protected Map<String, CargoDependencySpec> fromAst(Element node, MessageReporter reporter) {
    Map<String, CargoDependencySpec> map = null;
    try {
      map = CargoDependencySpec.parseAll(node);
    } catch (InvalidLfSourceException e) {
      reporter.at(e.getNode()).error(e.getMessage());
    }
    return map;
  }

  @Override
  protected Map<String, CargoDependencySpec> fromString(String string, MessageReporter reporter) {
    throw new UnsupportedOperationException("Not supported yet.");
  }

  @Override
  public Element toAstElement(Map<String, CargoDependencySpec> value) {
    if (value.size() == 0) {
      return null;
    } else {
      Element e = LfFactory.eINSTANCE.createElement();
      KeyValuePairs kvp = LfFactory.eINSTANCE.createKeyValuePairs();
      for (var ent : value.entrySet()) {
        KeyValuePair pair = LfFactory.eINSTANCE.createKeyValuePair();
        pair.setName(ent.getKey());
        pair.setValue(CargoDependencySpec.extractSpec(ent.getValue()));
        kvp.getPairs().add(pair);
      }
      e.setKeyvalue(kvp);
      return e;
    }
  }

  @Override
  public String name() {
    return "cargo-dependencies";
  }
}
