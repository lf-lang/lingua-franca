package org.lflang.target.property;

import org.lflang.MessageReporter;
import org.lflang.ast.ASTUtils;
import org.lflang.lf.Element;
import org.lflang.lf.KeyValuePair;
import org.lflang.lf.KeyValuePairs;
import org.lflang.lf.LfFactory;
import org.lflang.target.property.SSTProperty.SSTOptions;
import org.lflang.target.property.type.DictionaryType;
import org.lflang.target.property.type.DictionaryType.DictionaryElement;
import org.lflang.target.property.type.PrimitiveType;
import org.lflang.target.property.type.TargetPropertyType;

/**
 * Directive to specify SST configuration options.
 */
public final class SSTProperty extends TargetProperty<SSTOptions, DictionaryType> {

  /** Singleton target property instance. */
  public static final SSTProperty INSTANCE = new SSTProperty();

  private SSTProperty() {
    super(DictionaryType.SST_DICT);
  }

  @Override
  public SSTOptions initialValue() {
    return new SSTOptions("", false);
  }

  @Override
  public SSTOptions fromAst(Element node, MessageReporter reporter) {
    if (node.getKeyvalue() != null) {
      String rootPath = "";
      boolean usePermanentDistKey = false;
      for (KeyValuePair entry : node.getKeyvalue().getPairs()) {
        SSTOption option = (SSTOption) DictionaryType.SST_DICT.forName(entry.getName());
        switch (option) {
          case ROOT_PATH -> rootPath = ASTUtils.elementToSingleString(entry.getValue());
          case USE_PERMANENT_DIST_KEY -> usePermanentDistKey = ASTUtils.toBoolean(entry.getValue());
        }
      }
      return new SSTOptions(rootPath, usePermanentDistKey);
    }
    return initialValue();
  }

  @Override
  protected SSTOptions fromString(String string, MessageReporter reporter) {
    throw new UnsupportedOperationException("SST property must be a dictionary.");
  }

  @Override
  public Element toAstElement(SSTOptions value) {
    if (value.rootPath().isEmpty() && !value.usePermanentDistKey()) {
      return null;
    }
    Element e = LfFactory.eINSTANCE.createElement();
    KeyValuePairs kvp = LfFactory.eINSTANCE.createKeyValuePairs();
    for (SSTOption opt : SSTOption.values()) {
      KeyValuePair pair = LfFactory.eINSTANCE.createKeyValuePair();
      pair.setName(opt.toString());
      switch (opt) {
        case ROOT_PATH -> {
          if (!value.rootPath().isEmpty()) {
            pair.setValue(ASTUtils.toElement(value.rootPath()));
            kvp.getPairs().add(pair);
          }
        }
        case USE_PERMANENT_DIST_KEY -> {
          pair.setValue(ASTUtils.toElement(value.usePermanentDistKey()));
          kvp.getPairs().add(pair);
        }
      }
    }
    e.setKeyvalue(kvp);
    return e;
  }

  @Override
  public String name() {
    return "sst";
  }

  public record SSTOptions(String rootPath, boolean usePermanentDistKey) {}

  public enum SSTOption implements DictionaryElement {
    ROOT_PATH("sst-root-path", PrimitiveType.STRING),
    USE_PERMANENT_DIST_KEY("use-permanent-distribution-key", PrimitiveType.BOOLEAN);

    public final PrimitiveType type;
    public final String option;

    SSTOption(String option, PrimitiveType type) {
      this.option = option;
      this.type = type;
    }

    @Override
    public TargetPropertyType getType() {
      return this.type;
    }

    @Override
    public String toString() {
      return this.option;
    }
  }
}
