package org.lflang.target.property.type;

import java.util.Arrays;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

import org.lflang.Target;
import org.lflang.TargetPropertyConfig;
import org.lflang.ast.ASTUtils;
import org.lflang.lf.Element;

public class CompileDefinitionsConfig extends TargetPropertyConfig<Map<String, String>> {

    public CompileDefinitionsConfig() {
        super(StringDictionaryType.COMPILE_DEFINITION);
    }

    @Override
    public Map<String, String> initialValue() {
        return new HashMap<>();
    }

    @Override
    protected Map<String, String> parse(Element value) {
        return ASTUtils.elementToStringMaps(value);
    }

    @Override
    public List<Target> supportedTargets() {
        return Arrays.asList(Target.C, Target.CCPP, Target.Python);
    }

    @Override
    public Element export() {
        return ASTUtils.toElement(this.value);
    }
}
