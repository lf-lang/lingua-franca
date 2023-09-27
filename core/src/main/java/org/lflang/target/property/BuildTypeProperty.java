package org.lflang.target.property;

import java.util.Arrays;
import java.util.List;
import java.util.Properties;

import org.lflang.Target;
import org.lflang.TargetProperty;
import org.lflang.TargetPropertyConfig;
import org.lflang.ast.ASTUtils;
import org.lflang.lf.Element;
import org.lflang.target.property.BuildConfig.BuildType;
import org.lflang.target.property.type.UnionType;

public class BuildTypeProperty extends TargetPropertyConfig<BuildType> {

    public BuildTypeProperty() {
        super(UnionType.BUILD_TYPE_UNION);
    }

    @Override
    public Element export() {
        return ASTUtils.toElement(this.value.toString());
    }

    @Override
    public BuildType initialize() {
        return BuildType.RELEASE;
    }

    @Override
    public BuildType parse(Element value) {
        return (BuildType) UnionType.BUILD_TYPE_UNION.forName(ASTUtils.elementToSingleString(value));
    }

    @Override
    public List<Target> supportedTargets() {
        return Arrays.asList(Target.C, Target.CCPP, Target.CPP, Target.Rust);
    }

    @Override
    public void update(Properties cliArgs) {
        super.update(cliArgs);
        var key = TargetProperty.BUILD_TYPE.toString();
        if (cliArgs.containsKey(key)) {
            this.value =
                (BuildType) UnionType.BUILD_TYPE_UNION.forName(cliArgs.getProperty(key));
        }
    }
}
