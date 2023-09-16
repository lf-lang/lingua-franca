package org.lflang.target.property;

import java.util.Properties;

import org.lflang.TargetProperty;
import org.lflang.TargetPropertyConfig;
import org.lflang.ast.ASTUtils;
import org.lflang.lf.Element;
import org.lflang.target.property.BuildConfig.BuildType;
import org.lflang.target.property.type.UnionType;

public class BuildTypeConfig extends TargetPropertyConfig<BuildType> {

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
    public void update(Properties cliArgs) {
        super.update(cliArgs);
        var key = TargetProperty.BUILD_TYPE.toString();
        if (cliArgs.containsKey(key)) {
            this.value =
                (BuildType) UnionType.BUILD_TYPE_UNION.forName(cliArgs.getProperty(key));
        }
    }
}
