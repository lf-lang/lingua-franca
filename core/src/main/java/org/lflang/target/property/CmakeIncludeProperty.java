package org.lflang.target.property;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

import org.lflang.MessageReporter;
import org.lflang.Target;
import org.lflang.TargetPropertyConfig;
import org.lflang.ast.ASTUtils;
import org.lflang.lf.Element;
import org.lflang.target.property.type.UnionType;

public class CmakeIncludeProperty extends TargetPropertyConfig<List<String>> {

    public CmakeIncludeProperty() {
        super(UnionType.FILE_OR_FILE_ARRAY);
    }

    @Override
    public List<String> initialize() {
        return new ArrayList<>();
    }

    @Override
    public void update(Element value, MessageReporter err) {
        super.update(value, err);
        // FIXME: This merging of lists is potentially dangerous since
        // the incoming list of cmake-includes can belong to a .lf file that is
        // located in a different location, and keeping just filename
        // strings like this without absolute paths is incorrect.
        this.value.addAll(ASTUtils.elementToListOfStrings(value));
    }

    @Override
    protected List<String> parse(Element value) {
        return ASTUtils.elementToListOfStrings(value);
    }

    @Override
    public List<Target> supportedTargets() {
        return Arrays.asList(Target.CPP, Target.C, Target.CCPP);
    }

    @Override
    public Element export() {
        return ASTUtils.toElement(this.value);
    }
}
