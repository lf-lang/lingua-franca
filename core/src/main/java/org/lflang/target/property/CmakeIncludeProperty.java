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
    public List<String> initialValue() {
        return new ArrayList<>();
    }


    @Override
    public void set(Element value, MessageReporter err) {
        if (!this.isSet) {
            super.set(value, err);
        } else {
            // NOTE: This merging of lists is potentially dangerous since
            // the incoming list of cmake-includes can belong to a .lf file that is
            // located in a different location, and keeping just filename
            // strings like this without absolute paths is incorrect.
            this.value.addAll(ASTUtils.elementToListOfStrings(value));
        }

    }

    @Override
    protected List<String> fromAst(Element value, MessageReporter err) {
        return ASTUtils.elementToListOfStrings(value);
    }

    @Override
    protected List<String> fromString(String value, MessageReporter err) {
        return null; // FIXME: not sure about this one
    }

    @Override
    public List<Target> supportedTargets() {
        return Arrays.asList(Target.CPP, Target.C, Target.CCPP);
    }

    @Override
    public Element toAstElement() {
        return ASTUtils.toElement(this.value);
    }
}
