package org.lflang.target.property;


import java.util.ArrayList;
import java.util.List;

import org.lflang.MessageReporter;
import org.lflang.TargetPropertyConfig;
import org.lflang.ast.ASTUtils;
import org.lflang.lf.Element;
import org.lflang.target.property.type.UnionType;

/**
 * Note: {@code set} implements an "append" semantics.
 */
public abstract class DefaultStringListProperty extends TargetPropertyConfig<List<String>> {

    public DefaultStringListProperty() {
        super(UnionType.STRING_OR_STRING_ARRAY);
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
            this.value.addAll(this.fromAst(value, err));
        }
    }

    @Override
    public void set(String string, MessageReporter err) {
        if (!this.isSet) {
            super.set(string, err);
        } else {
            this.value.addAll(this.fromString(string, err));
        }
    }

    @Override
    public List<String> fromAst(Element value, MessageReporter err) {
        return ASTUtils.elementToListOfStrings(value);
    }

    @Override
    protected List<String> fromString(String value, MessageReporter err) {
        return List.of(value.split(" "));
    }


    @Override
    public Element toAstElement() {
        return ASTUtils.toElement(value);
    }

}
