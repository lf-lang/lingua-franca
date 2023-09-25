package org.lflang.target.property.type;

import java.util.ArrayList;
import java.util.List;

import org.lflang.MessageReporter;
import org.lflang.TargetPropertyConfig;
import org.lflang.ast.ASTUtils;
import org.lflang.lf.Element;

public class CmakeIncludeConfig extends TargetPropertyConfig<List<String>> {

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
    public Element export() {
        return ASTUtils.toElement(this.value);
    }
}
