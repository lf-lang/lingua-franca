package org.lflang.generator.c;


import java.util.HashMap;
import java.util.Map;
import java.util.stream.Collectors;

import org.lflang.ASTUtils;
import org.lflang.generator.CodeBuilder;
import org.lflang.lf.Instantiation;
import org.lflang.lf.Reactor;
import org.lflang.lf.Type;

public record TypeParameterizedReactor(Reactor r, Map<String, Type> typeArgs) {

    public TypeParameterizedReactor(Instantiation i) {
        this(ASTUtils.toDefinition(i.getReactorClass()), addTypeArgs(i, ASTUtils.toDefinition(i.getReactorClass())));
    }


    private static Map<String, Type> addTypeArgs(Instantiation instantiation, Reactor r) {
        HashMap<String, Type> ret = new HashMap<>();
        if (instantiation.getTypeArgs() != null) {
            for (int i = 0; i < r.getTypeParms().size(); i++) {
                ret.put(r.getTypeParms().get(i).getLiteral(), instantiation.getTypeArgs().get(i));
            }
        }
        return ret;
    }

    public String getName() {
        // FIXME: Types that are not just a single token need to be escaped or hashed
        return r.getName() + typeArgs.values().stream().map(ASTUtils::toOriginalText).collect(Collectors.joining("_"));
    }

    public void doDefines(CodeBuilder b) {
        typeArgs.forEach((literal, concreteType) -> b.pr(
            "#if defined " + literal + "\n" +
                "#undef " + literal + "\n" +
                "#endif // " + literal + "\n" +
                "#define " + literal + " " + ASTUtils.toOriginalText(concreteType)));
    }

    public Type resolveType(Type t) {
        if (t.getCode() == null) return t;
        var arg = typeArgs.get(t.getCode().getBody());
        if (arg != null) return arg;
        return t;
    }

    @Override
    public int hashCode() {
        return Math.abs(r.hashCode() * 31 + typeArgs.hashCode());
    }
}
