package org.lflang.generator.c;


import java.util.HashMap;
import java.util.Map;
import java.util.stream.Collectors;

import org.lflang.ASTUtils;
import org.lflang.lf.Instantiation;
import org.lflang.lf.Reactor;
import org.lflang.lf.Type;

import com.google.common.collect.ImmutableMap;

public record TypeParameterizedReactor(Reactor r, Map<String, Type> typeArgs) {

    public TypeParameterizedReactor(Instantiation i) {
        this(ASTUtils.toDefinition(i.getReactorClass()), addTypeArgs(i, ASTUtils.toDefinition(i.getReactorClass())));
    }

    private static ImmutableMap<String, Type> addTypeArgs(Instantiation instantiation, Reactor r) {
        HashMap<String, Type> ret = new HashMap<>();
        if (instantiation.getTypeArgs() != null) {
            for (int i = 0; i < r.getTypeParms().size(); i++) {
                ret.put(r.getTypeParms().get(i).getLiteral(), instantiation.getTypeArgs().get(i));
            }
        }
        return ImmutableMap.copyOf(ret);
    }

    public String getName() {
        // FIXME: Types that are not just a single token need to be escaped or hashed
        return r.getName() + typeArgs.values().stream().map(ASTUtils::toOriginalText).collect(Collectors.joining("_"));
    }

    @Override
    public int hashCode() {
        return Math.abs(r.hashCode() * 31 + typeArgs.hashCode());
    }
}
