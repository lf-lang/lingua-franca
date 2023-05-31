package org.lflang.generator.c;

import java.util.ArrayList;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;

import org.lflang.ast.ASTUtils;
import org.lflang.ast.AstTransformation;
import org.lflang.lf.Connection;
import org.lflang.lf.LfFactory;
import org.lflang.lf.Reactor;
import org.lflang.lf.ReactorDecl;

public class CEnclavedReactorTransformation implements AstTransformation {

    public static final LfFactory factory = ASTUtils.factory;

    Map<ReactorDecl, ReactorDecl> enclaveToWrapperMap = new LinkedHashMap<>();
    public void applyTransformation(List<Reactor> reactors) {
        // This function performs the whole AST transformation consisting in
        // 1. Get all Enclave Reactors
        List<Reactor> enclaves = getEnclaves(reactors);

        // 2. create ReactorWrappers for all of them.
        createEnclaveWrappers(enclaves);

        // 2. Replace enclave Reactors with enclave Wrappers. Make new instances and re-wire the connections
        List<Reactor> wrappers = replaceEnclavesWithWrappers(enclaves);

        // 3. Create ConnectionReactors for connections between enclave and parent
        insertConnectionReactorsForEnclaveOutputs(reactors);

        setEnclaveWrapperParams(reactors);

        setFreeConnectionReactorParams(reactors);
    }
    private List<Reactor> getEnclaves(List<Reactor> reactors) {
        List<Reactor> enclaves = new ArrayList<>();
        return enclaves;
    }

    // Side-effect: Fills the enclaveToWrapperMap with the newly created EnclaveWrappers
    private void createEnclaveWrappers(List<Reactor> enclaves) {

    }
    // Side effect: We overwrite the Reactor-list passed into the AST. We remove and add Reactors as
    // children into the hierarchy. This function should also remove all old connections between reactors
    // and original enclaves with new instances.
    private List<Reactor> replaceEnclavesWithWrappers(List<Reactor> enclaves) {
        List<Reactor> enclaveWrappers= new ArrayList<>();
        return enclaveWrappers;
    }

    // In the case where the output port of an enclave is not connected to the top-level port of another enclave.
    // E.g. a contained enclave is connected to the parents output. Or it is connected to another childs input.
    // Or it is directly connected to a reaction in the parent (this requires some more thought)
    // In this case we must generate a ConnectionReactor and put it into the receiving environment. Which is the parent
    // in this case.

    private void insertConnectionReactorsForEnclaveOutputs(List<Reactor> reactors) {

    }

    // This sets the in_delay and is_physical parameters for all the enclaveWrappers.
    // It is based on the class and any delay on the connections to its ports.
    // The connections should be replaced with ordinary connections
    private void setEnclaveWrapperParams(List<Reactor> reactors) {

    }

    // This function finds all the free ConnectionReactors and sets their parameters.
    private void setFreeConnectionReactorParams(List<Reactor> reactors) {

    }
}
