package org.lflang.generator.c;

import org.lflang.federated.extensions.CGeneratorExtension;
import org.lflang.federated.FederateInstance;
import org.lflang.lf.Expression;
import org.lflang.lf.VarRef;
import org.lflang.lf.Action;
import org.lflang.lf.Port;

import java.util.regex.Pattern;

import org.lflang.ASTUtils;
import org.lflang.InferredType;
import org.lflang.TimeValue;
import org.lflang.TargetProperty.CoordinationType;
import org.lflang.federated.serialization.FedROS2CPPSerialization;
import org.lflang.federated.serialization.SupportedSerializers;
import org.lflang.generator.CodeBuilder;
import org.lflang.generator.GeneratorBase;
import org.lflang.generator.ReactionInstance;

/**
 * Generates C code to support messaging-related functionalities
 * in federated execution.
 *
 * @author {Edward A. Lee <eal@berkeley.edu>}
 * @author {Soroush Bateni <soroush@utdallas.edu>}
 * @author {Hou Seng Wong <housengw@berkeley.edu>}
 */
public class CNetworkGenerator {
}
