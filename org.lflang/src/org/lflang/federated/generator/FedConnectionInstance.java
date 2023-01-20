package org.lflang.federated.generator;

import org.lflang.federated.serialization.SupportedSerializers;
import org.lflang.generator.PortInstance;
import org.lflang.generator.RuntimeRange;
import org.lflang.generator.SendRange;
import org.lflang.lf.Connection;

/**
 * Class representing a federated connection.
 *
 * This is an undocumented class written by a previous contributor who is no longer active.
 * It merely serves as a record, presumably to make it easier to pass around information
 * around.
 *
 * @author Soroush Bateni
 */
public class FedConnectionInstance {

    SendRange srcRange;

    RuntimeRange<PortInstance> dstRange;

    int srcChannel;

    int srcBank;

    int dstChannel;

    int dstBank;

    FederateInstance srcFederate;

    FederateInstance dstFederate;

    SupportedSerializers serializer;

    public FedConnectionInstance(
        SendRange srcRange,
        RuntimeRange<PortInstance> dstRange,
        int srcChannel,
        int srcBank,
        int dstChannel,
        int dstBank,
        FederateInstance srcFederate,
        FederateInstance dstFederate,
        SupportedSerializers serializer
    ) {
        this.srcRange = srcRange;
        this.srcChannel = srcChannel;
        this.srcBank = srcBank;
        this.dstChannel = dstChannel;
        this.dstBank = dstBank;
        this.srcFederate = srcFederate;
        this.dstFederate = dstFederate;
        this.dstRange = dstRange;
        this.serializer = serializer;

        this.srcFederate.connections.add(this);
        this.dstFederate.connections.add(this);
    }

    public SendRange getSrcRange() {
        return srcRange;
    }

    public RuntimeRange<PortInstance> getDstRange() {
        return dstRange;
    }

    public int getSrcChannel() {
        return srcChannel;
    }

    public int getSrcBank() {
        return srcBank;
    }

    public int getDstChannel() {
        return dstChannel;
    }

    public int getDstBank() {
        return dstBank;
    }

    public FederateInstance getSrcFederate() {
        return srcFederate;
    }

    public FederateInstance getDstFederate() {
        return dstFederate;
    }

    public SupportedSerializers getSerializer() {
        return serializer;
    }

    public Connection getDefinition() {
        return srcRange.connection;
    }

    public PortInstance getSourcePortInstance() {
        return srcRange.instance;
    }

    public PortInstance getDestinationPortInstance() {
        return dstRange.instance;
    }
}
