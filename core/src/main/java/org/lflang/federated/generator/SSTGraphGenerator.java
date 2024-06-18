package org.lflang.federated.generator;

import com.google.gson.JsonArray;
import com.google.gson.JsonObject;

import java.util.List;

public class SSTGraphGenerator {
  public static JsonObject generateGraphFile(List<FederateInstance> federateInstances) {
    JsonObject graphObject = new JsonObject();

    // Auth list
    JsonArray authList = new JsonArray();
    authList.add(createAuthEntry(101, "localhost", "localhost", 21900, 21902, 21901, 21903, 1, false, true));
    authList.add(createAuthEntry(102, "localhost", "localhost", 21900, 21902, 21901, 21903, 1, false, true));
    graphObject.add("authList", authList);

    // Auth trusts
    JsonArray authTrusts = new JsonArray();
    JsonObject trustRelation = new JsonObject();
    trustRelation.addProperty("id1", 101);
    trustRelation.addProperty("id2", 102);
    authTrusts.add(trustRelation);
    graphObject.add("authTrusts", authTrusts);

    // Assignments section
    JsonObject assignments = new JsonObject();
    assignments.addProperty("net1.rti", 101);
    for (FederateInstance federate : federateInstances) {
      assignments.addProperty("net1." + federate.name, 101); // Assuming "101" is a placeholder
    }
    graphObject.add("assignments", assignments);

    // Entity list section
    JsonArray entityList = createEntityList(federateInstances);
    graphObject.add("entityList", entityList);

    // File sharing lists (empty for this example)
    graphObject.add("filesharingLists", new JsonArray());

    return graphObject;
  }

  private static JsonObject createAuthEntry(int id, String entityHost, String authHost, int tcpPort, int udpPort,
      int authPort, int callbackPort, int dbProtectionMethod,
      boolean backupEnabled, boolean contextualCallbackEnabled) {
    JsonObject authEntry = new JsonObject();
    authEntry.addProperty("id", id);
    authEntry.addProperty("entityHost", entityHost);
    authEntry.addProperty("authHost", authHost);
    authEntry.addProperty("tcpPort", tcpPort);
    authEntry.addProperty("udpPort", udpPort);
    authEntry.addProperty("authPort", authPort);
    authEntry.addProperty("callbackPort", callbackPort);
    authEntry.addProperty("dbProtectionMethod", dbProtectionMethod);
    authEntry.addProperty("backupEnabled", backupEnabled);
    authEntry.addProperty("contextualCallbackEnabled", contextualCallbackEnabled);
    return authEntry;
  }

  private static JsonArray createEntityList(List<FederateInstance> federateInstances) {
    JsonArray entityList = new JsonArray();

    // RTI entity
    JsonObject rti = createEntity("Servers", "net1.rti", "Net1.rti");
    // TODO: Make the two below work on future.
    rti.addProperty("port", 15045);
    rti.addProperty("host", "localhost");
    entityList.add(rti);

    // Federate entities
    for (FederateInstance federate : federateInstances) {
      String federateName = federate.name;
      JsonObject entity = createEntity("Clients", "net1." + federateName, "Net1." + federateName);
      entityList.add(entity);
    }

    return entityList;
  }

  private static JsonObject createEntity(String group, String name, String credentialPrefix) {
    JsonObject entity = new JsonObject();
    entity.addProperty("group", group);
    entity.addProperty("name", name);
    entity.addProperty("distProtocol", "TCP");
    entity.addProperty("usePermanentDistKey", false);
    entity.addProperty("distKeyValidityPeriod", "1*hour");
    entity.addProperty("maxSessionKeysPerRequest", 1);
    entity.addProperty("netName", "net1");
    entity.addProperty("credentialPrefix", credentialPrefix);
    entity.add("backupToAuthIds", new JsonArray()); // Empty array for backupToAuthIds
    return entity;
  }

}
