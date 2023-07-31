package org.lflang.analyses.scheduler;

import java.io.FileOutputStream;
import java.io.IOException;
import java.io.OutputStream;
import java.nio.file.Files;
import java.nio.file.Path;
import javax.xml.parsers.DocumentBuilder;
import javax.xml.parsers.DocumentBuilderFactory;
import javax.xml.parsers.ParserConfigurationException;
import javax.xml.transform.Transformer;
import javax.xml.transform.TransformerException;
import javax.xml.transform.TransformerFactory;
import javax.xml.transform.dom.DOMSource;
import javax.xml.transform.stream.StreamResult;
import org.lflang.analyses.dag.Dag;
import org.lflang.analyses.dag.DagEdge;
import org.lflang.generator.c.CFileConfig;
import org.w3c.dom.Comment;
import org.w3c.dom.Document;
import org.w3c.dom.Element;

/** An external static scheduler using the `mocasin` tool */
public class MocasinScheduler implements StaticScheduler {

  /** File config */
  protected final CFileConfig fileConfig;

  /** Directory where graphs are stored */
  protected final Path graphDir;

  /** Directory where mocasin files are stored */
  protected final Path mocasinDir;

  /** Constructor */
  public MocasinScheduler(CFileConfig fileConfig) {
    this.fileConfig = fileConfig;
    this.graphDir = fileConfig.getSrcGenPath().resolve("graphs");
    this.mocasinDir = fileConfig.getSrcGenPath().resolve("mocasin");

    // Create the mocasin directory.
    try {
      Files.createDirectories(this.mocasinDir);
    } catch (IOException e) {
      throw new RuntimeException(e);
    }
  }

  /** Turn the original DAG into SDF format by adding an edge from tail to head. */
  public Dag turnDagIntoSdfFormat(Dag dagRaw) {
    // Create a copy of the original dag.
    Dag dag = new Dag(dagRaw);

    // Connect tail to head.
    dag.addEdge(dag.tail, dag.head);

    return dag;
  }

  /**
   * Generate an XML file that represents the DAG using the SDF3 format
   *
   * @throws ParserConfigurationException
   * @throws TransformerException
   */
  public void generateSDF3XML(Dag dagSdf)
      throws ParserConfigurationException, TransformerException {
    DocumentBuilderFactory docFactory = DocumentBuilderFactory.newInstance();
    DocumentBuilder docBuilder = docFactory.newDocumentBuilder();

    // root elements: sdf3
    Document doc = docBuilder.newDocument();
    Element rootElement = doc.createElement("sdf3");
    doc.appendChild(rootElement);

    // applicationGraph
    Element appGraph = doc.createElement("applicationGraph");
    rootElement.appendChild(appGraph);

    // sdf
    Element sdf = doc.createElement("sdf");
    sdf.setAttribute("name", "g"); // FIXME: Is this necessary?
    sdf.setAttribute("type", "G"); // FIXME: Is this necessary?
    appGraph.appendChild(sdf);

    // Append reaction nodes under the SDF element.
    for (var node : dagSdf.dagNodes) {
      // Each SDF "actor" here is actually a reaction node.
      Comment comment = doc.createComment("This actor is: " + node.toString());
      Element actor = doc.createElement("actor");
      actor.setAttribute("name", node.toString());
      appGraph.appendChild(comment);
      appGraph.appendChild(actor);

      // Incoming edges constitute input ports.
      var incomingEdges = dagSdf.dagEdgesRev.get(node);
      for (var srcNode : incomingEdges.keySet()) {
        Element inputPort = doc.createElement("port");
        inputPort.setAttribute("name", incomingEdges.get(srcNode).toString() + "_input");
        inputPort.setAttribute("type", "in");
        inputPort.setAttribute("rate", "1");

        actor.appendChild(inputPort);
      }

      // Outgoing edges constitute output ports.
      var outgoingEdges = dagSdf.dagEdges.get(node);
      for (var destNode : outgoingEdges.keySet()) {
        Element outputPort = doc.createElement("port");
        outputPort.setAttribute("name", outgoingEdges.get(destNode).toString() + "_output");
        outputPort.setAttribute("type", "out");
        outputPort.setAttribute("rate", "1");

        actor.appendChild(outputPort);
      }
    }

    // Generate channel fields.
    for (var srcNode : dagSdf.dagNodes) {
      for (var destNode : dagSdf.dagEdges.get(srcNode).keySet()) {
        DagEdge edge = dagSdf.dagEdges.get(srcNode).get(destNode);
        Element channel = doc.createElement("channel");
        channel.setAttribute("srcActor", srcNode.toString());
        channel.setAttribute("srcPort", edge.toString() + "_output");
        channel.setAttribute("dstActor", destNode.toString());
        channel.setAttribute("dstPort", edge.toString() + "_input");
        appGraph.appendChild(channel);
      }
    }

    // write dom document to a file
    String path = this.mocasinDir.toString() + "/sdf.xml";
    try (FileOutputStream output = new FileOutputStream(path)) {
      writeXml(doc, output);
    } catch (IOException e) {
      e.printStackTrace();
    }
  }

  /** Write XML doc to output stream */
  private static void writeXml(Document doc, OutputStream output) throws TransformerException {

    TransformerFactory transformerFactory = TransformerFactory.newInstance();
    Transformer transformer = transformerFactory.newTransformer();
    DOMSource source = new DOMSource(doc);
    StreamResult result = new StreamResult(output);

    transformer.transform(source, result);
  }

  public Dag partitionDag(Dag dagRaw, int numWorkers, String dotFilePostfix) {

    // Prune redundant edges.
    Dag dagPruned = StaticSchedulerUtils.removeRedundantEdges(dagRaw);

    // Generate a dot file.
    Path filePruned = graphDir.resolve("dag_pruned" + dotFilePostfix + ".dot");
    dagPruned.generateDotFile(filePruned);

    // Turn the DAG into the SDF3 format.
    Dag dagSdf = turnDagIntoSdfFormat(dagPruned);

    // Generate a dot file.
    Path fileSDF = graphDir.resolve("dag_sdf" + dotFilePostfix + ".dot");
    dagSdf.generateDotFile(fileSDF);

    // Write an XML file in SDF3 format.
    try {
      generateSDF3XML(dagSdf);
    } catch (Exception e) {
      throw new RuntimeException(e);
    }

    return dagSdf;
  }

  /**
   * If the number of workers is unspecified, determine a value for the number of workers. This
   * scheduler base class simply returns 1. An advanced scheduler is free to run advanced algorithms
   * here.
   */
  public int setNumberOfWorkers() {
    return 1;
  }
}
