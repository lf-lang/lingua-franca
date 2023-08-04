package org.lflang.analyses.scheduler;

import java.io.File;
import java.io.FileOutputStream;
import java.io.IOException;
import java.io.OutputStream;
import java.nio.file.Files;
import java.nio.file.Path;
import java.util.List;
import javax.xml.XMLConstants;
import javax.xml.parsers.DocumentBuilder;
import javax.xml.parsers.DocumentBuilderFactory;
import javax.xml.parsers.ParserConfigurationException;
import javax.xml.transform.OutputKeys;
import javax.xml.transform.Transformer;
import javax.xml.transform.TransformerException;
import javax.xml.transform.TransformerFactory;
import javax.xml.transform.dom.DOMSource;
import javax.xml.transform.stream.StreamResult;
import javax.xml.transform.stream.StreamSource;
import javax.xml.validation.Schema;
import javax.xml.validation.SchemaFactory;
import javax.xml.validation.Validator;
import org.lflang.analyses.dag.Dag;
import org.lflang.analyses.dag.DagEdge;
import org.lflang.generator.c.CFileConfig;
import org.lflang.util.FileUtil;
import org.w3c.dom.Comment;
import org.w3c.dom.Document;
import org.w3c.dom.Element;
import org.xml.sax.SAXException;

/**
 * An external static scheduler using the `mocasin` tool
 *
 * @author Shaokai Lin
 */
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
  public String generateSDF3XML(Dag dagSdf)
      throws ParserConfigurationException, TransformerException {
    DocumentBuilderFactory docFactory = DocumentBuilderFactory.newInstance();
    DocumentBuilder docBuilder = docFactory.newDocumentBuilder();

    // root elements: sdf3
    Document doc = docBuilder.newDocument();
    Element rootElement = doc.createElement("sdf3");
    rootElement.setAttribute("version", "1.0");
    rootElement.setAttribute("type", "sdf");
    doc.appendChild(rootElement);

    // applicationGraph
    Element appGraph = doc.createElement("applicationGraph");
    appGraph.setAttribute("name", "lf");
    rootElement.appendChild(appGraph);

    // sdf
    Element sdf = doc.createElement("sdf");
    sdf.setAttribute("name", "g");
    sdf.setAttribute("type", "G");
    appGraph.appendChild(sdf);

    // Append reaction nodes under the SDF element.
    for (var node : dagSdf.dagNodes) {
      // Each SDF "actor" here is actually a reaction node.
      Comment comment = doc.createComment("This actor is: " + node.toString());
      Element actor = doc.createElement("actor");
      actor.setAttribute("name", node.toString());
      sdf.appendChild(comment);
      sdf.appendChild(actor);

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
    List<DagEdge> edges = dagSdf.getDagEdges();
    for (int i = 0; i < edges.size(); i++) {
      DagEdge edge = edges.get(i);
      Element channel = doc.createElement("channel");
      channel.setAttribute("name", "ch" + i);
      channel.setAttribute("srcActor", edge.sourceNode.toString());
      channel.setAttribute("srcPort", edge.toString() + "_output");
      channel.setAttribute("dstActor", edge.sinkNode.toString());
      channel.setAttribute("dstPort", edge.toString() + "_input");

      // If the edge is the added back edge from tail to head,
      // add an initial token.
      if (edge.sourceNode == dagSdf.tail && edge.sinkNode == dagSdf.head) {
        channel.setAttribute("initialTokens", "1");
      }

      sdf.appendChild(channel);
    }

    // sdfProperties
    Element sdfProperties = doc.createElement("sdfProperties");
    appGraph.appendChild(sdfProperties);

    // Generate actorProperties (i.e., execution times)
    for (var node : dagSdf.dagNodes) {
      // actorProperties
      Element actorProperties = doc.createElement("actorProperties");
      actorProperties.setAttribute("actor", node.toString());

      // processor
      Element processor = doc.createElement("processor");
      processor.setAttribute("type", "proc_0");
      processor.setAttribute("default", "true");

      // executionTime
      Element executionTime = doc.createElement("executionTime");
      if (node.isAuxiliary()) executionTime.setAttribute("time", "0");
      else
        executionTime.setAttribute(
            "time", ((Long) node.getReaction().wcet.toNanoSeconds()).toString());

      // memory
      Element memory = doc.createElement("memory");

      // stateSize
      Element stateSize = doc.createElement("stateSize");
      stateSize.setAttribute("max", "1"); // FIXME: What does this do? This is currently hardcoded.

      // Append elements.
      memory.appendChild(stateSize);
      processor.appendChild(executionTime);
      processor.appendChild(memory);
      actorProperties.appendChild(processor);
      sdfProperties.appendChild(actorProperties);
    }

    // Generate channelProperties
    // FIXME: All values here are hardcoded. Make sure they make sense.
    for (int i = 0; i < edges.size(); i++) {
      Element channelProperties = doc.createElement("channelProperties");
      channelProperties.setAttribute("channel", "ch" + i);

      // bufferSize
      Element bufferSize = doc.createElement("bufferSize");
      bufferSize.setAttribute("sz", "1");
      bufferSize.setAttribute("src", "1");
      bufferSize.setAttribute("dst", "1");
      bufferSize.setAttribute("mem", "1");

      // tokenSize
      Element tokenSize = doc.createElement("tokenSize");
      tokenSize.setAttribute("sz", "1");

      // bandwidth
      Element bandwidth = doc.createElement("bandwidth");
      bandwidth.setAttribute("min", "1");

      // latency
      Element latency = doc.createElement("latency");
      latency.setAttribute("min", "0");

      channelProperties.appendChild(bufferSize);
      channelProperties.appendChild(tokenSize);
      channelProperties.appendChild(bandwidth);
      channelProperties.appendChild(latency);
      sdfProperties.appendChild(channelProperties);
    }

    // Write dom document to a file.
    String path = this.mocasinDir.toString() + "/sdf.xml";
    try (FileOutputStream output = new FileOutputStream(path)) {
      writeXml(doc, output);
    } catch (IOException e) {
      throw new RuntimeException(e);
    }

    return path;
  }

  /** Write XML doc to output stream */
  private static void writeXml(Document doc, OutputStream output) throws TransformerException {

    TransformerFactory transformerFactory = TransformerFactory.newInstance();
    Transformer transformer = transformerFactory.newTransformer();
    transformer.setOutputProperty(OutputKeys.INDENT, "yes");
    transformer.setOutputProperty(
        "{http://xml.apache.org/xslt}indent-amount", "2"); // Indent by 2 spaces
    DOMSource source = new DOMSource(doc);
    StreamResult result = new StreamResult(output);

    transformer.transform(source, result);
  }

  /** Check whether an XML file is valid wrt a schema file (XSD) */
  public static boolean validateXMLSchema(String xsdPath, String xmlPath) {
    try {
      SchemaFactory factory = SchemaFactory.newInstance(XMLConstants.W3C_XML_SCHEMA_NS_URI);
      Schema schema = factory.newSchema(new File(xsdPath));
      Validator validator = schema.newValidator();
      validator.validate(new StreamSource(new File(xmlPath)));
    } catch (IOException e) {
      System.out.println("Exception: " + e.getMessage());
      return false;
    } catch (SAXException e1) {
      System.out.println("SAX Exception: " + e1.getMessage());
      return false;
    }
    return true;
  }

  /** Main function for assigning nodes to workers */
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
    String xmlPath = "";
    try {
      xmlPath = generateSDF3XML(dagSdf);
    } catch (Exception e) {
      throw new RuntimeException(e);
    }
    assert !xmlPath.equals("") : "XML path is empty.";

    // Validate the generated XML.
    try {
      FileUtil.copyFromClassPath("/staticScheduler/mocasin/sdf3-sdf.xsd", mocasinDir, false, false);
    } catch (IOException e) {
      throw new RuntimeException(e);
    }
    String xsdPath = mocasinDir.resolve("sdf3-sdf.xsd").toString();
    if (!validateXMLSchema(xsdPath, xmlPath)) {
      throw new RuntimeException("The generated SDF3 XML is invalid.");
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
