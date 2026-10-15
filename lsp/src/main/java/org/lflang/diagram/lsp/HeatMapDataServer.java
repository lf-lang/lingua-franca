package org.lflang.diagram.lsp;

import com.google.gson.JsonObject;
import com.google.gson.JsonParser;
import java.io.BufferedReader;
import java.io.IOException;
import java.io.InputStreamReader;
import java.net.ServerSocket;
import java.net.Socket;
import java.nio.charset.StandardCharsets;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.concurrent.CopyOnWriteArrayList;
import org.lflang.diagram.synthesis.util.HeatMapDataProvider;
import org.lflang.diagram.synthesis.util.HeatMapDataProvider.ReactionExecutionData;

/**
 * A lightweight TCP server that receives execution trace data from a running Lingua Franca program
 * and forwards it to the {@link HeatMapDataProvider} singleton.
 *
 * <p>The server listens on a configurable port (default 15045) for newline-delimited JSON messages.
 * Supported message types:
 *
 * <ul>
 *   <li>{@code {"type":"register","program":"<name>"}} - registers a program (currently a no-op)
 *   <li>{@code {"type":"execution_data","reactions":[...]}} - updates reaction execution statistics
 *   <li>{@code {"type":"shutdown"}} - signals that the connected program has stopped
 * </ul>
 *
 * <p>Listeners are notified at most once every 500 ms to avoid flooding the diagram renderer with
 * updates.
 */
public class HeatMapDataServer {

  /** Callback interface for heat map data changes. */
  @FunctionalInterface
  public interface HeatMapUpdateListener {
    void onHeatMapUpdate();
  }

  private static final int DEFAULT_PORT = 15045;
  private static final long THROTTLE_MS = 500;

  private final int port;
  private ServerSocket serverSocket;
  private Thread acceptThread;
  private volatile boolean running;

  private final List<HeatMapUpdateListener> listeners = new CopyOnWriteArrayList<>();
  private long lastNotifyTime = 0;

  public HeatMapDataServer() {
    this(DEFAULT_PORT);
  }

  public HeatMapDataServer(int port) {
    this.port = port;
  }

  /** Start listening for connections on a daemon thread. */
  public void start() throws IOException {
    serverSocket = new ServerSocket(port);
    running = true;
    acceptThread = new Thread(this::acceptLoop, "HeatMapDataServer-accept");
    acceptThread.setDaemon(true);
    acceptThread.start();
  }

  /** Stop the server and release resources. */
  public void stop() {
    running = false;
    try {
      if (serverSocket != null && !serverSocket.isClosed()) {
        serverSocket.close();
      }
    } catch (IOException ignored) {
      // Best-effort shutdown.
    }
  }

  /** Returns the port this server is listening on. */
  public int getPort() {
    return port;
  }

  /** Register a listener that will be called (throttled) when heat map data changes. */
  public void addListener(HeatMapUpdateListener listener) {
    listeners.add(listener);
  }

  // -------------------------------------------------------------------------

  private void acceptLoop() {
    while (running) {
      try {
        Socket clientSocket = serverSocket.accept();
        Thread clientThread =
            new Thread(
                () -> handleClient(clientSocket),
                "HeatMapDataServer-client-" + clientSocket.getRemoteSocketAddress());
        clientThread.setDaemon(true);
        clientThread.start();
      } catch (IOException e) {
        if (running) {
          System.err.println("HeatMapDataServer accept error: " + e.getMessage());
        }
      }
    }
  }

  private void handleClient(Socket socket) {
    try (BufferedReader reader =
        new BufferedReader(new InputStreamReader(socket.getInputStream(), StandardCharsets.UTF_8))) {
      String line;
      while ((line = reader.readLine()) != null) {
        line = line.trim();
        if (line.isEmpty()) continue;
        try {
          processMessage(line);
        } catch (Exception e) {
          System.err.println("HeatMapDataServer message error: " + e.getMessage());
        }
      }
    } catch (IOException e) {
      if (running) {
        System.err.println("HeatMapDataServer client error: " + e.getMessage());
      }
    } finally {
      try {
        socket.close();
      } catch (IOException ignored) {
        // Best-effort close.
      }
    }
  }

  private void processMessage(String json) {
    JsonObject msg = JsonParser.parseString(json).getAsJsonObject();
    String type = msg.get("type").getAsString();

    switch (type) {
      case "register":
        // Currently a no-op; could be used for multi-program tracking later.
        break;

      case "execution_data":
        Map<String, ReactionExecutionData> newData = new HashMap<>();
        for (var element : msg.getAsJsonArray("reactions")) {
          JsonObject r = element.getAsJsonObject();
          String reactor = r.get("reactor").getAsString();
          int index = r.get("index").getAsInt();
          long avgNs = r.get("avg_ns").getAsLong();
          long maxNs = r.get("max_ns").getAsLong();
          long count = r.get("count").getAsLong();
          String key = HeatMapDataProvider.key(reactor, index);
          newData.put(key, new ReactionExecutionData(reactor, index, avgNs, maxNs, count));
        }
        HeatMapDataProvider.getInstance().updateData(newData);
        throttledNotify();
        break;

      case "shutdown":
        HeatMapDataProvider.getInstance().clear();
        throttledNotify();
        break;

      default:
        System.err.println("HeatMapDataServer: unknown message type '" + type + "'");
    }
  }

  private void throttledNotify() {
    long now = System.currentTimeMillis();
    if (now - lastNotifyTime < THROTTLE_MS) {
      return;
    }
    lastNotifyTime = now;
    for (HeatMapUpdateListener listener : listeners) {
      try {
        listener.onHeatMapUpdate();
      } catch (Exception e) {
        System.err.println("HeatMapDataServer listener error: " + e.getMessage());
      }
    }
  }
}
