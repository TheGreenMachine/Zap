package com.team1816.lib.vision;
import java.io.*;
import java.net.*;

public class VisionSocket {
    private final String PROTOCOL_LINE = "\\|";
    private Socket socket;
    private BufferedReader socketIn;
    private PrintWriter socketOut;
    private long needsReconnect = 0;
    private boolean enabled = false;
    private boolean useDebug = false;

    public void setDebug(boolean debug) {
        useDebug = debug;
    }

    public boolean connect() {
        if (!enabled) return false;
        try {
            socket = new Socket();
            socket.connect(new InetSocketAddress(InetAddress.getLocalHost(), 5802), 10);
            socketIn = new BufferedReader(new InputStreamReader(socket.getInputStream()));
            socketOut = new PrintWriter(socket.getOutputStream(), true);
        } catch (Throwable t) {
            debug("connect failed: " + t.getMessage());
            needsReconnect = System.currentTimeMillis();
            return false;
        }
        return true;
    }
    // todo make this configurable
    private void debug(String message) {
        if (!useDebug) return;
        System.out.println("CAMERA DEBUG: " + message);
    }
    public void close() {
        try {
            if (socket != null) socket.close();
            cleanup();
        } catch (IOException e) {
            debug("Close failed: " + e.getMessage());
           // e.printStackTrace();
           return;
        }
    }
    private void cleanup() {
        try {
            if (!socket.isClosed()) socket.close();
            socketIn.close();
            socketOut.close();
            socket = null;
            socketOut = null;
            socketIn = null;
        } catch (Throwable t) {
            return;
        }
    }
    public void setEnabled(boolean enabled) {
        this.enabled = enabled;
        debug("enabled: " + enabled);
        if (!enabled) {
            // close and cleanup
            close();
        } else {
            connect();
        }
    }
    public boolean isConnected() {
        if (!enabled || needsReconnect != 0) return false;
        boolean connected = socket != null && socket.isConnected();
        if (!connected) {
            cleanup();
        }
        return connected;
    }
    public String[] request(String message) {
        // we can safely return new String[0] because
        // all the code already checks for length > 1 as a safety measure
        // against, like, `distance|` being returned.
        if (!enabled || needsReconnect != 0) return new String[0];
        debug("enabled, sending request: " + message);
        try {
            socketOut.write(message + "\n");
            socketOut.flush();
            debug("Wrote line");
            String line = socketIn.readLine();
            debug("Read line: " + line);
            if (line == null) return new String[0];
            System.out.println("CAMERA LINE: " + line);
            return line.split(PROTOCOL_LINE);
        } catch (IOException e) {
            debug("Write failed: " + e.getMessage());
            needsReconnect = System.currentTimeMillis();
            return null;
        }
    }
    public boolean shouldReconnect() {
        if (!enabled) return false;
        if (needsReconnect == 0) return false;
        return (System.currentTimeMillis() - needsReconnect) >= 200;
    }
}
