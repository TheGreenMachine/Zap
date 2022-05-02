package com.team1816.lib.vision;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import java.io.*;
import java.net.*;
import javax.inject.Singleton;

@Singleton
public class VisionSocket {

    private final String PROTOCOL_LINE = "\\|";
    private Socket socket;
    private BufferedReader socketIn;
    private PrintWriter socketOut;
    private static double needsReconnect = 0;
    private boolean enabled = false;
    private boolean useDebug = false;

    public void setDebug(boolean debug) {
        useDebug = debug;
    }

    public boolean connect() {
        if (!enabled) return false;
        try {
            socket = new Socket();
            socket.setSoTimeout(15);
            socket.connect(new InetSocketAddress("10.18.16.16", 5802), 15);
            socketIn = new BufferedReader(new InputStreamReader(socket.getInputStream()));
            socketOut = new PrintWriter(socket.getOutputStream(), true);
            debug("connect succeeded");
            needsReconnect = 0;
        } catch (Throwable t) {
            DriverStation.reportError("connect failed: " + t.getMessage(), false);
            needsReconnect = Timer.getFPGATimestamp();
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
            if (socket != null) {
                socket.close();
            }
            cleanup();
        } catch (IOException e) {
            DriverStation.reportError("Close failed: " + e.getMessage(), false);
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
            DriverStation.reportError("Cleanup failed: " + t.getMessage(), false);
        }
    }

    public void setEnabled(boolean enabled) {
        if (this.enabled != enabled) {
            this.enabled = enabled;
            debug("enabled: " + enabled);
            if (!enabled) {
                // close and cleanup
                close();
            } else {
                connect();
            }
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
        if (!enabled || !isConnected()) {
            return null;
        }
        debug("enabled, sending request: " + message);
        try {
            socketOut.write(message + "\n");
            socketOut.flush();
            debug("Wrote line");
            String line = socketIn.readLine();
            debug("Read line: " + line);
            if (line == null) return new String[0];
            debug("CAMERA LINE: " + line);
            return line.split(PROTOCOL_LINE);
        } catch (IOException e) {
            debug("Write failed: " + e.getMessage() + " " + needsReconnect);
            if (needsReconnect == 0) needsReconnect = Timer.getFPGATimestamp();
            return null;
        }
    }

    public boolean shouldReconnect() {
        if (!enabled) return false;
        if (needsReconnect == 0) return false;
        return (System.currentTimeMillis() - needsReconnect) >= 200;
    }
}
