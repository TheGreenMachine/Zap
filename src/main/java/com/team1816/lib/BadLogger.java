package com.team1816.lib;

import badlog.lib.BadLog;
import com.team1816.lib.subsystems.SubsystemManager;
import com.team1816.season.Robot;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;

import java.nio.file.Files;
import java.nio.file.Path;
import java.text.SimpleDateFormat;
import java.util.Date;

public class BadLogger {

    private static BadLog logger;
    public static void setupLogs(TimedRobot tr){
        if(tr instanceof Robot){
            var logFile = new SimpleDateFormat("MMdd_HH-mm").format(new Date());
            var robotName = System.getenv("ROBOT_NAME");
            if (robotName == null) robotName = "default";
            var logFileDir = "/home/lvuser/";
            // if there is a USB drive use it
            if (Files.exists(Path.of("/media/sda1"))) {
                logFileDir = "/media/sda1/";
            }
            if (RobotBase.isSimulation()) {
                if (System.getProperty("os.name").toLowerCase().contains("win")) {
                    logFileDir = System.getenv("temp") + "\\";
                } else {
                    logFileDir = System.getProperty("user.dir") + "/";
                }
            }
            var filePath = logFileDir + robotName + "_" + logFile + ".bag";
            logger = BadLog.init(filePath);

            BadLog.createTopic(
                "Timings/Looper",
                "ms",
                ((Robot) tr)::getLastLooperLoop,
                "hide",
                "join:Timings"
            );
            BadLog.createTopic(
                "Timings/RobotLoop",
                "ms",
                ((Robot) tr)::getLastRobotLoop,
                "hide",
                "join:Timings"
            );
            BadLog.createTopic(
                "Timings/Timestamp",
                "s",
                Timer::getFPGATimestamp,
                "xaxis",
                "hide"
            );

            logger.finishInitialization();
        }
    }

    public static void update(){
        logger.updateTopics();
        logger.log();
    }
}
