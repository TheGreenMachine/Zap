package com.team1816.lib.loops;

import com.team1816.season.Constants;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import java.util.ArrayList;
import java.util.List;

/**
 * This code runs all the robot's loops. Loop objects are stored in a List object. They are started when the robot
 * powers up and stopped after the match.
 */
public class Looper implements ILooper {

    private boolean mRunning;
    private final List<Loop> mLoops;
    private final Object mTaskRunningLock = new Object();
    private double mTimestamp = 0;
    private double mDT = 0;
    private double mStart = 0;

    public Looper(TimedRobot robot) {
        Runnable runnable_ = new Runnable() {
            @Override
            public void run() {
                synchronized (mTaskRunningLock) {
                    if (mRunning) {
                        double now = Timer.getFPGATimestamp();

                        for (Loop loop : mLoops) {
                            loop.onLoop(now);
                        }

                        mDT = now - mTimestamp;
                        mTimestamp = now;
                    }
                }
            }
        };
        // add callback relative to robot loop time
        robot.addPeriodic(runnable_, Constants.kLooperDt, 0.005); // robot.getPeriod() / 2
        mRunning = false;
        mLoops = new ArrayList<>();
    }

    @Override
    public synchronized void register(Loop loop) {
        synchronized (mTaskRunningLock) {
            mLoops.add(loop);
        }
    }

    public synchronized void start() {
        synchronized (mTaskRunningLock) {
            if (!mRunning) {
                System.out.println("Starting loops");
                mTimestamp = Timer.getFPGATimestamp();
                mStart = mTimestamp;
                for (Loop loop : mLoops) {
                    loop.onStart(mTimestamp);
                }
                mRunning = true;
            }
        }
    }

    public synchronized void stop() {
        synchronized (mTaskRunningLock) {
            if (mRunning) {
                System.out.println("Stopping loops");
                mRunning = false;
                mTimestamp = Timer.getFPGATimestamp();
                for (Loop loop : mLoops) {
                    loop.onStop(mTimestamp);
                }
            }
        }
    }

    public double getLastLoop() {
        if (!mRunning) return 0;
        return mDT * 1000; // Convert to ms
    }
}
