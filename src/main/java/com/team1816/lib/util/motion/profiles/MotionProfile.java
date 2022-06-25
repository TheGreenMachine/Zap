package com.team1816.lib.util.motion.profiles;

/**
 * This class will construct a jerk limited motion profile, by which it will take in parameters of maximum rate of change
 * of position, maximum rate of change of velocity, and maximum rate of change of acceleration and then construct the
 * appropriate motion profile. Note, that this is not the same as a trapezoidal motion profile but one degree higher.
 */
public class MotionProfile {
    /**
     * Internal Definitions
     */
    public static class Constraints {
        private double maxVel, maxAccel, maxJerk;
        public Constraints() {
            maxVel = 0;
            maxAccel = 0;
            maxJerk = 0;
        }

        public Constraints(double mv, double ma, double mj) {
            maxVel = mv;
            maxAccel = ma;
            maxJerk = mj;
        }
    }
    public static class State {
        public double position, velocity;
        public State(){
            position = 0;
            velocity = 0;
        }
        public State(double p, double v) {
            position = p;
            velocity = v;
        }
    }
    public static class Phase {
        public double duration;
        public Phase(){
            duration = 0;
        }
    }
    /**
     * Profile properties
     */

    // 7 profile phases
    private Phase p1; // positive jerk, increasing acceleration
    private Phase p2; // zero jerk, positive acceleration
    private Phase p3; // negative jerk, decreasing acceleration
    private Phase p4; // zero jerk, zero acceleration
    private Phase p5; // negative jerk, decreasing acceleration
    private Phase p6; // zero jerk, negative acceleration
    private Phase p7; // positive jerk, increasing acceleration

    // the duration of odd phases can be computed by abs(Δa/j) and the duration of the even phases can be computed by abs(Δv/a)

    public MotionProfile() {
        p1 = new Phase();
        p2 = new Phase();
        p3 = new Phase();
        p4 = new Phase();
        p5 = new Phase();
        p6 = new Phase();
        p7 = new Phase();
    }

    public MotionProfile(Constraints constraints, State initial, State target) {

    }

    public double getPosition(double t) {

    }

    public double getVelocity(double t) {

    }

    public double getAcceleration(double t) {

    }

    public double getJerk(double t) {

    }

}
