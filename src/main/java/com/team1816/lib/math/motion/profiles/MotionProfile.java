package com.team1816.lib.math.motion.profiles;

public abstract class MotionProfile {

    /**
     * Internal Definitions
     */

    public static class Constraints {

        public double maxVel, maxAccel, maxJerk;

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

        public double getMaxVel() {
            return maxVel;
        }

        public double getMaxAccel() {
            return maxAccel;
        }

        public double getMaxJerk() {
            return maxJerk;
        }
    }

    public static class State {

        public double position, velocity;

        public State() {
            position = 0;
            velocity = 0;
        }

        public State(double p) {
            position = p;
            velocity = 0;
        }

        public State(double p, double v) {
            position = p;
            velocity = v;
        }
    }

    public static class Phase {

        public double duration;

        public Phase() {
            duration = 0;
        }
    }

    private Constraints constraints;
    private State initial;
    private State target;

    public MotionProfile() {
        constraints = new Constraints();
        initial = new State();
        target = new State();
    }

    public MotionProfile(Constraints c, State i, State t) {
        constraints = c;
        initial = i;
        target = t;
    }

    public double getPosition() {
        return 0;
    }

    public double getVelocity() {
        return 0;
    }

    public double getAcceleration() {
        return 0;
    }

    public double getJerk() {
        return 0;
    }
}
