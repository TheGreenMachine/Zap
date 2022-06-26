package com.team1816.season.motion.profiles;

public class SinusodalMotionProfile {

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
            maxJerk = 0;
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
        public Phase(double d) {
            duration = d;
        }
    }

    private Phase[] p = new Phase[3]; // sinusodal acceleration, flat, deceleration
    private Constraints constraints;
    private State initial;
    private State target;
    private double duration;

    public SinusodalMotionProfile() {
        for (int i = 0; i < p.length; i++) {
            p[i] = new Phase();
            p[i].duration = 0;
        }
        duration = 0;
        new SinusodalMotionProfile(new Constraints(), new State(), new State());
    }
    public SinusodalMotionProfile(Constraints c, State i, State t) {
        constraints = c;
        initial = i;
        target = t;

        double dX = t.position - i.position; // need to deal with dX sign
        double cx = 0;

        double t1 = Math.PI/2*(c.maxVel-i.velocity)/c.maxAccel;
        double t2 = 0;
        double t3 = Math.PI/2*(c.maxVel-t.velocity)/c.maxAccel;

        // cx calculations
        cx+=Math.pow((c.maxVel-i.velocity),2)/c.maxAccel*Math.PI/4;
        cx+=Math.pow((c.maxVel-t.velocity),2)/c.maxAccel*Math.PI/4;
        cx+=initial.velocity*t1;
        cx+= target.velocity*t3;

        t2+=(Math.abs(dX)-cx)/c.maxVel;

        p[0].duration = t1;
        p[1].duration = t2;
        p[2].duration = t3;

        duration = 0;
        for (Phase ph : p) {
            duration += Math.max(ph.duration, 0);
        }
    }
    public double getPosition(double t) {
        double cx = 0;
        double tmp = p[1].duration;
        if(t<tmp) {
            return 0;
        }
        return 0;
    }

    public double getVelocity(double t) {
        return 0;
    }

    public double getAcceleration(double t) {
        return 0;
    }

    public double getJerk(double t) {
        return 0;
    }

    public double getDuration() {
        return duration;
    }

    public boolean isFinished(double t) {
        return t >= duration;
    }
}
