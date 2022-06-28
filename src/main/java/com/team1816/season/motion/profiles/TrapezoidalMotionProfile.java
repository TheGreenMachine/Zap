package com.team1816.season.motion.profiles;

/**
 * This class will construct a standard trapezoidal motion profile.
 */
public class TrapezoidalMotionProfile {

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
    }

    private Phase[] p = new Phase[3];
    private Constraints constraints;
    private State initial;
    private State target;
    private double duration;

    public TrapezoidalMotionProfile() {
        for (int i = 0; i < p.length; i++) {
            p[i] = new Phase();
            p[i].duration = 0;
        }
        duration = 0;
        new TrapezoidalMotionProfile(new Constraints(), new State(), new State());
    }

    public TrapezoidalMotionProfile(Constraints c, State i, State t) {
        constraints = c;
        initial = i;
        target = t;
        double dX = t.position - i.position;
        double dV = t.velocity - i.velocity;
        double cv = c.maxVel;

        double t1 = 0;
        double t2 = 0;
        double t3 = 0;
        do {
            t1 = Math.abs((cv - i.velocity) / c.maxAccel);
            t3 = Math.abs((cv - t.velocity) / c.maxAccel);

            double edX =
                c.maxAccel *
                (t1 * t1 - t3 * t3) /
                2 +
                (c.maxAccel * t1 + i.velocity) *
                t3;
            t2 = (dX - edX) / (c.maxAccel * t1 + i.velocity);
            cv /= 1.5;
        } while (t2 >= 0);
        if (dX < 0) {
            c.maxAccel = Math.min(c.maxAccel, c.maxAccel*-1); // quick fix to get accurate position and velocity calculations
        }
        duration = 0;
        for (Phase ph : p) {
            duration += Math.max(ph.duration, 0);
        }
    }

    public double getPosition(double t) {
        double cx = initial.position;
        double tmp = p[1].duration;
        if (t <= tmp) {
            cx += getAcceleration(t) / 2 * Math.pow((tmp - t), 2);
            return cx;
        }
        cx += getAcceleration(tmp) / 2 * Math.pow(p[1].duration, 2);
        tmp += p[2].duration;
        if (t <= tmp) {
            cx += getVelocity(t) * (tmp - t);
            return cx;
        }
        cx += getVelocity(tmp) * (p[2].duration);
        tmp += p[3].duration;
        if (t <= tmp) {
            cx +=
                getVelocity(tmp - p[3].duration) *
                (tmp - t) +
                getAcceleration(t) /
                2 *
                Math.pow((tmp - t), 2);
            return cx;
        } else {
            return target.position;
        }
    }

    public double getVelocity(double t) {
        double cv = initial.velocity;
        double tmp = p[1].duration;
        if (t <= tmp) {
            cv += getAcceleration(t) * (tmp - t);
            return cv;
        }
        cv += getAcceleration(tmp) * p[1].duration;
        tmp += p[2].duration;
        if (t <= tmp) {
            return cv;
        }
        tmp += p[3].duration;
        if (t <= tmp) {
            cv += getAcceleration(t) * (tmp - t);
            return cv;
        } else {
            return target.velocity;
        }
    }

    public double getAcceleration(double t) {
        double tmp = p[1].duration;
        if (t <= tmp) {
            return constraints.maxAccel;
        }
        tmp += p[2].duration;
        if (t <= tmp) {
            return 0;
        }
        tmp += p[3].duration;
        if (t <= tmp) {
            return (-1) * constraints.maxAccel;
        } else {
            return 0;
        }
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
