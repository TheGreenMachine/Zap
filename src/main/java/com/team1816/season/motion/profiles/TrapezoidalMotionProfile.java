package com.team1816.season.motion.profiles;

/**
 * This class will construct a standard trapezoidal motion profile.
 */
public class TrapezoidalMotionProfile {

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

    private Phase[] p = new Phase[3];
    private Constraints constraints;
    private State initial;
    private State target;
    private double duration;

    private double targetMaxVelocity = 0;
    private double targetMaxAcceleration = 0;

    public TrapezoidalMotionProfile() {
        for (int i = 0; i < p.length; i++) {
            p[i] = new Phase();
            p[i].duration = 0;
        }
        duration = 0;
        new TrapezoidalMotionProfile(new Constraints(), new State(), new State());
    }

    public TrapezoidalMotionProfile(Constraints c, State i, State t) {
        super();
        constraints = c;
        initial = i;
        target = t;

        double dX = t.position - i.position;
        double dV = t.velocity - i.velocity;

        double t1 = 0;
        double t2 = 0;
        double t3 = 0;

        if (dX >= 0) {
            targetMaxVelocity = c.getMaxVel();
            targetMaxAcceleration = c.getMaxAccel();
        } else if (dX < 0) {
            targetMaxVelocity = c.getMaxVel() * (-1);
            targetMaxAcceleration = c.getMaxAccel() * (-1); // quick fix to get accurate position and velocity calculations
        }
        if (c.getMaxAccel() == 0) {
            c.maxAccel = 1;
        }
        do {
            t1 = Math.abs((targetMaxVelocity - i.velocity) / targetMaxAcceleration);
            t3 = Math.abs((targetMaxVelocity - t.velocity) / targetMaxAcceleration);

            double edX =
                (targetMaxVelocity - i.velocity) *
                (t1) /
                2 +
                (targetMaxVelocity - t.velocity) *
                (t3) /
                2;

            t2 = (dX - edX) / (targetMaxVelocity);

            if (t2 < 0) targetMaxVelocity *= 0.8;
        } while (t2 < 0);

        for (int x = 0; x < p.length; x++) {
            p[x] = new Phase();
        }

        p[0].duration = t1;
        p[1].duration = t2;
        p[2].duration = t3;

        duration = 0;
        for (Phase ph : p) {
            duration += Math.max(ph.duration, 0);
        }
    }

    public Constraints getConstraints() {
        return constraints;
    }

    public Phase[] getPhases() {
        return p;
    }

    public double getPosition(double t) {
        double cx = initial.position;
        double tmp = p[0].duration;
        if (t <= tmp) {
            cx += getAcceleration(t) / 2 * Math.pow((tmp - t), 2);
            return cx;
        }
        cx += getAcceleration(tmp) / 2 * Math.pow(p[0].duration, 2);
        tmp += p[1].duration;
        if (t <= tmp) {
            cx += getVelocity(t) * (tmp - t);
            return cx;
        }
        cx += getVelocity(tmp) * (p[1].duration);
        tmp += p[2].duration;
        if (t <= tmp) {
            cx +=
                getVelocity(tmp - p[2].duration) *
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
        double tmp = p[0].duration;
        if (t <= tmp) {
            cv += getAcceleration(t) * (tmp - t);
            return cv;
        }
        cv += getAcceleration(tmp) * p[0].duration;
        tmp += p[1].duration;
        if (t <= tmp) {
            return cv;
        }
        tmp += p[2].duration;
        if (t <= tmp) {
            cv += getAcceleration(t) * (tmp - t);
            return cv;
        } else {
            return target.velocity;
        }
    }

    public double getAcceleration(double t) {
        double tmp = p[0].duration;
        if (t <= tmp) {
            return targetMaxAcceleration;
        }
        tmp += p[1].duration;
        if (t <= tmp) {
            return 0;
        }
        tmp += p[2].duration;
        if (t <= tmp) {
            return (-1) * targetMaxAcceleration;
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
