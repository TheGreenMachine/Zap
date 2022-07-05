package com.team1816.season.motion.profiles;

public class SinusoidalMotionProfile {

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

        public Phase(double d) {
            duration = d;
        }
    }

    private Phase[] p = new Phase[3]; // sinusodal acceleration, flat, deceleration
    private Constraints constraints;
    private State initial;
    private State target;
    private double duration;
    private boolean positive;

    public SinusoidalMotionProfile() {
        for (int i = 0; i < p.length; i++) {
            p[i] = new Phase();
            p[i].duration = 0;
        }
        duration = 0;
        positive = true;
        new SinusoidalMotionProfile(new Constraints(), new State(), new State());
    }

    public SinusoidalMotionProfile(Constraints c, State i, State t) {
        constraints = c;
        initial = i;
        target = t;

        double dX = t.position - i.position; // need to deal with dX sign
        double cx = 0;
        positive = dX > 0;

        double t1 = Math.abs(
            Math.PI /
            2 *
            Math.abs((positive ? (1) : (-1)) * c.maxVel - i.velocity) /
            c.maxAccel
        );
        double t2 = 0;
        double t3 = Math.abs(
            Math.PI /
            2 *
            Math.abs((positive ? (1) : (-1)) * c.maxVel - t.velocity) /
            c.maxAccel
        );

        // cx calculations
        cx +=
            Math.pow((positive ? (1) : (-1)) * (c.maxVel) + i.velocity, 2) /
            c.maxAccel *
            ((positive ? (1) : (-1)) * c.maxVel - i.velocity) *
            Math.PI /
            4;
        cx +=
            Math.pow((positive ? (1) : (-1)) * (c.maxVel) + t.velocity, 2) /
            c.maxAccel *
            ((positive ? (1) : (-1)) * c.maxVel - t.velocity) *
            Math.PI /
            4;
        cx += initial.velocity * t1;
        cx += target.velocity * t3;

        t2 += (Math.abs(dX) - cx) / c.maxVel;

        p[0].duration = t1;
        p[1].duration = t2;
        p[2].duration = t3;

        if (!positive) c.maxVel *= -1;

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
        return 0;
    }

    public double getVelocity(double t) {
        double cv = 0;
        double tmp = p[0].duration;
        if (t <= tmp) {
            cv +=
                (constraints.maxVel + initial.velocity) /
                (-2.0) *
                Math.cos(
                    2 *
                    (t) *
                    constraints.maxAccel /
                    (constraints.maxVel - initial.velocity)
                ) +
                (constraints.maxVel + initial.velocity) /
                2.0;
            return cv;
        }
        cv +=
            (constraints.maxVel + initial.velocity) /
            (-2.0) *
            Math.cos(
                2 * (tmp) * constraints.maxAccel / (constraints.maxVel - initial.velocity)
            ) +
            (constraints.maxVel + initial.velocity) /
            2.0;
        tmp += p[1].duration;
        if (t <= tmp) {
            return cv; // flat
        }
        tmp += p[2].duration;
        if (t <= tmp) {
            cv +=
                (constraints.maxVel + target.velocity) /
                (2.0) *
                Math.cos(
                    2 *
                    (tmp - t - p[2].duration) *
                    constraints.maxAccel /
                    (constraints.maxVel - target.velocity)
                ) +
                (constraints.maxVel + target.velocity) /
                2.0;
            return cv;
        }
        return 0;
    }

    public double getAcceleration(double t) {
        double ca = 0;
        double tmp = p[0].duration;
        if (t <= tmp) {
            ca +=
                -constraints.maxAccel *
                Math.sin(
                    2 *
                    (t) *
                    constraints.maxAccel /
                    (constraints.maxVel - initial.velocity)
                );
            return ca;
        }
        tmp += p[1].duration;
        if (t <= tmp) {
            return 0;
        }
        tmp += p[2].duration;
        if (t <= tmp) {
            ca +=
                constraints.maxAccel *
                Math.sin(
                    2 *
                    (tmp - t - p[2].duration) *
                    constraints.maxAccel /
                    (constraints.maxVel - target.velocity)
                );
            return ca;
        }
        return 0;
    }

    public double getJerk(double t) {
        double cj = 0;
        double tmp = p[0].duration;
        if (t <= tmp) {
            return (
                -getVelocity(t) *
                Math.pow(
                    2 * constraints.maxAccel / (constraints.maxVel - initial.velocity),
                    2
                )
            );
        }
        tmp += p[1].duration;
        if (t <= tmp) {
            return 0;
        }
        tmp += p[2].duration;
        if (t <= tmp) {
            return (
                -getVelocity(t) *
                Math.pow(
                    2 * constraints.maxAccel / (constraints.maxVel - target.velocity),
                    2
                )
            );
        }
        return 0;
    }

    public double getDuration() {
        return duration;
    }

    public boolean isFinished(double t) {
        return t >= duration;
    }
}
