package com.team1816.season.motion.profiles;

/**
 * This class will construct a jerk limited motion profile, by which it will take in parameters of maximum rate of change
 * of position, maximum rate of change of velocity, and maximum rate of change of acceleration and then construct the
 * appropriate motion profile. Note, that this is not the same as a trapezoidal motion profile but one degree higher.
 */
public class SCurveMotionProfile {

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

    /**
     * Profile properties
     */
    private Phase[] p = new Phase[3];
    private Constraints constraints;
    private TrapezoidalMotionProfile p1, p2;
    private double targetMaxVel;
    private State initial;
    private State target;
    private double duration;

    // 7 profile phases
    //    private Phase p1; // positive jerk, increasing acceleration t1
    //    private Phase p2; // zero jerk, positive acceleration t2
    //    private Phase p3; // negative jerk, decreasing acceleration t1
    //    private Phase p4; // zero jerk, zero acceleration t3
    //    private Phase p5; // negative jerk, decreasing acceleration t4
    //    private Phase p6; // zero jerk, negative acceleration t5
    //    private Phase p7; // positive jerk, increasing acceleration t4

    // the duration of odd phases can be computed by abs(Δa/j) and the duration of the even phases can be computed by abs(Δv/a)

    public SCurveMotionProfile() {
        for (int i = 0; i < p.length; i++) {
            p[i] = new Phase();
            p[i].duration = 0;
        }
        p1 = p2 = new TrapezoidalMotionProfile();
        duration = 0;
        new SCurveMotionProfile(new Constraints(), new State(), new State());
    }

    public SCurveMotionProfile(Constraints c, State i, State t) { // with absolutely no troubleshooting :)
        constraints = c;
        initial = i;
        target = t;
        double dX = target.position - initial.position;
        double dV = target.velocity - initial.velocity;
        double t1 = 0;
        double t2 = 0;
        double cV = c.maxVel;
        // we're going to trick a regular trapezoidal profile to be applied to acceleration instead of velocity
        if (dX<0) {
            targetMaxVel = constraints.maxVel * (-1);
        } else {
            targetMaxVel = constraints.maxVel;
        }
        p1 = new TrapezoidalMotionProfile(new TrapezoidalMotionProfile.Constraints(c.maxAccel, c.maxJerk, 0), new TrapezoidalMotionProfile.State(i.velocity), new TrapezoidalMotionProfile.State(targetMaxVel));
        p2 = new TrapezoidalMotionProfile(new TrapezoidalMotionProfile.Constraints(c.maxAccel, c.maxJerk, 0), new TrapezoidalMotionProfile.State(targetMaxVel), new TrapezoidalMotionProfile.State(t.velocity));
        double cx = 0;
        cx += p1.getPosition(p1.getDuration());
        cx += p2.getPosition(p2.getDuration());
        while (Math.abs(cx)>Math.abs(dX)) {
            cx = 0;
            p1 = new TrapezoidalMotionProfile(new TrapezoidalMotionProfile.Constraints(p1.getConstraints().getMaxVel()*0.8, c.maxJerk, 0), new TrapezoidalMotionProfile.State(i.velocity), new TrapezoidalMotionProfile.State(targetMaxVel));
            p2 = new TrapezoidalMotionProfile(new TrapezoidalMotionProfile.Constraints(p1.getConstraints().getMaxVel()*0.8, c.maxJerk, 0), new TrapezoidalMotionProfile.State(targetMaxVel), new TrapezoidalMotionProfile.State(t.velocity));
            cx += p1.getPosition(p1.getDuration());
            cx += p2.getPosition(p2.getDuration());
        }
        p[0].duration = p1.getDuration();
        p[2].duration = p2.getDuration();
        p[1].duration = (Math.abs(dX)-Math.abs(cx))/(constraints.maxVel);
        // alternate method
        /*
        if (
            Math.abs(target.velocity) > Math.abs(c.maxVel) ||
            Math.abs(initial.velocity) > Math.abs(c.maxVel)
        ) {
            c.maxJerk = 0;
            c.maxAccel = 0;
            c.maxVel = 0;
        } else {
            if (cV >= c.maxAccel * c.maxAccel / c.maxJerk) {
                t1 = c.maxAccel / c.maxJerk;
                t2 = (cV - c.maxAccel * t1) / c.maxAccel;
            } else {
                t1 = Math.sqrt(cV / c.maxJerk);
                t2 = 0;
            }
            p[0].duration = p[2].duration = p[4].duration = p[6].duration = t1;
            p[1].duration = p[5].duration = t2; // this is false
            p[3].duration = 0;

            //calculate p[3].duration
            double tdX = 0;
            double t3 = 0;
            for (int l = 0; l < p.length; l++) {
                t3 += p[l].duration;
            }
            tdX = getPosition(t3);
            p[3].duration = Math.max(0, (Math.abs(dX) - Math.abs(tdX)) / cV);
            // note that if this is truly negative, you will have to reduce the velocity to corroborate for it
            // ie it assumes that the motion profile is declared sensibly.
        }

        for (Phase ph : p) {
            duration += Math.max(ph.duration, 0);
        }

        // back calculations
        // J = 3P/(t1(t1+t2)(3t1+3t2+2t3))
        // A = 2P/((t1+t2)(3t1+3t2+2t3))
        // V = 2P/(3t1+3t2+2t3)
        if (dX < 0) {
            constraints.maxJerk *= (-1);
        }*/
    }

    public double getPosition(double t) {
        double cx = initial.position;
        double tmp = p[0].duration;
        if (t <= tmp) {
            //TODO: finish this
        }
        return target.position;
    }

    public double getVelocity(double t) {
        double cv = initial.velocity;
        double tmp = 0;
        tmp += p[0].duration;
        if (t <= tmp) {
            return p1.getPosition(t);
        }
        cv += p1.getPosition(p1.getDuration());
        tmp += p[1].duration;
        if (t <= tmp) {
            return cv; //return targetMaxVel;
        }
        tmp += p[2].duration;
        if (t <= tmp) {
            return p2.getPosition(t);
        }
        return target.velocity;
    }

    public double getAcceleration(double t) {
        double ca = 0;
        double tmp = 0;
        tmp += p[0].duration;
        if (t <= tmp) {
            return getVelocity(t);
        }
        ca += p1.getVelocity(p1.getDuration());
        tmp += p[1].duration;
        if (t <= tmp) {
            return ca;
        }
        tmp += p[2].duration;
        if (t <= tmp) {
            return p2.getVelocity(t);
        }
        return 0;
    }

    public double getJerk(double t) {
        double cj = 0;
        double tmp = 0;
        tmp += p[0].duration;
        if (t <= tmp) {
            return getAcceleration(t);
        }
        cj += p1.getAcceleration(p1.getDuration());
        tmp += p[1].duration;
        if (t <= tmp) {
            return 0;
        }
        tmp += p[2].duration;
        if (t <= tmp) {
            return p2.getAcceleration(t);
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
