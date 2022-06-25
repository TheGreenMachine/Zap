package com.team1816.season.motion.profiles;

/**
 * This class will construct a jerk limited motion profile, by which it will take in parameters of maximum rate of change
 * of position, maximum rate of change of velocity, and maximum rate of change of acceleration and then construct the
 * appropriate motion profile. Note, that this is not the same as a trapezoidal motion profile but one degree higher.
 */
public class SMotionProfile {

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
    private Phase[] p = new Phase[7];
    private Constraints constraints;
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

    public SMotionProfile() {
        for (int i = 0; i < p.length; i++) {
            p[i] = new Phase();
            p[i].duration = 0;
        }
        duration = 0;
        new SMotionProfile(new Constraints(), new State(), new State());
    }

    public SMotionProfile(Constraints c, State i, State t) { // with absolutely no troubleshooting :)
        constraints = c;
        initial = i;
        target = t;
        double dX = target.position - initial.position;
        double dV = target.velocity - initial.velocity;
        double t1 = 0;
        double t2 = 0;
        double cV = c.maxVel;

        if (Math.abs(target.velocity)>Math.abs(c.maxVel)||Math.abs(initial.velocity)>Math.abs(c.maxVel)) {
            c.maxJerk = 0; c.maxAccel = 0; c.maxVel = 0;
        } else {
            if (cV >= c.maxAccel * c.maxAccel / c.maxJerk) {
                t1 = c.maxAccel / c.maxJerk;
                t2 = (cV - c.maxAccel * t1) / c.maxAccel;
            } else {
                t1 = Math.sqrt(cV / c.maxJerk);
                t2 = 0;
            }
            p[0].duration = p[2].duration = p[4].duration = p[6].duration = t1;
            p[1].duration = p[5].duration = t2;
            p[3].duration = 0;

            //calculate p[3].duration
            double tdX = 0;
            double t3 = 0;
            for(int l = 0; l<p.length; l++){
                t3+=p[l].duration;
            }
            tdX = getPosition(t3);
            p[3].duration = Math.max(0, (Math.abs(dX) - Math.abs(tdX))/cV);
            // note that if this is truly negative, you will have to reduce the velocity to corroborate for it
            // ie it assumes that the motion profile is declared sensibly.
        }

        for(Phase ph: p) {
            duration += Math.max(ph.duration, 0);
        }

        // back calculations
        // J = 3P/(t1(t1+t2)(3t1+3t2+2t3))
        // A = 2P/((t1+t2)(3t1+3t2+2t3))
        // V = 2P/(3t1+3t2+2t3)
        if(dX<0) {
            constraints.maxJerk*=(-1);
        }
    }

    public double getPosition(double t) {
        double cx = initial.position;
        double tmp = 0;
        tmp += p[0].duration;
        if (t <= tmp) {
            cx += getJerk(t) / 6 * Math.pow(tmp - t, 3); // phase 1
            return cx;
        }
        cx += getJerk(tmp) / 6 * Math.pow(p[0].duration, 3); // phase 1
        tmp += p[1].duration;
        if (t <= tmp) {
            cx += getVelocity(tmp - p[1].duration) * (tmp - t) + getAcceleration(t) / 2 * Math.pow((tmp - t), 2);
            return cx;
        }
        cx += getVelocity(tmp - p[1].duration) * p[1].duration + getAcceleration(tmp) / 2 * Math.pow(p[1].duration, 2);
        tmp += p[2].duration;
        if (t <= tmp) {
            cx +=
                getVelocity(tmp - p[2].duration) *
                (tmp - t) +
                getAcceleration(tmp - p[2].duration) /
                2 *
                Math.pow((tmp - t), 2) +
                getJerk(t) /
                6 *
                Math.pow((tmp - t), 3);
            return cx;
        }
        cx +=
            getVelocity(tmp - p[2].duration) *
            p[2].duration +
            getAcceleration(tmp - p[2].duration) /
            2 *
            Math.pow(p[2].duration, 2) +
            getJerk(tmp) /
            6 *
            Math.pow(p[2].duration, 3);
        tmp += p[3].duration;
        if (t <= tmp) {
            cx += getVelocity(t) * (tmp - t);
            return cx;
        }
        cx += getVelocity(tmp) * p[3].duration;
        tmp += p[4].duration;
        if (t <= tmp) {
            cx += getVelocity(tmp - p[4].duration) * (tmp - t) + getJerk(t) / 6 * Math.pow((tmp - t), 3);
            return cx;
        }
        cx += getVelocity(tmp - p[4].duration) * p[4].duration + getJerk(tmp) / 6 * Math.pow(p[4].duration, 3);
        tmp += p[5].duration;
        if (t <= tmp) {
            cx += getVelocity(tmp - p[5].duration) * (tmp - t) + getAcceleration(t) / 2 * Math.pow((tmp - t), 2);
            return cx;
        }
        cx += getVelocity(tmp - p[5].duration) * p[5].duration + getAcceleration(tmp) / 2 * Math.pow(p[5].duration, 2);
        tmp += p[6].duration;
        if (t <= tmp) {
            cx +=
                getVelocity(tmp - p[6].duration) *
                (tmp - t) +
                getAcceleration(tmp - p[6].duration) /
                2 *
                Math.pow((tmp - t), 2) +
                getJerk(t) /
                6 *
                Math.pow((tmp - t), 3);
            return cx;
        } else {
            return target.position;
        }
    }

    public double getVelocity(double t) {
        double cv = initial.velocity;
        double tmp = 0;
        tmp += p[0].duration;
        if (t <= tmp) {
            cv += getJerk(t) / 2 * Math.pow((tmp - t), 2);
            return cv;
        }
        cv += getJerk(tmp) / 2 * Math.pow(p[0].duration, 2);
        tmp += p[1].duration;
        if (t <= tmp) {
            cv += getAcceleration(t) * (tmp - t);
            return cv;
        }
        cv += getAcceleration(tmp) * p[1].duration;
        tmp += p[2].duration;
        if (t <= tmp) {
            cv +=
                getAcceleration(tmp - p[2].duration) *
                (tmp - t) +
                getJerk(t) /
                2 *
                Math.pow((tmp - t), 2);
            return cv;
        }
        cv +=
            getAcceleration(tmp - p[2].duration) *
            p[2].duration +
            getJerk(tmp) /
            2 *
            Math.pow(p[2].duration, 2);
        tmp += p[3].duration;
        if (t <= tmp) {
            return cv;
        }
        tmp += p[4].duration;
        if (t <= tmp) {
            cv += getJerk(t) / 2 * Math.pow((tmp - t), 2);
            return cv;
        }
        cv += getJerk(tmp) / 2 * Math.pow(p[4].duration, 2);
        tmp += p[5].duration;
        if (t <= tmp) {
            cv += getAcceleration(t) * (tmp - t);
            return cv;
        }
        cv += getAcceleration(tmp) * p[5].duration;
        tmp += p[6].duration;
        if (t <= tmp) {
            cv +=
                getAcceleration(tmp - p[6].duration) *
                (tmp - t) +
                getJerk(t) /
                2 *
                Math.pow((tmp - t), 2);
            return cv;
        } else {
            return target.velocity;
        }
    }

    public double getAcceleration(double t) {
        double ca = 0;
        double tmp = 0;
        tmp += p[0].duration;
        if (t <= tmp) {
            ca += getJerk(t) * (tmp - t);
            return ca;
        }
        ca += getJerk(tmp) * p[0].duration;
        tmp += p[1].duration;
        if (t <= tmp) {
            return ca;
        }
        tmp += p[2].duration;
        if (t <= tmp) {
            ca += getJerk(t) * (tmp - t);
            return ca;
        }
        ca += getJerk(tmp) * p[2].duration;
        tmp += p[3].duration;
        if (t <= tmp) {
            return ca;
        }
        tmp += p[4].duration;
        if (t <= tmp) {
            ca += getJerk(t) * (tmp - t);
            return ca;
        }
        ca += getJerk(tmp) * p[4].duration;
        tmp += p[5].duration;
        if (t <= tmp) {
            return ca;
        }
        tmp += p[6].duration;
        if (t <= tmp) {
            ca += getJerk(t) * (tmp - t);
            return ca;
        } else {
            return 0;
        }
    }

    public double getJerk(double t) {
        double tmp = p[0].duration;
        if (t <= tmp) {
            return constraints.maxJerk;
        }
        tmp += p[1].duration;
        if (t <= tmp) {
            return 0;
        }
        tmp += p[2].duration;
        if (t <= tmp) {
            return -constraints.maxJerk;
        }
        tmp += p[3].duration;
        if (t <= tmp) {
            return 0;
        }
        tmp += p[4].duration;
        if (t <= tmp) {
            return -constraints.maxJerk;
        }
        tmp += p[5].duration;
        if (t <= tmp) {
            return 0;
        }
        tmp += p[6].duration;
        if (t <= tmp) {
            return constraints.maxJerk;
        } else {
            return 0;
        }
    }

    public boolean isFinished(double t) {
        return t>duration;
    }
}
