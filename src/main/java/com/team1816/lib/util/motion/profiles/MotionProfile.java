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

        public State() {
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
    // 7 profile phases
    //    private Phase p1; // positive jerk, increasing acceleration t1
    //    private Phase p2; // zero jerk, positive acceleration t2
    //    private Phase p3; // negative jerk, decreasing acceleration t1
    //    private Phase p4; // zero jerk, zero acceleration t3
    //    private Phase p5; // negative jerk, decreasing acceleration t4
    //    private Phase p6; // zero jerk, negative acceleration t5
    //    private Phase p7; // positive jerk, increasing acceleration t4

    // the duration of odd phases can be computed by abs(Δa/j) and the duration of the even phases can be computed by abs(Δv/a)

    public MotionProfile() {
        for (int i = 0; i < p.length; i++) {
            p[i] = new Phase();
        }
        new MotionProfile(new Constraints(), new State(), new State());
    }

    public MotionProfile(Constraints c, State i, State t) { // with absolutely no troubleshooting :)
        constraints = c;
        initial = i;
        target = t;
        double dX = target.position - initial.position;
        double dV = target.velocity - initial.velocity;
        double t1 = constraints.maxAccel / constraints.maxJerk;
        double t2 = constraints.maxVel / constraints.maxAccel - constraints.maxAccel / constraints.maxJerk;

        //TODO: Corroborate these
        p[1].duration = t1;
        p[2].duration = t2;
        p[3].duration = t1;
        p[5].duration = t1;
        p[6].duration = t2;
        p[7].duration = t1;

        double cx = initial.position;
        double cv = initial.velocity;
        cx += constraints.maxJerk / 6 * Math.pow(p[1].duration, 3); // phase 1
        cv += constraints.maxJerk / 2 * Math.pow(p[1].duration, 2);
        cx += cv * p[2].duration + constraints.maxAccel / 2 * Math.pow(p[2].duration, 2);
        cv += constraints.maxAccel * p[2].duration;
        cx += cv * p[3].duration + constraints.maxAccel / 2 * Math.pow(p[3].duration, 2) - constraints.maxJerk / 6 * Math.pow(p[3].duration, 3);
        cv += constraints.maxAccel * p[3].duration + constraints.maxJerk / 2 * Math.pow(p[3].duration, 2);

        double t3 = cx / cv - 3 / 2 * constraints.maxAccel / constraints.maxJerk - constraints.maxVel / constraints.maxAccel;
        p[4].duration = t3;

        // back calculations
        // J = 3P/(t1(t1+t2)(3t1+3t2+2t3))
        // A = 2P/((t1+t2)(3t1+3t2+2t3))
        // V = 2P/(3t1+3t2+2t3)
    }

    public double getPosition(double t) {
        double cx = initial.position;
        double cv = initial.velocity;
        double tmp = 0;
        tmp += p[1].duration;
        if (t < tmp) {
            cx += constraints.maxJerk / 6 * Math.pow(tmp - t, 3); // phase 1
            return cx;
        }
        cx += constraints.maxJerk / 6 * Math.pow(p[1].duration, 3); // phase 1
        cv += constraints.maxJerk / 2 * Math.pow(p[1].duration, 2);
        tmp += p[2].duration;
        if (t < tmp) {
            cx += cv * (tmp - t) + constraints.maxAccel / 2 * Math.pow((tmp - t), 2);
            return cx;
        }
        cx += cv * p[2].duration + constraints.maxAccel / 2 * Math.pow(p[2].duration, 2);
        cv += constraints.maxAccel * p[2].duration;
        tmp += p[3].duration;
        if (t < tmp) {
            cx += cv * (tmp - t) + constraints.maxAccel / 2 * Math.pow((tmp - t), 2) - constraints.maxJerk / 6 * Math.pow((tmp - t), 3);
            return cx;
        }
        cx += cv * p[3].duration + constraints.maxAccel / 2 * Math.pow(p[3].duration, 2) - constraints.maxJerk / 6 * Math.pow(p[3].duration, 3);
        cv += constraints.maxAccel * p[3].duration - constraints.maxJerk / 2 * Math.pow(p[3].duration, 2);
        tmp += p[4].duration;
        if (t < tmp) {
            cx += cv * (tmp - t);
            return cx;
        }
        cx += cv * p[4].duration;
        tmp += p[5].duration;
        if (t < tmp) {
            cx += cv * (tmp - t) - constraints.maxJerk / 6 * Math.pow((tmp - t), 3);
            return cx;
        }
        cx += cv * p[5].duration - constraints.maxJerk / 6 * Math.pow(p[5].duration, 3);
        cv -= constraints.maxJerk / 2 * Math.pow(p[5].duration, 2);
        tmp += p[6].duration;
        if (t < tmp) {
            cx += cv * (tmp - t) - constraints.maxAccel / 2 * Math.pow((tmp - t), 2);
            return cx;
        }
        cx += cv * p[6].duration - constraints.maxAccel / 2 * Math.pow(p[6].duration, 2);
        cv -= constraints.maxAccel * p[6].duration;
        tmp += p[7].duration;
        if (t < tmp) {
            cx += cv * (tmp - t) - constraints.maxAccel / 2 * Math.pow((tmp - t), 2) + constraints.maxJerk / 6 * Math.pow((tmp - t), 7);
            return cx;
        } else {
            return target.position;
        }
    }

    public double getVelocity(double t) {
        double cv = initial.velocity;
        double tmp = 0;
        tmp += p[1].duration;
        if (t < tmp) {
            cv += constraints.maxJerk / 2 * Math.pow((tmp - t), 2);
            return cv;
        }
        cv += constraints.maxJerk / 2 * Math.pow(p[1].duration, 2);
        tmp += p[2].duration;
        if (t < tmp) {
            cv += constraints.maxAccel * (tmp - t);
            return cv;
        }
        cv += constraints.maxAccel * p[2].duration;
        tmp += p[3].duration;
        if (t < tmp) {
            cv += constraints.maxAccel * (tmp - t) - constraints.maxJerk / 2 * Math.pow((tmp - t), 2);
            return cv;
        }
        cv += constraints.maxAccel * p[3].duration - constraints.maxJerk / 2 * Math.pow(p[3].duration, 2);
        tmp += p[4].duration;
        if (t < tmp) {
            return cv;
        }
        tmp += p[5].duration;
        if (t < tmp) {
            cv -= constraints.maxJerk / 2 * Math.pow((tmp - t), 2);
            return cv;
        }
        cv -= constraints.maxJerk / 2 * Math.pow(p[5].duration, 2);
        tmp += p[6].duration;
        if (t < tmp) {
            cv -= constraints.maxAccel * (tmp - t);
            return cv;
        }
        cv -= constraints.maxAccel * p[6].duration;
        tmp += p[7].duration;
        if (t < tmp) {
            cv += (-1) * constraints.maxAccel * (tmp - t) + constraints.maxJerk / 2 * Math.pow((tmp - t), 2);
            return cv;
        } else {
            return target.velocity;
        }
    }

    public double getAcceleration(double t) {
        double ca = 0;
        double tmp = 0;
        tmp += p[1].duration;
        if (t < tmp) {
            ca += constraints.maxJerk * (tmp - t);
            return ca;
        }
        ca += constraints.maxJerk * p[1].duration;
        tmp += p[2].duration;
        if (t < tmp) {
            return ca;
        }
        tmp += p[3].duration;
        if (t < tmp) {
            ca -= constraints.maxJerk * (tmp - t);
            return ca;
        }
        ca -= constraints.maxJerk * p[3].duration;
        tmp += p[4].duration;
        if (t < tmp) {
            return ca;
        }
        tmp += p[5].duration;
        if (t < tmp) {
            ca -= constraints.maxJerk * (tmp - t);
            return ca;
        }
        tmp += p[6].duration;
        if (t < tmp) {
            return ca;
        }
        tmp += p[7].duration;
        if (t < tmp) {
            ca += constraints.maxJerk * (tmp - t);
            return ca;
        } else {
            return 0;
        }
    }

    public double getJerk(double t) {
        double tmp = p[1].duration;
        if (t < tmp) {
            return constraints.maxJerk;
        }
        tmp += p[2].duration;
        if (t < tmp) {
            return 0;
        }
        tmp += p[3].duration;
        if (t < tmp) {
            return -constraints.maxJerk;
        }
        tmp += p[4].duration;
        if (t < tmp) {
            return 0;
        }
        tmp += p[5].duration;
        if (t < tmp) {
            return -constraints.maxJerk;
        }
        tmp += p[5].duration;
        if (t < tmp) {
            return 0;
        }
        tmp += p[7].duration;
        if (t < tmp) {
            return constraints.maxJerk;
        } else {
            return 0;
        }
    }
}
