package com.team1816.lib.math;

import com.team254.lib.util.Util;
import edu.wpi.first.math.geometry.Pose2d;

public class Conversions {

    // make the degree stuff to ticks/radians/ whatnot come into here instead of being spread out everywhere

    public static boolean epsilonEquals(final Pose2d reference, final Pose2d other, double epsilon) {
        return Util.epsilonEquals(reference.getX(), other.getX(), epsilon)
            && Util.epsilonEquals(reference.getY(), other.getY(), epsilon)
            && Util.epsilonEquals(reference.getRotation().getDegrees(), other.getRotation().getDegrees(), epsilon
        );
    }

}
