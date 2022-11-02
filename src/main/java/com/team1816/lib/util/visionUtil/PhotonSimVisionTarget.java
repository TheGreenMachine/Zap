/*
 * MIT License
 *
 * Copyright (c) 2022 PhotonVision
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

package com.team1816.lib.util.visionUtil;

import edu.wpi.first.math.geometry.Pose2d;

public class PhotonSimVisionTarget {

    Pose2d targetPos;
    double targetWidthMeters;
    double targetHeightMeters;
    double targetHeightAboveGroundMeters;
    double tgtAreaMeters2;
    int fiducialID;

    /**
     * Describes a vision target located somewhere on the field that your SimVisionSystem can detect.
     *
     * @param targetPos Pose2d of the target on the field. Define it such that, if you are standing in
     *     the middle of the field facing the target, the Y axis points to your left, and the X axis
     *     points away from you.
     * @param targetHeightAboveGroundMeters Height of the target above the field plane, in meters.
     * @param targetWidthMeters Width of the outer bounding box of the target in meters.
     * @param targetHeightMeters Pair Height of the outer bounding box of the target in meters.
     */
    public PhotonSimVisionTarget(
        Pose2d targetPos,
        double targetHeightAboveGroundMeters,
        double targetWidthMeters,
        double targetHeightMeters,
        int fiducialID
    ) {
        this.targetPos = targetPos;
        this.targetHeightAboveGroundMeters = targetHeightAboveGroundMeters;
        this.targetWidthMeters = targetWidthMeters;
        this.targetHeightMeters = targetHeightMeters;
        this.tgtAreaMeters2 = targetWidthMeters * targetHeightMeters;
        this.fiducialID = fiducialID;
    }
}
