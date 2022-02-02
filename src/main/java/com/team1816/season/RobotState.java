package com.team1816.season;

import com.google.inject.Singleton;
import com.team254.lib.util.InterpolatingDouble;
import com.team254.lib.util.InterpolatingTreeMap;
import com.team254.lib.util.MovingAverageTwist2d;
import com.team254.lib.vision.AimingParameters;
import com.team254.lib.vision.GoalTracker;
import com.team254.lib.vision.GoalTracker.TrackReportComparator;
import com.team254.lib.geometry.Pose2d;
import com.team254.lib.geometry.Rotation2d;
import com.team254.lib.geometry.Translation2d;
import com.team254.lib.geometry.Twist2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import java.util.ArrayList;
import java.util.List;
import java.util.Map;
import java.util.Optional;

@Singleton
public class RobotState {

    private static final int kObservationBufferSize = 100;

    /*
     * RobotState keeps track of the poses of various coordinate frames throughout
     * the match. A coordinate frame is simply a point and direction in space that
     * defines an (x,y) coordinate system. Transforms (or poses) keep track of the
     * spatial relationship between different frames.
     *
     * Robot frames of interest (from parent to child):
     *
     * 1. Field frame: origin is where the robot is turned on.
     *
     * 2. Vehicle frame: origin is the center of the robot wheelbase, facing
     * forwards
     *
     * 3. Turret frame: origin is the center of the turret, which is coincident with
     * the origin of the vehicle frame but with potentially different angle.
     *
     * 4. Camera frame: origin is the center of the Limelight imager relative to the
     * turret.
     *
     * 5. Goal frame: origin is the center of the vision target, facing outwards
     * along the normal. Also note that there can be multiple goal frames.
     *
     * As a kinematic chain with 5 frames, there are 4 transforms of interest:
     *
     * 1. Field-to-vehicle: This is tracked over time by integrating encoder and
     * gyro measurements. It will inevitably drift, but is usually accurate over
     * short time periods.
     *
     * 2. Vehicle-to-turret: Measured by the turret encoder. This is a pure
     * rotation.
     *
     * 3. Turret-to-camera: This is a constant (per camera).
     *
     * 4. Camera-to-goal: Measured by the vision system.
     */

    // FPGATimestamp -> Pose2d or Rotation2d
    private InterpolatingTreeMap<InterpolatingDouble, Pose2d> field_to_vehicle_;
    private InterpolatingTreeMap<InterpolatingDouble, Rotation2d> vehicle_to_turret_;
    private Twist2d vehicle_velocity_predicted_;
    private Twist2d vehicle_velocity_measured_;
    private MovingAverageTwist2d vehicle_velocity_measured_filtered_;
    private double distance_driven_;

    private GoalTracker vision_target_low_ = new GoalTracker();
    private GoalTracker vision_target_high_ = new GoalTracker();

    List<Translation2d> mCameraToVisionTargetPosesLow = new ArrayList<>();
    List<Translation2d> mCameraToVisionTargetPosesHigh = new ArrayList<>();

    private Rotation2d headingRelativeToInitial = new Rotation2d();

    public final Field2d field = new Field2d();

    public RobotState() {
        SmartDashboard.putData("Field", field);
        reset(0.0, new Pose2d(), new Rotation2d());
    }

    /**
     * Resets the field to robot transform (robot's position on the field)
     */
    public synchronized void reset(
        double start_time,
        Pose2d initial_field_to_vehicle,
        Rotation2d initial_vehicle_to_turret
    ) {
        reset(start_time, initial_field_to_vehicle);
        vehicle_to_turret_ = new InterpolatingTreeMap<>(kObservationBufferSize);
        vehicle_to_turret_.put(
            new InterpolatingDouble(start_time),
            initial_vehicle_to_turret
        );
        field.setRobotPose(
            new edu.wpi.first.math.geometry.Pose2d(
                Constants.StartingPose.getTranslation().getX(),
                Constants.StartingPose.getTranslation().getY(),
                new edu.wpi.first.math.geometry.Rotation2d(
                    Constants.StartingPose.getRotation().getRadians()
                )
            )
        );
    }

    public synchronized void reset(double start_time, Pose2d initial_field_to_vehicle) {
        field_to_vehicle_ = new InterpolatingTreeMap<>(kObservationBufferSize);
        field_to_vehicle_.put(
            new InterpolatingDouble(start_time),
            initial_field_to_vehicle
        );
        vehicle_velocity_predicted_ = Twist2d.identity();
        vehicle_velocity_measured_ = Twist2d.identity();
        vehicle_velocity_measured_filtered_ = new MovingAverageTwist2d(25);
        distance_driven_ = 0.0;
    }

    public synchronized void reset() {
        reset(Timer.getFPGATimestamp(), new Pose2d(), new Rotation2d());
    }

    /**
     * Returns the robot's position on the field at a certain time. Linearly interpolates between stored robot positions
     * to fill in the gaps.
     */

    public synchronized double getEstimatedX() {
        return field_to_vehicle_.lastEntry().getValue().getTranslation().x();
    }

    public synchronized double getEstimatedY() {
        return field_to_vehicle_.lastEntry().getValue().getTranslation().y();
    }

    public synchronized double getPoseX(double timestamp) {
        return field_to_vehicle_
            .getInterpolated(new InterpolatingDouble(timestamp))
            .getTranslation()
            .x();
    }

    public synchronized double getPoseY(double timestamp) {
        return field_to_vehicle_
            .getInterpolated(new InterpolatingDouble(timestamp))
            .getTranslation()
            .y();
    }

    public synchronized Pose2d getFieldToVehicle(double timestamp) {
        return field_to_vehicle_.getInterpolated(new InterpolatingDouble(timestamp));
    }

    public synchronized Rotation2d getVehicleToTurret(double timestamp) {
        return vehicle_to_turret_.getInterpolated(new InterpolatingDouble(timestamp));
    }

    public synchronized Pose2d getFieldToTurret(double timestamp) {
        return getFieldToVehicle(timestamp)
            .transformBy(Pose2d.fromRotation(getVehicleToTurret(timestamp)));
    }

    public synchronized Map.Entry<InterpolatingDouble, Pose2d> getLatestFieldToVehicle() {
        return field_to_vehicle_.lastEntry();
    }

    public synchronized Map.Entry<InterpolatingDouble, Rotation2d> getLatestVehicleToTurret() {
        return vehicle_to_turret_.lastEntry();
    }

    public synchronized Pose2d getPredictedFieldToVehicle(double lookahead_time) {
        return getLatestFieldToVehicle()
            .getValue()
            .transformBy(Pose2d.exp(vehicle_velocity_predicted_.scaled(lookahead_time)));
    }

    public synchronized void addFieldToVehicleObservation(
        double timestamp,
        Pose2d observation
    ) {
        field_to_vehicle_.put(new InterpolatingDouble(timestamp), observation);
    }

    public synchronized void addVehicleToTurretObservation(
        double timestamp,
        Rotation2d observation
    ) {
        vehicle_to_turret_.put(new InterpolatingDouble(timestamp), observation);
    }

    /**
     * Rotation of robot relative to initial position,
     * unaffected by calls to {@link #reset()}
     */
    public Rotation2d getHeadingRelativeToInitial() {
        return headingRelativeToInitial;
    }

    public void setHeadingRelativeToInitial(Rotation2d heading) {
        this.headingRelativeToInitial = heading;
    }

    public double getLatestFieldToTurret() {
        Rotation2d fieldToTurret = getLatestFieldToVehicle()
            .getValue()
            .getRotation()
            .inverse()
            .rotateBy(getLatestVehicleToTurret().getValue());
        return fieldToTurret.getDegrees();
    }
    public synchronized double getDistanceDriven() {
        return distance_driven_;
    }

    public synchronized void resetDistanceDriven() {
        distance_driven_ = 0.0;
    }

    public synchronized Twist2d getPredictedVelocity() {
        return vehicle_velocity_predicted_;
    }

    public synchronized Twist2d getMeasuredVelocity() {
        return vehicle_velocity_measured_;
    }

    public synchronized Twist2d getSmoothedVelocity() {
        return vehicle_velocity_measured_filtered_.getAverage();
    }

    public synchronized void resetVision() {
        vision_target_low_.reset();
        vision_target_high_.reset();
    }

    // use known field target orientations to compensate for inaccuracy, assumes robot starts pointing directly away
    // from and perpendicular to alliance wall
    private final double[] kPossibleTargetNormals = {
        0.0,
        90.0,
        180.0,
        270.0,
        30.0,
        150.0,
        210.0,
        330.0,
    };

    public synchronized Pose2d getFieldToVisionTarget(boolean highTarget) {
        GoalTracker tracker = highTarget ? vision_target_high_ : vision_target_low_;

        if (!tracker.hasTracks()) {
            return null;
        }

        Pose2d fieldToTarget = tracker.getTracks().get(0).field_to_target;

        double normalPositive = (fieldToTarget.getRotation().getDegrees() + 360) % 360;
        double normalClamped = kPossibleTargetNormals[0];
        for (double possible : kPossibleTargetNormals) {
            if (
                Math.abs(normalPositive - possible) <
                Math.abs(normalPositive - normalClamped)
            ) {
                normalClamped = possible;
            }
        }

        return new Pose2d(
            fieldToTarget.getTranslation(),
            Rotation2d.fromDegrees(normalClamped)
        );
    }

    public synchronized Pose2d getVehicleToVisionTarget(
        double timestamp,
        boolean highTarget
    ) {
        Pose2d fieldToVisionTarget = getFieldToVisionTarget(highTarget);

        if (fieldToVisionTarget == null) {
            return null;
        }

        return getFieldToVehicle(timestamp).inverse().transformBy(fieldToVisionTarget);
    }

//    public synchronized Optional<AimingParameters> getAimingParameters(
//        boolean highTarget,
//        int prev_track_id,
//        double max_track_age
//    ) {
//        GoalTracker tracker = highTarget ? vision_target_high_ : vision_target_low_;
//        List<GoalTracker.TrackReport> reports = tracker.getTracks();
//
//        if (reports.isEmpty()) {
//            return Optional.empty();
//        }
//
//        double timestamp = Timer.getFPGATimestamp();
//
//        // Find the best track.
//        TrackReportComparator comparator = new TrackReportComparator(
//            Constants.kTrackStabilityWeight,
//            Constants.kTrackAgeWeight,
//            Constants.kTrackSwitchingWeight,
//            prev_track_id,
//            timestamp
//        );
//        reports.sort(comparator);
//
//        GoalTracker.TrackReport report = null;
//        for (GoalTracker.TrackReport track : reports) {
//            if (track.latest_timestamp > timestamp - max_track_age) {
//                report = track;
//                break;
//            }
//        }
//        if (report == null) {
//            return Optional.empty();
//        }
//        Pose2d vehicleToGoal = getFieldToVehicle(timestamp)
//            .inverse()
//            .transformBy(report.field_to_target)
//            .transformBy(getVisionTargetToGoalOffset());
//
//        AimingParameters params = new AimingParameters(
//            vehicleToGoal,
//            report.field_to_target,
//            report.field_to_target.getRotation(),
//            report.latest_timestamp,
//            report.stability,
//            report.id
//        );
//        return Optional.of(params);
//    }

    public Pose2d getRobot() {
        return new Pose2d();
    }

    public synchronized Pose2d getVisionTargetToGoalOffset() {
        // if (SuperstructureCommands.isInCargoShipPosition() && EndEffector.getInstance().getObservedGamePiece() == GamePiece.BALL) {
        //     return Pose2d.fromTranslation(new Translation2d(-6.0, 0.0));
        // }

        return new Pose2d();
    }

    public synchronized void outputToSmartDashboard() {
        SmartDashboard.putString("Robot Velocity", getMeasuredVelocity().toString());
        SmartDashboard.putNumber("Estimated Pose X", getEstimatedX());
        SmartDashboard.putNumber("Estimated Pose Y", getEstimatedY());

        SmartDashboard.putNumber("Field to Turret", getLatestFieldToTurret());
        SmartDashboard.putNumber(
            "Vehicle to Turret",
            getLatestVehicleToTurret().getValue().getDegrees()
        );
        SmartDashboard.putNumber(
            "Heading Relative to Initial",
            getHeadingRelativeToInitial().getDegrees()
        );
    }
}
