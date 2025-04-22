package team3647.lib.vision;

import static edu.wpi.first.units.Units.Meters;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Timer;

import java.util.ArrayList;
import java.util.List;
import java.util.Optional;
import java.util.Random;
import java.util.Set;
import java.util.function.Function;
import org.littletonrobotics.junction.Logger;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonTrackedTarget;

import team3647.frc2025.constants.FieldConstants;
import team3647.lib.vision.old.AprilTagCamera.AprilTagId;

public class AprilTagPhotonVision extends PhotonCamera implements AprilTagCamera {

    AprilTagFieldLayout aprilTagFieldLayout;
    PhotonPoseEstimator photonPoseEstimator;
    Transform3d robotToCam;
    private final edu.wpi.first.math.Vector<N3> baseStdDevs;
    private final edu.wpi.first.math.Vector<N3> multiStdDevs =
            VecBuilder.fill(0.00096, 0.00096, 0.02979);
    private boolean hasPriority = false;
    private final String name;

    public final Function<EstimatedRobotPose, Boolean> customHeuristics;

    public AprilTagPhotonVision(
            String camera,
            Transform3d robotToCam,
            edu.wpi.first.math.Vector<N3> baseStdDevs,
            Function<EstimatedRobotPose, Boolean> customHeuristics,
            AprilTagFieldLayout layout) {

        super(NetworkTableInstance.getDefault(), camera);
        this.aprilTagFieldLayout = layout;
        this.name = camera;
        photonPoseEstimator =
                new PhotonPoseEstimator(
                        aprilTagFieldLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, robotToCam);
        this.baseStdDevs = baseStdDevs;
        this.robotToCam = robotToCam;
        this.customHeuristics = customHeuristics;
    }

    public AprilTagPhotonVision(
            String camera, Transform3d robotToCam, edu.wpi.first.math.Vector<N3> baseStdDevs) {
        this(camera, robotToCam, baseStdDevs, (update) -> false, AprilTagFieldLayout.loadField(AprilTagFields.k2025Reefscape));
    }

    public AprilTagPhotonVision(String camera, Transform3d robotToCam) {
        this(camera, robotToCam, VecBuilder.fill(0.05, 0.05, 0.1), (update) -> false, AprilTagFieldLayout.loadField(AprilTagFields.k2025Reefscape));
    }

    public AprilTagId getId(int id) {
        var adjustedId = id - 1;
        var possibleValues = AprilTagId.values();
        if (adjustedId < 0 || adjustedId > possibleValues.length) {
            return AprilTagId.ID_DNE;
        }

        return possibleValues[adjustedId];
    }

    public AprilTagPhotonVision withPriority(boolean priority) {
        this.hasPriority = true;
        return this;
    }

    public String getName() {
        return this.name;
    }

    public Optional<Pose3d> camPose() {
        var update =
                photonPoseEstimator.update(
                        this.getLatestResult(), this.getCameraMatrix(), this.getDistCoeffs());
        if (update.isEmpty()) {
            return Optional.empty();
        }
        var bruh = update.get().estimatedPose;
        return Optional.of(bruh.transformBy(robotToCam));
    }

    public Optional<VisionMeasurement> QueueToInputs() {
        var resultList = this.getAllUnreadResults();
        if (resultList.size() <= 0) {
            return Optional.empty();
        }

        var result = resultList.get(0);
        if (!result.hasTargets()) {
            return Optional.empty();
        }
        var update = photonPoseEstimator.update(result);
        if (update.isEmpty()) {
            return Optional.empty();
        }

        Logger.recordOutput("Robot/unfilteredVision", update.get().estimatedPose);

        if (customHeuristics.apply(update.get())) {
            return Optional.empty();
        }
        double targetDistance =
                result.getBestTarget()
                        .getBestCameraToTarget()
                        .getTranslation()
                        .toTranslation2d()
                        .getNorm();
        if (hasPriority) {
            targetDistance = MathUtil.clamp(targetDistance - 3, 1, 1000);
        }
        // if (result.getBestTarget().getFiducialId() != 3
        //         && result.getBestTarget().getFiducialId() != 4) {
        //     return Optional.empty();
        // }
        if (targetDistance > 4 && !hasPriority) {
            return Optional.empty();
        }
        if (targetDistance > 8) {
            return Optional.empty();
        }
        if (Math.abs(update.get().estimatedPose.getZ()) > 0.5) {
            return Optional.empty();
        }
        if (update.get().estimatedPose.getX() > FieldConstants.kFieldLength.in(Meters)
                || update.get().estimatedPose.getX() < 0
                || update.get().estimatedPose.getY() > FieldConstants.kFieldWidth.in(Meters)
                || update.get().estimatedPose.getY() < 0) {
            return Optional.empty();
        }

        // if (result.getBestTarget().getFiducialId() == 5
        //         || result.getBestTarget().getFiducialId() == 6) {
        //     return Optional.empty();
        // }
        // Logger.recordOutput(
        //         "Cams/" + this.getName(), update.get().estimatedPose.transformBy(robotToCam));
        double numTargets = result.getTargets().size();
        if (hasPriority && numTargets < 2) {
            return Optional.empty();
        }
        final var stdDevs = baseStdDevs.times(targetDistance).times(8 / Math.pow(numTargets, 3));
        double ambiguityScore =
                1 / (numTargets * 100 + (1 - result.getBestTarget().getPoseAmbiguity()));
        final double priorityScore = this.hasPriority ? 50 : 0;
        ambiguityScore += priorityScore;
        if (result.targets.stream().anyMatch(target -> target.getPoseAmbiguity() > 0.2)) {
            return Optional.empty();
        }
        VisionMeasurement measurement =
                VisionMeasurement.fromEstimatedRobotPose(
                        update.get(),
                        update.get().timestampSeconds,
                        ambiguityScore,
                        stdDevs,
                        getName());
        return Optional.of(measurement);
    }

    @Override
    public double getTx() {
        var resultList = this.getAllUnreadResults();
        if (resultList.size() <= 0) {
            return 0;
        }

        var result = resultList.get(0);
        if (!result.hasTargets()) return 0;

        return result.getBestTarget().yaw;
    }

    @Override
    public double getTy() {
        var resultList = this.getAllUnreadResults();
        if (resultList.size() <= 0) {
            return 0;
        }

        var result = resultList.get(0);
        if (!result.hasTargets()) return 0;

        return result.getBestTarget().pitch;
    }

    @Override
    public double getTa() {
        var resultList = this.getAllUnreadResults();
        if (resultList.size() <= 0) {
            return 0;
        }

        var result = resultList.get(0);
        if (!result.hasTargets()) return 0;

        return result.getBestTarget().area;
    }

    @Override
    public boolean hasTarget() {
        return this.getAllUnreadResults().get(0).hasTargets();
    }

    public int getTagNum() {
        var result = this.getLatestResult();
        if (result.hasTargets()) {
            return result.getBestTarget().getFiducialId();
        } else {
            return -1;
        }
    }

    // public void addGyroData(Orientation orientation){
    //     photonPoseEstimator.addHeadingData(Timer.getTimestamp(), Rotation2d.fromDegrees(orientation.yaw()));
    // }


}
