package team3647.lib.vision;

import static edu.wpi.first.units.Units.Degree;
import static edu.wpi.first.units.Units.Millisecond;
import static edu.wpi.first.units.Units.Second;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

import java.lang.StackWalker.Option;
import java.util.Optional;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;
import team3647.frc2025.constants.FieldConstants;
import team3647.lib.team6328.VirtualSubsystem;
import team3647.lib.vision.LimelightHelpers.PoseEstimate;
import team3647.lib.vision.LimelightHelpers.RawFiducial;

public class AprilTagLimelight extends VirtualSubsystem implements AprilTagCamera {

    public final String name;
    public final Transform3d robotToCamera;

    public boolean useMt2 = false;
    public boolean convergeToMT1 = false;

    private final int kAprilTagPipelineIndex = 0;

    private final Supplier<Orientation> orientation;

    public final PoseEstimate kEmpty =
            new PoseEstimate(
                    Pose2d.kZero,
                    -1,
                    -1,
                    -1,
                    -1,
                    -1,
                    -1,
                    new RawFiducial[] {new RawFiducial(-1, -1, -1, -1, -1, -1, 9999999)},
                    false);

    private final Vector<N3> baseStdDevs;

    AprilTagFieldLayout aprilTagFieldLayout =
            AprilTagFieldLayout.loadField(AprilTagFields.k2025Reefscape);

    public AprilTagLimelight(
            String name,
            Transform3d robotToCamera,
            Supplier<Orientation> orientationSupplier,
            Vector<N3> baseStdDevs) {
        super();
        this.name = name;
        this.robotToCamera = robotToCamera;
        this.orientation = orientationSupplier;
        setIMUMode(3);
        // LimelightHelpers.setCameraPose_RobotSpace(
        //         name,
        //         robotToCamera.getX(),
        //         robotToCamera.getY(),
        //         robotToCamera.getX(),
        //         robotToCamera.getRotation().getX(),
        //         robotToCamera.getRotation().getY(),
        //         robotToCamera.getRotation().getZ());
        LimelightHelpers.setPipelineIndex(name, kAprilTagPipelineIndex);
        LimelightHelpers.SetIMUAssistAlpha(name, 3);

        this.baseStdDevs = baseStdDevs;
    }

    @Override
    public Optional<Pose3d> camPose() {

        var robotPose = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(name);
        if (robotPose.isEmpty()) {
            return Optional.empty();
        }
        var robotPose3d = new Pose3d(robotPose.get().pose);

        return Optional.of(robotPose3d.transformBy(robotToCamera));
    }

    public void setOrientation() {
        var angle = orientation.get();
        LimelightHelpers.SetRobotOrientation(name, angle.yaw.in(Degree), 0, 0, 0, 0, 0);
    }

    private boolean isMultitag() {
        var sampleMaybe = LimelightHelpers.getBotPoseEstimate_wpiBlue(name);

        if (sampleMaybe.isEmpty()) {
            return false;
        }

        var sample = sampleMaybe.get();
        return sample.tagCount > 1;
    }

    public Command setConvergeToMT1() {
        return Commands.runOnce(() -> convergeToMT1 = true);
    }

    public Command setConvergeToGyro() {
        return Commands.runOnce(() -> convergeToMT1 = false);
    }

    @Override
    public void periodic() {
        setOrientation();

        useMt2 = VisionController.hasReset && !isMultitag();
        if (DriverStation.isEnabled()) {
            LimelightHelpers.cancelThrottle(name);
            // setIMUMode(1);
        }
        if (DriverStation.isDisabled()) {
            LimelightHelpers.setThrottle(name);
            // setIMUMode(2);
        }

        setIMUMode((!VisionController.hasReset) ? 1 : 2);

        Logger.recordOutput("DEBUG/autoAlign/hasreset", VisionController.hasReset);
        Logger.recordOutput("DEBUG/autoAlign/useMT2", useMt2);
        Logger.recordOutput("DEBUG/autoAlign/is not multitag", !isMultitag());
    }

    public RawFiducial getBestTagDist(RawFiducial[] fiducials) {
        double lowest = Double.POSITIVE_INFINITY;
        RawFiducial bestTag = fiducials[0];
        for (RawFiducial rawFiducial : fiducials) {
            if (rawFiducial.distToCamera < lowest) {
                lowest = rawFiducial.distToCamera;
                bestTag = rawFiducial;
            }
        }

        return bestTag;
    }

    public void setIMUMode(int mode) {
        LimelightHelpers.SetIMUMode(name, mode);
    }

    public RawFiducial getBestTag(RawFiducial[] fiducials) {
        RawFiducial best = fiducials[0];
        double lowest = Double.POSITIVE_INFINITY;
        double lowestAmbig = 999999;

        for (RawFiducial rawFiducial : fiducials) {
            if (rawFiducial.distToCamera < lowest && rawFiducial.ambiguity < lowestAmbig) {
                lowest = rawFiducial.distToCamera;
                lowestAmbig = rawFiducial.ambiguity;
                best = rawFiducial;
            }
        }

        return best;
    }

  

    @Override
    public Optional<VisionMeasurement> QueueToInputs() {
        setOrientation();
        var botPose =
                VisionController.hasReset
                        ? LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(name)
                        : LimelightHelpers.getBotPoseEstimate_wpiBlue(name);

		var botPoseRaw = LimelightHelpers.getBotPose3d_wpiBlue(name);
        if (botPose.isEmpty() || botPoseRaw.isEmpty()) {
            return Optional.empty();
        }
        var result = botPose.get();
        if (result.tagCount == 0) {
            return Optional.empty();
        }

		if (botPoseRaw.get().getZ() > 0.2) {
			return Optional.empty();
		}
        var bestTag = getBestTag(result.rawFiducials);

		
        if (bestTag.distToCamera > 3.5) {
            return Optional.empty();
        }

		if (bestTag.ambiguity > 0.2) {
			return Optional.empty();
		}



        if (result.pose.getMeasureX().gt(FieldConstants.kFieldLength)
                || result.pose.getX() < 0
                || result.pose.getMeasureY().gt(FieldConstants.kFieldWidth)
                || result.pose.getY() < 0) {
            return Optional.empty();
        }

        var stdDevs = baseStdDevs.times(result.avgTagDist / Math.pow(result.tagCount, 3));

        var ambiguityScore = 1 / (result.tagCount * 100 + (1 - bestTag.ambiguity));

        double timestamp = Timer.getFPGATimestamp() - Millisecond.of(result.latency).in(Second);

        var measurement =
                new VisionMeasurement(result.pose, timestamp, ambiguityScore, stdDevs, name);

        return Optional.of(measurement);
    }

    @Override
    public int getTagNum() {
        var botPose = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(name);

        if (botPose.isEmpty()) {
            return -1;
        }
        var bestTag = getBestTag(botPose.get().rawFiducials);

        return bestTag.id;
    }

    @Override
    public double getTx() {
        return LimelightHelpers.getTXNC(name);
    }

    @Override
    public double getTy() {
        return LimelightHelpers.getTYNC(name);
    }

    @Override
    public double getTa() {
        return LimelightHelpers.getTA(name);
    }

    public double getTd() {
        var allTags = LimelightHelpers.getRawFiducials(name);
        Logger.recordOutput("tages???", allTags.length);
        if (allTags.length < 1) return -1;
        var bestTag = getBestTagDist(allTags);
        return bestTag.distToCamera;
    }

    @Override
    public boolean hasTarget() {
        return LimelightHelpers.getTV(name);
    }

    @Override
    public String getName() {
        return name;
    }
}
