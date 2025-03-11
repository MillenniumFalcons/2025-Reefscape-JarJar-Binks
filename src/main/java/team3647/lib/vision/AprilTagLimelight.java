package team3647.lib.vision;

import static edu.wpi.first.units.Units.Degree;
import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Millisecond;
import static edu.wpi.first.units.Units.Second;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.DriverStation;

import edu.wpi.first.wpilibj.Timer;

import java.lang.StackWalker.Option;
import java.util.Optional;
import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix.motorcontrol.IFollower;

import team3647.frc2025.constants.FieldConstants;
import team3647.lib.team6328.VirtualSubsystem;
import team3647.lib.vision.LimelightHelpers.PoseEstimate;
import team3647.lib.vision.LimelightHelpers.RawFiducial;
import team3647.lib.vision.old.Limelight;

public class AprilTagLimelight extends VirtualSubsystem implements AprilTagCamera {

    public final String name;
    public final Transform3d robotToCamera;

    private final int kAprilTagPipelineIndex = 0;

    private final Supplier<Orientation> orientation;

	public final PoseEstimate kEmpty = new PoseEstimate(
		Pose2d.kZero, 
		-1, 
		-1, 
		-1, 
		-1, 
		-1, 
		-1, 
		new RawFiducial[] { new RawFiducial(-1, -1, -1, -1, -1, -1, 9999999)}, 
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
        // LimelightHelpers.setCameraPose_RobotSpace(
        //         name,
        //         robotToCamera.getX(),
        //         robotToCamera.getY(),
        //         robotToCamera.getX(),
        //         robotToCamera.getRotation().getX(),
        //         robotToCamera.getRotation().getY(),
        //         robotToCamera.getRotation().getZ());
        LimelightHelpers.setPipelineIndex(name, kAprilTagPipelineIndex);
		setIMUMode(4);
      

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

	public void setOrientation(){
		var angle = orientation.get();
        LimelightHelpers.SetRobotOrientation(
                name,
                angle.yaw.in(Degree),
                angle.yawRate.in(DegreesPerSecond),
                angle.pitch.in(Degree),
                angle.pitchRate.in(DegreesPerSecond),
                angle.roll.in(Degree),
                angle.rollRate.in(DegreesPerSecond));
	}

    @Override
    public void periodic() {
       setOrientation();
		if (DriverStation.isEnabled()) {
			LimelightHelpers.cancelThrottle(name);
			// setIMUMode(1);
		}
		if (DriverStation.isDisabled()) {
			LimelightHelpers.setThrottle(name);
			// setIMUMode(2);
		}
		setIMUMode(4);

		Logger.recordOutput("td", getTd());
		Logger.recordOutput("tv", LimelightHelpers.getTV(name));

		getDX().ifPresentOrElse(pose -> Logger.recordOutput("name", pose), () -> Logger.recordOutput("name", Pose2d.kZero));
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

	
	public Optional<Pose2d> getDX(){
		var estimate =  LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(name);
	
		if (estimate.isEmpty()) return Optional.empty();

		return Optional.of(estimate.get().pose);
	}

	
	public double getDY(){
		var estimate =  LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(name);
		if (estimate.isEmpty()) return 0;

		var bestTagID = getBestTag(estimate.get().rawFiducials).id;
		return aprilTagFieldLayout.getTagPose(bestTagID).get().getY() - estimate.get().pose.getY();
	}

    @Override
    public Optional<VisionMeasurement> QueueToInputs() {
		setOrientation();
        var botPose = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(name);
        if (botPose.isEmpty()) {
            return Optional.empty();
        }
        var result = botPose.get();
        if (result.tagCount == 0) {
            return Optional.empty();
        }

        var bestTag = getBestTag(result.rawFiducials);

        if (bestTag.distToCamera > 4) {
            return Optional.empty();
        }

        if (result.pose.getMeasureX().gt(FieldConstants.kFieldLength)
                || result.pose.getX() < 0
                || result.pose.getMeasureY().gt(FieldConstants.kFieldWidth)
                || result.pose.getY() < 0) {
            return Optional.empty();
        }

        var stdDevs =
                baseStdDevs
                        .times(result.avgTagDist / Math.pow(result.tagCount, 3))
                        .times(result.isMegaTag2 ? 1 / 8 : 1);

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
        return LimelightHelpers.getTX(name);
    }

    @Override
    public double getTy() {
        return LimelightHelpers.getTY(name);
    }

    @Override
    public double getTa() {
        return LimelightHelpers.getTA(name);
    }

	public double getTd(){
		var allTags = LimelightHelpers.getRawFiducials(name);
		Logger.recordOutput("tages???", allTags.length);
		if(allTags.length < 1) return -1;
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
