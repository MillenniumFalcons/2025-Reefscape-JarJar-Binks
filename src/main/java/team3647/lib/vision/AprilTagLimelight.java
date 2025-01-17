package team3647.lib.vision;

import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.Timer;

import static edu.wpi.first.units.Units.Degree;
import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Millisecond;
import static edu.wpi.first.units.Units.Second;

import java.util.List;
import java.util.Optional;
import java.util.function.Supplier;

import team3647.frc2025.constants.FieldConstants;
import team3647.lib.team6328.VirtualSubsystem;
import team3647.lib.vision.LimelightHelpers.RawFiducial;

public class AprilTagLimelight extends VirtualSubsystem implements AprilTagCamera {
	
	public String name;
	public Transform3d robotToCamera;

	Supplier<Orientation> orientation;

	private final Vector<N3> baseStdDevs;

	public AprilTagLimelight(String name, Transform3d robotToCamera, Supplier<Orientation> orientationSupplier, Vector<N3> baseStdDevs){
		super();
		this.name = name;
		this.robotToCamera = robotToCamera;
		this.orientation = orientationSupplier;
		LimelightHelpers.setCameraPose_RobotSpace(
			name, 
			robotToCamera.getX(),
			robotToCamera.getY(), 
			robotToCamera.getX(), 
			robotToCamera.getRotation().getX(), 
			robotToCamera.getRotation().getY(), 
			robotToCamera.getRotation().getZ());

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


	@Override
	public void periodic() {
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


	public Distance getBestTagDist(RawFiducial[] fiducials){
		double lowest = Double.POSITIVE_INFINITY;
		for (RawFiducial rawFiducial : fiducials) {
			if (rawFiducial.distToCamera < lowest) {
				lowest = rawFiducial.distToCamera;
			}
		}

		return Meters.of(lowest);
	}

	public RawFiducial getBestTag(RawFiducial[] fiducials){
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
	public Optional<List<VisionMeasurement>> QueueToInputs() {
		var botPose = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(name);
		if (botPose.isEmpty()) {
			return Optional.empty();
		}
		var result = botPose.get();
		if (result.tagCount == 0) {
			return Optional.empty();
		}

		var bestTag = getBestTag(result.rawFiducials);
		
		if(bestTag.distToCamera > 4){
			return Optional.empty();
		}

		if (result.pose.getMeasureX().gt(FieldConstants.kFieldLength)
					|| result.pose.getX() < 0
					|| result.pose.getMeasureY().gt(FieldConstants.kFieldWidth) 
					|| result.pose.getY() < 0) {
				return Optional.empty();
			}

		var stdDevs = baseStdDevs.times(result.avgTagDist/ Math.pow(result.tagCount, 3)).times(result.isMegaTag2? 1/8 : 1 );

		var ambiguityScore = 1/(result.tagCount * 100 + (1-bestTag.ambiguity));

		double timestamp = Timer.getFPGATimestamp() - Millisecond.of(result.latency).in(Second);

		var measurement = new VisionMeasurement(result.pose, timestamp , ambiguityScore, stdDevs, name);

		return Optional.of(List.of(measurement));

	}

	@Override
	public int getTagNum() {
		var botPose = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(name);

		if(botPose.isEmpty()){
			return -1;
		}
		var bestTag = getBestTag(botPose.get().rawFiducials);

		return bestTag.id;
	}




	@Override
	public String getName() {
		return name;
	}
}
