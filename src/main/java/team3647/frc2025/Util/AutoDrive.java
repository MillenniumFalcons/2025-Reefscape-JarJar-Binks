package team3647.frc2025.Util;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;

import static edu.wpi.first.units.Units.Inches;

import java.util.List;
import java.util.function.Supplier;
import team3647.frc2025.constants.FieldConstants.ScoringPos;
import team3647.frc2025.constants.SwerveDriveConstants;
import team3647.lib.team9442.AllianceObserver;

public class AutoDrive implements AllianceObserver {

    private final Supplier<Pose2d> odoPoseFunction;
    private final Supplier<ScoringPos> wantedScoringPos;

    private final PIDController xController;
    private final PIDController yController;
    private final PIDController rotController;

    private Alliance color = Alliance.Red;

    private DriveMode wantedMode = DriveMode.NONE;

    private List<Pose2d> sourcePoses;

	private final List<Pose2d> redSourcePoses;
	private final List<Pose2d> blueSourcePoses;

    public AutoDrive(
            Supplier<Pose2d> odoPoseFunction,
            Supplier<ScoringPos> wantedScoringPose,
            List<Pose2d> redSourcePoses,
			List<Pose2d> blueSourcePoses,
            PIDController xController,
            PIDController yController,
            PIDController rotController) {
        this.odoPoseFunction = odoPoseFunction;
        this.xController = xController;
        this.yController = yController;
        this.rotController = rotController;
        this.wantedScoringPos = wantedScoringPose;
        this.redSourcePoses = redSourcePoses;
		this.blueSourcePoses = blueSourcePoses;

		this.sourcePoses = color == Alliance.Red? redSourcePoses : blueSourcePoses;
    }

    public enum DriveMode {
        INTAKE,
        SRCINTAKE,
        SCORE,
        NONE
    }

    public Pose2d getPose() {
        return odoPoseFunction.get();
    }

    // TODO: ground intake implementatoin
    public double getX() {
        switch (wantedMode) {
            case SRCINTAKE:
                return xController.calculate(
                        getPose().getX(), getPose().nearest(sourcePoses).getX());
            case SCORE:
                return xController.calculate(getPose().getX(), wantedScoringPos.get().pose.getX());

            default:
                return xController.calculate(getPose().getX(), wantedScoringPos.get().pose.getX());
        }
    }

    public double getY() {
        switch (wantedMode) {
            case SRCINTAKE:
                return yController.calculate(
                        getPose().getY(), getPose().nearest(sourcePoses).getY());
            case SCORE:
                return yController.calculate(getPose().getY(), wantedScoringPos.get().pose.getY());

            default:
                return yController.calculate(getPose().getY(), wantedScoringPos.get().pose.getY());
        }
    }

    public double getRot() {
        switch (wantedMode) {
            case SRCINTAKE:
                return rotController.calculate(
                        getPose().getRotation().getRadians(),
                        getPose().nearest(sourcePoses).getRotation().getRadians());
            case SCORE:
                return rotController.calculate(
                        getPose().getRotation().getRadians(),
                        wantedScoringPos.get().pose.getRotation().getRadians());

            default:
                return rotController.calculate(
                        getPose()
                                .getRotation()
                                .minus(wantedScoringPos.get().pose.getRotation())
                                .getRadians());
        }
    }

    public Command pathFindToPose(Pose2d pose) {
        return AutoBuilder.pathfindToPose(
                pose,
                new PathConstraints(
                        SwerveDriveConstants.kDrivePossibleMaxSpeedMPS,
                        SwerveDriveConstants.kTeleopDriveMaxAccelUnitsPerSec,
                        SwerveDriveConstants.kRotPossibleMaxSpeedRadPerSec,
                        SwerveDriveConstants.kTeleopDriveMaxAngularAccelUnitsPerSec),
                0);
    }


	public boolean isAlignedToReef(){
		return PoseUtils.inCircle(getPose(), wantedScoringPos.get().pose, Inches.of(4));
	}


	public Twist2d getVelocities(){
		return new Twist2d(getX(), getY(), getRot());
	}

    @Override
    public void onAllianceFound(Alliance color) {
		sourcePoses = color == Alliance.Red? redSourcePoses : blueSourcePoses;
        this.color = color;
    }
}
