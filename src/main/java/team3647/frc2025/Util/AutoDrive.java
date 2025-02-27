package team3647.frc2025.Util;

import static edu.wpi.first.units.Units.Degree;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Radian;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import java.util.List;
import java.util.Map;
import java.util.function.Supplier;

import org.dyn4j.world.result.DetectResult;
import org.littletonrobotics.junction.Logger;
import team3647.frc2025.constants.FieldConstants.ScoringPos;
import team3647.frc2025.constants.SwerveDriveConstants;
import team3647.frc2025.subsystems.Superstructure.Branch;
import team3647.frc2025.subsystems.Superstructure.Side;
import team3647.lib.team6328.VirtualSubsystem;
import team3647.lib.team9442.AllianceObserver;
import team3647.lib.vision.NeuralDetector;

public class AutoDrive extends VirtualSubsystem implements AllianceObserver {

    private final Supplier<Pose2d> odoPoseFunction;
    private ScoringPos wantedScoringPos;

	private NeuralDetector detector;

    private Side wantedSide = Side.A;

    public final PIDController xController;
    public final PIDController yController;
    public final PIDController rotController;

    private Alliance color = Alliance.Red;

    private DriveMode wantedMode = DriveMode.NONE;

    private List<Pose2d> sourcePoses;

    private Branch wantedBranch = Branch.ONE;

    private Map<Pose2d, Side> poseToSideMap;

    private final List<Pose2d> redSourcePoses;
    private final List<Pose2d> blueSourcePoses;
    private final List<Pose2d> redSidePoses, blueSidePoses;
    private List<Pose2d> sidePoses;

    public boolean enabled = true;

    public AutoDrive(
            Supplier<Pose2d> odoPoseFunction,
            List<Pose2d> redSourcePoses,
            List<Pose2d> blueSourcePoses,
            PIDController xController,
            PIDController yController,
            PIDController rotController,
            List<Pose2d> redSidePoses,
            List<Pose2d> blueSidePoses,
            NeuralDetector detector) {
        super();
        this.odoPoseFunction = odoPoseFunction;
        this.xController = xController;
        this.yController = yController;
        this.rotController = rotController;
        this.wantedScoringPos = ScoringPos.NONE;
        this.redSourcePoses = redSourcePoses;
        this.blueSourcePoses = blueSourcePoses;
        this.redSidePoses = redSidePoses;
        this.blueSidePoses = blueSidePoses;
		this.detector = detector;

        this.sidePoses = color == Alliance.Red ? redSidePoses : blueSidePoses;

        this.sourcePoses = color == Alliance.Red ? redSourcePoses : blueSourcePoses;
        this.rotController.enableContinuousInput(-Math.PI, Math.PI);

        this.poseToSideMap =
                Map.of(
                        sidePoses.get(4),
                        Side.A,
                        sidePoses.get(3),
                        Side.B,
                        sidePoses.get(2),
                        Side.C,
                        sidePoses.get(1),
                        Side.D,
                        sidePoses.get(0),
                        Side.E,
                        sidePoses.get(5),
                        Side.F);
    }

    public enum DriveMode {
        INTAKE,
        SRCINTAKE,
        SCORE,
        TEST,
        ALGAE,
        NONE
    }

    public Pose2d getPose() {
        return odoPoseFunction.get();
    }

    public Command enableAutoDrive() {
        return Commands.runOnce(() -> this.enabled = true);
    }

    public Command disableAutoDrive() {
        return Commands.runOnce(() -> this.enabled = false);
    }

    public boolean getAutoDriveEnabled() {
        return enabled;
    }

    public Command setDriveMode(DriveMode mode) {
        return Commands.runOnce(() -> this.wantedMode = mode);
    }

    // TODO: ground intake implementatoin
    public double getX() {
        switch (wantedMode) {
            case SRCINTAKE:
                return xController.calculate(
                        getPose().getX(), getPose().nearest(sourcePoses).getX());
            case SCORE:
				
                var k = xController.calculate(
                        getPose().getX(), AllianceFlip.flip(wantedScoringPos.pose, color).getX());
						Logger.recordOutput("kx", k);
				return Math.abs(k) < 0.02? 0 : k;
			case INTAKE:

			return detector.getTY() >= 0? xController.calculate(detector.getTY()) : 0;

            default:
                return xController.calculate(
                        getPose().getX(), AllianceFlip.flip(wantedScoringPos.pose, color).getX());
        }
    }

    public double getY() {
        switch (wantedMode) {
            case SRCINTAKE:
                return yController.calculate(
                        getPose().getY(), getPose().nearest(sourcePoses).getY());
            case SCORE:
                var k = yController.calculate(
                        getPose().getY(), AllianceFlip.flip(wantedScoringPos.pose, color).getY());
						Logger.recordOutput("ky", k);
				return Math.abs(k) < 0.02? 0 : k;
			case INTAKE:
				return detector.getTY() >= 0? yController.calculate(detector.getTX()) : 0;

            default:
                return yController.calculate(
                        getPose().getY(), AllianceFlip.flip(wantedScoringPos.pose, color).getY());
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
                        AllianceFlip.flip(wantedScoringPos.pose, color).getRotation().getRadians());
            case TEST:
                Logger.recordOutput("error", rotController.getError());
                return rotController.calculate(getPose().getRotation().getRadians());
			case INTAKE:
			return detector.getTY() >= 0? rotController.calculate(Units.degreesToRadians(detector.getTX())) : 0;

            default:
                return rotController.calculate(
                        getPose()
                                .getRotation()
                                .minus(
                                        AllianceFlip.flip(wantedScoringPos.pose, color)
                                                .getRotation())
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

    public boolean isAlignedToReef() {
        return PoseUtils.inCircle(
                getPose(), AllianceFlip.flip(wantedScoringPos.pose, color), Inches.of(4));
    }

    public DriveMode getWantedMode() {
        return wantedMode;
    }

    public Twist2d getVelocities() {
        return new Twist2d(getX(), getY(), getRot());
    }

	public Command setWantedScoringPos(ScoringPos pos){
		return Commands.runOnce(() -> wantedScoringPos = pos);
	}

    @Override
    public void onAllianceFound(Alliance color) {

        this.color = color;

        sourcePoses = color == Alliance.Red ? redSourcePoses : blueSourcePoses;
        setwantedScoringPosBySideLevel();
        sidePoses = color == Alliance.Red ? redSidePoses : blueSidePoses;

        this.poseToSideMap =
                Map.of(
                        sidePoses.get(4),
                        Side.A,
                        sidePoses.get(3),
                        Side.B,
                        sidePoses.get(2),
                        Side.C,
                        sidePoses.get(1),
                        Side.D,
                        sidePoses.get(0),
                        Side.E,
                        sidePoses.get(5),
                        Side.F);
    }

    @Override
    public void periodic() {
        wantedSide = poseToSideMap.get(getPose().nearest(sidePoses));
        Logger.recordOutput("Robot/distance", ScoringPos.A1.pose.getX() - getPose().getX());
        // forscoring: 0.324 m away from the face
        setwantedScoringPosBySideLevel();
    }

    public void setwantedScoringPosBySideLevel() {
        switch (wantedSide) {
            case A:
                switch (wantedBranch) {
                    case ONE:
                        wantedScoringPos = ScoringPos.A1;
                        break;
                    case TWO:
                        wantedScoringPos = ScoringPos.A2;
                        break;
                    default:
                        DriverStation.reportError(
                                "Cannot set Wanted Scoring pos, no branch provided", false);
                        Logger.recordOutput(
                                "Robot/robotErrors",
                                "Cannot set Wanted Scoring pos, no branch provided"
                                        + Timer.getFPGATimestamp());
                        break;
                }
                break;
            case B:
                switch (wantedBranch) {
                    case ONE:
                        wantedScoringPos = ScoringPos.B1;
                        break;
                    case TWO:
                        wantedScoringPos = ScoringPos.B2;
                        break;
                    default:
                        DriverStation.reportError(
                                "Cannot set Wanted Scoring pos, no branch provided", false);
                        Logger.recordOutput(
                                "Robot/robotErrors",
                                "Cannot set Wanted Scoring pos, no branch provided"
                                        + Timer.getFPGATimestamp());
                        break;
                }
                break;
            case C:
                switch (wantedBranch) {
                    case ONE:
                        wantedScoringPos = ScoringPos.C1;
                        break;
                    case TWO:
                        wantedScoringPos = ScoringPos.C2;
                        break;
                    default:
                        DriverStation.reportError(
                                "Cannot set Wanted Scoring pos, no branch provided", false);
                        Logger.recordOutput(
                                "Robot/robotErrors",
                                "Cannot set Wanted Scoring pos, no branch provided"
                                        + Timer.getFPGATimestamp());
                        break;
                }
                break;
            case D:
                switch (wantedBranch) {
                    case ONE:
                        wantedScoringPos = ScoringPos.D1;
                        break;
                    case TWO:
                        wantedScoringPos = ScoringPos.D2;
                        break;
                    default:
                        DriverStation.reportError(
                                "Cannot set Wanted Scoring pos, no branch provided", false);
                        Logger.recordOutput(
                                "Robot/robotErrors",
                                "Cannot set Wanted Scoring pos, no branch provided"
                                        + Timer.getFPGATimestamp());
                        break;
                }
                break;
            case E:
                switch (wantedBranch) {
                    case ONE:
                        wantedScoringPos = ScoringPos.E1;
                        break;
                    case TWO:
                        wantedScoringPos = ScoringPos.E2;
                        break;
                    default:
                        DriverStation.reportError(
                                "Cannot set Wanted Scoring pos, no branch provided", false);
                        Logger.recordOutput(
                                "Robot/robotErrors",
                                "Cannot set Wanted Scoring pos, no branch provided"
                                        + Timer.getFPGATimestamp());
                        break;
                }
                break;
            case F:
                switch (wantedBranch) {
                    case ONE:
                        wantedScoringPos = ScoringPos.F1;
                        break;
                    case TWO:
                        wantedScoringPos = ScoringPos.F2;
                        break;
                    default:
                        DriverStation.reportError(
                                "Cannot set Wanted Scoring pos, no branch provided", false);
                        Logger.recordOutput(
                                "Robot/robotErrors",
                                "Cannot set Wanted Scoring pos, no branch provided"
                                        + Timer.getFPGATimestamp());
                        break;
                }
                break;
            default:
                wantedScoringPos = ScoringPos.NONE;
                DriverStation.reportError("Cannot set Wanted scoring pos, no SIDE provided", false);
                Logger.recordOutput(
                        "Robot/robotErrors", "Cannot set Wanted Scoring pos, no SIDE provided");
                break;
        }
    }

    public Command setWantedBranch(Branch wantedBranch) {
        return Commands.runOnce(
                () -> {
                    this.wantedBranch = wantedBranch;
                });
    }

	public Pose2d getAlignPose(){
		if(wantedMode == DriveMode.ALGAE){
			return getPose().nearest(sidePoses);
		}

		return AllianceFlip.flip(wantedScoringPos.pose, color);
	}
}
