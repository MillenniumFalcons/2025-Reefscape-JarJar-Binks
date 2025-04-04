package team3647.frc2025.Util;

import static edu.wpi.first.units.Units.Degree;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Radian;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import team3647.frc2025.commands.PivotCommands;
import team3647.frc2025.constants.ElevatorConstants;
import team3647.frc2025.constants.PivotConstants;
import team3647.frc2025.constants.WristConstants;

public class SuperstructureState {
    public double pivotAngleRads;
    public double elevatorHeightM;
    public double wristAngleDegs;

    public static SuperstructureState kInvalidState =
            new SuperstructureState(Degree.of(-100), Meters.of(-1), Degree.of(-100));
    public static SuperstructureState TroughScore =
            new SuperstructureState(
                    PivotConstants.kLevel1Angle,
                    ElevatorConstants.kLowScoreHeight,
                    WristConstants.kStowAngle);
    public static SuperstructureState LowScore =
            new SuperstructureState(
                    PivotConstants.kLevel2Angle,
                    ElevatorConstants.kLowScoreHeight,
                    WristConstants.kStowAngle);
    public static SuperstructureState MidScore =
            new SuperstructureState(
                    PivotConstants.kLevel3Angle,
                    ElevatorConstants.kLowScoreHeight,
                    WristConstants.kStowAngle);
    public static SuperstructureState HighScore =
            new SuperstructureState(
                    PivotConstants.kLevel4Angle,
                    ElevatorConstants.kLevel4Height,
                    WristConstants.kStowAngle);

	public static SuperstructureState L4Stow = 
			new SuperstructureState(
				PivotConstants.klowLevelsStow, 
				ElevatorConstants.kLevel4Height, 
				WristConstants.kStowAngle);

    // gg more algae stuff
    public static SuperstructureState HighAlgae =
            new SuperstructureState(
                    PivotConstants.kAlgaeAngleHigh,
                    ElevatorConstants.kHighAlgaeHeight.plus(Inches.of(10)),
                    WristConstants.kStowAngle);
    public static SuperstructureState LowAlgae =
            new SuperstructureState(
                    PivotConstants.kAlgaeAngleLow,
                    ElevatorConstants.kStowHeight,
                    WristConstants.kStowAngle);

    public static SuperstructureState AlgaeStow =
            new SuperstructureState(
                    PivotConstants.kMaxAngle.in(Radian), LowScore.elevatorHeightM, WristConstants.kStowAngle.in(Degree));

    public static SuperstructureState AlgaeBarge =
            new SuperstructureState(
                    PivotConstants.kMaxAngle.minus(Degree.of(10)),
                    ElevatorConstants.kMaxHeight,
                    WristConstants.kStowAngle);

    // end algae stuff

    public static SuperstructureState Stow =
            new SuperstructureState(
                    PivotConstants.kStowAngle,
                    ElevatorConstants.kStowHeight,
                    WristConstants.kStowAngle);

    public static SuperstructureState ToStow =
            new SuperstructureState(
                    PivotConstants.kStowAngle,
                    ElevatorConstants.kClearHeight,
                    WristConstants.kStowAngle);

    public static SuperstructureState StowScore =
            new SuperstructureState(
                    InverseKinematics.getMinAngle(LowScore),
                    ElevatorConstants.kLowScoreHeight,
                    WristConstants.kStowAngle);

    public static SuperstructureState Intake =
            new SuperstructureState(
                    PivotConstants.kStowAngle,
                    ElevatorConstants.kHandoffHeight,
                    WristConstants.kIntakeAngle);

    public static SuperstructureState Transfer =
            new SuperstructureState(
                    PivotConstants.kStowAngle,
                    ElevatorConstants.kHandoffHeight,
                    WristConstants.kHandoffAngle);

    public static SuperstructureState Handoff =
            new SuperstructureState(
                    PivotConstants.kStowAngle,
                    ElevatorConstants.kStartingHeight,
                    WristConstants.kStowWithPiece);

    public SuperstructureState(Angle pivotAngle, Distance elevatorHeight, Angle wristAngle) {
        this.pivotAngleRads = pivotAngle.in(Radian);
        this.elevatorHeightM = elevatorHeight.in(Meters);
        this.wristAngleDegs = wristAngle.in(Degree);
    }

	public SuperstructureState(double pivotAngleRads, double elevatorHightM, double wrsitAngleDegs){
		this.pivotAngleRads = pivotAngleRads;
		this.elevatorHeightM = elevatorHightM;
		this.wristAngleDegs = wrsitAngleDegs;
	}

    public SuperstructureState withPivotAngle(Angle pivotAngle) {
        return new SuperstructureState(pivotAngle.in(Radian), this.elevatorHeightM, this.wristAngleDegs);
    }
	public SuperstructureState withPivotAngle(double pivotAngle) {
        return new SuperstructureState(pivotAngle, this.elevatorHeightM, this.wristAngleDegs);
    }

    public SuperstructureState withElevatorHeight(Distance elevatorHeight) {
        return new SuperstructureState(this.pivotAngleRads, elevatorHeight.in(Meters), this.wristAngleDegs);
    }

    public SuperstructureState withWristAngle(Angle wristAngle) {
        return new SuperstructureState(this.pivotAngleRads, this.elevatorHeightM, wristAngle.in(Degree));
    }

    @Override
    public boolean equals(Object obj) {
        if (!(obj instanceof SuperstructureState)) {
            return false;
        }

        return this.elevatorHeightM == ((SuperstructureState) obj).elevatorHeightM
                && this.pivotAngleRads == ((SuperstructureState) obj).pivotAngleRads;
    }
}
