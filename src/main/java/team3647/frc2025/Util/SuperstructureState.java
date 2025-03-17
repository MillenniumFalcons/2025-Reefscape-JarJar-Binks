package team3647.frc2025.Util;

import static edu.wpi.first.units.Units.Degree;
import static edu.wpi.first.units.Units.Meters;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import team3647.frc2025.constants.ElevatorConstants;
import team3647.frc2025.constants.PivotConstants;
import team3647.frc2025.constants.WristConstants;

public class SuperstructureState {
    public Angle pivotAngle;
    public Distance elevatorHeight;
    public Angle wristAngle;

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

        //gg more algae stuff
        public static SuperstructureState HighAlgae = 
                new SuperstructureState(
                        PivotConstants.kAlgaeAngleHigh, 
                        ElevatorConstants.kHighAlgaeHeight, 
                        WristConstants.kStowAngle);
        //end algae stuff

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
        this.pivotAngle = pivotAngle;
        this.elevatorHeight = elevatorHeight;
        this.wristAngle = wristAngle;
    }

    public SuperstructureState withPivotAngle(Angle pivotAngle) {
        return new SuperstructureState(pivotAngle, this.elevatorHeight, this.wristAngle);
    }

    public SuperstructureState withElevatorHeight(Distance elevatorHeight) {
        return new SuperstructureState(this.pivotAngle, elevatorHeight, this.wristAngle);
    }

	public SuperstructureState withWristAngle(Angle wristAngle) {
        return new SuperstructureState(this.pivotAngle, this.elevatorHeight, wristAngle);
    }

    @Override
    public boolean equals(Object obj) {
        if (!(obj instanceof SuperstructureState)) {
            return false;
        }

        return this.elevatorHeight.equals(((SuperstructureState) obj).elevatorHeight)
                && this.pivotAngle.equals(((SuperstructureState) obj).pivotAngle);
    }
}
