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
    public static SuperstructureState troughScore =
            new SuperstructureState(
                    PivotConstants.kLevel1Angle,
                    ElevatorConstants.kLevel1Height,
                    WristConstants.kStowAngle);
    public static SuperstructureState lowScore =
            new SuperstructureState(
                    PivotConstants.kLevel2Angle,
                    ElevatorConstants.kLevel2Height,
                    WristConstants.kStowAngle);
    public static SuperstructureState midScore =
            new SuperstructureState(
                    PivotConstants.kLevel3Angle,
                    ElevatorConstants.kLevel3Height,
                    WristConstants.kStowAngle);
    public static SuperstructureState highScore =
            new SuperstructureState(
                    PivotConstants.kLevel4Angle,
                    ElevatorConstants.kLevel4Height,
                    WristConstants.kStowAngle);

    public static SuperstructureState stow =
            new SuperstructureState(
                    PivotConstants.kStowAngle,
                    ElevatorConstants.kStowHeight,
                    WristConstants.kStowAngle);

    public static SuperstructureState toStow =
            new SuperstructureState(
                    PivotConstants.kStowAngle,
                    ElevatorConstants.kClearHeight,
                    WristConstants.kStowAngle);

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

    @Override
    public boolean equals(Object obj) {
        if (!(obj instanceof SuperstructureState)) {
            return false;
        }

        return this.elevatorHeight.equals(((SuperstructureState) obj).elevatorHeight)
                && this.pivotAngle.equals(((SuperstructureState) obj).pivotAngle);
    }
}
