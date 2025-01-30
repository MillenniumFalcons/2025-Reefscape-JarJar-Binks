package team3647.frc2025.Util;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;

public class SuperstructureState {
    public Angle pivotAngle;
    public Distance elevatorHeight;
    public double rollers;

    public SuperstructureState(Angle pivotAngle, Distance elevatorHeight, double rollers) {
        this.pivotAngle = pivotAngle;
        this.elevatorHeight = elevatorHeight;
        this.rollers = rollers;
    }

    public SuperstructureState withPivotAngle(Angle pivotAngle) {
        return new SuperstructureState(pivotAngle, this.elevatorHeight, this.rollers);
    }

    public SuperstructureState withElevatorHeight(Distance elevatorHeight) {
        return new SuperstructureState(this.pivotAngle, elevatorHeight, this.rollers);
    }

    public SuperstructureState withRollerSpeed(double rollers) {
        return new SuperstructureState(this.pivotAngle, this.elevatorHeight, rollers);
    }
}
