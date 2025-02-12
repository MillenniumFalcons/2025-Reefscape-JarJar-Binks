package team3647.frc2025.Util;

import static edu.wpi.first.units.Units.Degree;
import static edu.wpi.first.units.Units.Meters;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import team3647.frc2025.constants.ElevatorConstants;
import team3647.frc2025.constants.PivotConstants;

public class SuperstructureState {
    public Angle pivotAngle;
    public Distance elevatorHeight;
	public static SuperstructureState kInvalidState = new SuperstructureState(Degree.of(-100), Meters.of(-1));
	public static SuperstructureState troughScore = new SuperstructureState(PivotConstants.kLevel1Angle, ElevatorConstants.kLevel1Height);
	public static SuperstructureState lowScore = new SuperstructureState(PivotConstants.kLevel2Angle, ElevatorConstants.kLevel2Height);
	public static SuperstructureState midScore = new SuperstructureState(PivotConstants.kLevel3Angle, ElevatorConstants.kLevel3Height);
	public static SuperstructureState highScore = new SuperstructureState(PivotConstants.kLevel4Angle, ElevatorConstants.kLevel4Height);

    public SuperstructureState(Angle pivotAngle, Distance elevatorHeight) {
        this.pivotAngle = pivotAngle;
        this.elevatorHeight = elevatorHeight;

    }

    public SuperstructureState withPivotAngle(Angle pivotAngle) {
        return new SuperstructureState(pivotAngle, this.elevatorHeight);
    }

    public SuperstructureState withElevatorHeight(Distance elevatorHeight) {
        return new SuperstructureState(this.pivotAngle, elevatorHeight);
    }



	@Override
	public boolean equals(Object obj) {
		if (!(obj instanceof SuperstructureState)) {
			return false;
		}

		return this.elevatorHeight.equals(((SuperstructureState)obj).elevatorHeight)
			&& this.pivotAngle.equals(((SuperstructureState)obj).pivotAngle);
	}
}
