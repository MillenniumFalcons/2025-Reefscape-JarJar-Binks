package team3647.frc2025.Util;


import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meter;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Radian;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Dimensionless;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.DriverStation;

public class InverseKinematics {

	public static Distance armLength = Units.Inches.of(25.500).plus(Inches.of(2));
	

	public static SuperstructureState getIK(Translation2d pose){
		var x = pose.getMeasureX();
		var y = pose.getMeasureY();

		if(x.in(Meters) < 0){
			DriverStation.reportError("Tried to set negative x for IK value!!!", false);
			return SuperstructureState.kInvalidState;
		}

		var armAngle = Radian.of(Math.acos(x.div(armLength).in(Units.Value)));

		var armAddedHeight = Math.sin(armAngle.in(Radian)) * armLength.in(Meter);

		var elevatorHeight = y.in(Meter) - armAddedHeight;

		return new SuperstructureState(armAngle, Meters.of(elevatorHeight));
	}


}
