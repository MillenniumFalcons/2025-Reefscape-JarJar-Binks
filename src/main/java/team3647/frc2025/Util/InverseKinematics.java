package team3647.frc2025.Util;

import static edu.wpi.first.units.Units.Centimeter;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meter;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Radian;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.DriverStation;

public class InverseKinematics {

    public static Distance armLenOffset = Inches.of(1);
    public static Distance armLength =
            Units.Centimeter.of(71)
                    .plus(armLenOffset); // Units.Inches.of(25.500).plus(Inches.of(2));
    public static Distance elevatorXOffset = Inches.of(4.875);

    public static SuperstructureState getIK(Translation2d pose) {
        var x = pose.getMeasureX();
        var y = pose.getMeasureY();

        if (x.in(Meters) < 0) {
            DriverStation.reportError("Tried to set negative x for IK value!!!", false);
            return SuperstructureState.kInvalidState;
        }

        var armAngle = Radian.of(Math.acos(x.div(armLength).in(Units.Value)));

        var armAddedHeight = Math.sin(armAngle.in(Radian)) * armLength.in(Meter);

        var elevatorHeight = y.in(Meter) - armAddedHeight;

        return new SuperstructureState(armAngle, Meters.of(elevatorHeight));
    }

    public static boolean getClawXYBad(SuperstructureState state) {
        var y =
                state.elevatorHeight.in(Meter)
                        + (armLength.in(Meters) * Math.sin(state.pivotAngle.in(Radian)));
        var x =
                armLength.in(Meters) * Math.cos(state.pivotAngle.in(Radian))
                        + elevatorXOffset.in(Meters);
        var trans = new Translation2d(x, y);

        return PoseUtils.inRect(
                trans,
                elevatorXOffset.in(Meters),
                elevatorXOffset.plus(Inches.of(8.625)).in(Meters),
                0,
                Centimeter.of(58.5).in(Meters));
    }
}
