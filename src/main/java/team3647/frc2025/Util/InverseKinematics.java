package team3647.frc2025.Util;

import static edu.wpi.first.units.Units.Centimeter;
import static edu.wpi.first.units.Units.Degree;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meter;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Radian;

import edu.wpi.first.math.MatBuilder;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.DriverStation;
import org.littletonrobotics.junction.Logger;
import team3647.frc2025.constants.ElevatorConstants;
import team3647.frc2025.constants.PivotConstants;
import team3647.frc2025.constants.WristConstants;

public class InverseKinematics {

    public static Distance armLenOffset = Inches.of(1);
    public static Distance armLength =
            Units.Centimeter.of(71)
                    .plus(armLenOffset); // Units.Inches.of(25.500).plus(Inches.of(2));
    public static Distance elevatorXOffset = Inches.of(4.875);

    // the highest tip of the wirst, make it a variable function
    private static Translation2d wristTopPos() {
        // real top pos at stow angle
        return new Translation2d(Inches.of(8.006), Inches.of(20.980));
    }

    public static boolean shouldClear(SuperstructureState currentState) {
        return currentState.pivotAngle.gt(Radian.of(-0.7))
                && currentState.elevatorHeight.lt(ElevatorConstants.kClearHeight);
    }

    private static Translation2d minPos(SuperstructureState currentState) {
        // should pivot around wrist
        // use vecbuilder.fill().times(getRotationMatrix(currentState.wristAngle))
        SuperstructureState minState =
                new SuperstructureState(
                        PivotConstants.kLevel1Angle.minus(Degree.of(1)),
                        ElevatorConstants.kLevel1Height,
                        WristConstants.idrc);

        Translation2d minPos = forwardKinematics(minState);

        return minPos;
    }

    public Matrix<N2, N2> getRotationMatrix(Angle theta) {
        return MatBuilder.fill(
                N2.instance,
                N2.instance,
                Math.cos(theta.in(Radian)),
                -Math.sin(theta.in(Radian)),
                Math.sin(theta.in(Radian)),
                Math.cos(theta.in(Radian)));
    }

    public static SuperstructureState getIK(Translation2d pose) {
        var x = pose.getMeasureX();
        var y = pose.getMeasureY();

        if (x.in(Meters) < 0) {
            DriverStation.reportError("Tried to set negative x for IK value!!!", false);
            return SuperstructureState.kInvalidState;
        }

        var armAngle = Radian.of(Math.asin(x.div(armLength).in(Units.Value)));

        var armAddedHeight = Math.cos(armAngle.in(Radian)) * armLength.in(Meter);

        var elevatorHeight = y.in(Meter) + armAddedHeight;

        return new SuperstructureState(
                armAngle, Meters.of(elevatorHeight), WristConstants.kStowAngle);
    }

    public static Translation2d forwardKinematics(SuperstructureState state) {
        var x = armLength.in(Meters) * Math.sin(state.pivotAngle.in(Radian));
        var y =
                state.elevatorHeight.in(Meters)
                        + (armLength.in(Meters) * Math.cos(state.pivotAngle.in(Radian)));

        return new Translation2d(x, y);
    }

    public static Angle getMinAngle(SuperstructureState currentState) {
        if (!shouldClear(currentState)) {
            var armAngle = Radian.of(-Math.PI / 2.0);
            Logger.recordOutput("calculated min angle", armAngle);
            return armAngle;
        }
        var minPos = minPos(currentState);
        var minState = getIK(minPos);

        var phi = minState.pivotAngle.unaryMinus();
        var dx = armLength.in(Meters) * Math.sin(phi.in(Radian));

        var elev = currentState.elevatorHeight;
        var dy = elev.in(Meters) - minPos.getY();

        // Logger.recordOutput("dy", dy);
        // Logger.recordOutput("dx", dx);

        var armAngle = Radian.of(Math.atan(dx / dy));

        Logger.recordOutput("calculated min angle", armAngle);

        return armAngle;
    }

    public void getWristOutofthewayAngle() {}

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
