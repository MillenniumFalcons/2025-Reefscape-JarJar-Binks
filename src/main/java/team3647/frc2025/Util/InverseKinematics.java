package team3647.frc2025.Util;

import static edu.wpi.first.units.Units.Degree;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meter;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Radian;

import edu.wpi.first.math.MatBuilder;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rectangle2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import org.littletonrobotics.junction.Logger;
import team3647.frc2025.constants.ElevatorConstants;

public class InverseKinematics {
	public static final double kNativeToMeters = (1.21 - 0.85) / 19.855;
    public static final Distance kMaxHeight = Meters.of(84.27 * kNativeToMeters);
	public static final Distance kMinHeight = Meters.of(0.85);

    public static final Angle WristIdrc = Degree.of(-100);


    public static Distance armLenOffset = Inches.of(1);

    public static double kWristStartingAngleDeg = 118.739;

    public static Distance armLength =
            Units.Centimeter.of(71)
                    .plus(armLenOffset); // Units.Inches.of(25.500).plus(Inches.of(2));
    public static Distance elevatorXOffset = Inches.of(4.875);

    public static Translation2d startingWristTopPos =
            new Translation2d(Inches.of(4.594), Inches.of(20.489));

	public static Rectangle2d rect = 
				new Rectangle2d(new Pose2d(Inches.of(18.43), Inches.of(13.49), Rotation2d.kZero), Inches.of(20), Inches.of(14));
	
	private static SuperstructureState minState = 
		SuperstructureState.TroughScore
			.withPivotAngle(SuperstructureState.TroughScore.pivotAngle.minus(Degree.of(3)));

    // the highest tip of the wirst, make it a variable function
    public static Translation2d wristTopPos(SuperstructureState currentState) {
        // real top pos at stow angle
        return startingWristTopPos.rotateBy(
                Rotation2d.fromDegrees(
                        (kWristStartingAngleDeg - currentState.wristAngle.in(Degree))));
    }

	@Deprecated
	/**
	 * WARNING: DOESN'T WORK, DON'T USE
	 * @param wantedState
	 * @return
	 */
    public static SuperstructureState getMinClearStateGoingUP(SuperstructureState wantedState) {
        var trans = forwardKinematics(wantedState);
        var clearheightTotal = wristTopPos(wantedState).getY();
		Logger.recordOutput("IKTEST/clearHeightTotal", (clearheightTotal - armLength.in(Meters)));

        var state = getIK(new Translation2d(trans.getX(), clearheightTotal)).withWristAngle(Degree.of(20));

		Logger.recordOutput("IKSTATE/wrist", state.wristAngle.in(Degree));
		Logger.recordOutput("IKSTATE/pivot", state.pivotAngle.in(Degree));
		Logger.recordOutput("IKSTATE/elev", state.elevatorHeight.in(Meter));

        return state;
    }

    public static boolean shouldClear(SuperstructureState currentState) {
        return currentState.pivotAngle.gte(Radian.of(-0.7))
                && currentState.elevatorHeight.lte(ElevatorConstants.kClearHeight);
    }

    private static Translation2d minPos(SuperstructureState currentState) {
        // should pivot around wrist
        // use MatBuilder.fill(N2.instance,N1.instance, minpos.x,
        // minpos.y).times(getRotationMatrix(normalize(currentState.wristAngle)));


        Translation2d minPos = forwardKinematics(minState);

        return minPos;
    }

    public static Matrix<N2, N2> getRotationMatrix(Angle theta) {

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

        var elevheight = MathUtil.clamp(y.in(Meters), kMinHeight.in(Meters), kMaxHeight.in(Meters));

		var leftOverHeight = elevheight - y.in(Meters);

		leftOverHeight = MathUtil.clamp(leftOverHeight, 0, armLength.in(Meters));
		
		var theta = Radian.of(Math.asin(leftOverHeight/armLength.in(Meters)));

		return new SuperstructureState(theta, Meter.of(elevheight), WristIdrc);
    }

    public static Translation2d forwardKinematics(SuperstructureState state) {
        var x = armLength.in(Meters) * Math.cos(state.pivotAngle.in(Radian));
        var y =
                state.elevatorHeight.in(Meters)
                        + (armLength.in(Meters) * Math.sin(state.pivotAngle.in(Radian)));

        return new Translation2d(x, y);
    }

    public static Angle getMinAngle(SuperstructureState currentState) {
        if (!shouldClear(currentState)) {
            var armAngle = Radian.of(-Math.PI / 2.0);
            Logger.recordOutput("calculated min angle", armAngle);
            return armAngle;
        }
        var minPos = minPos(currentState);
    

        var phi = minState.pivotAngle;
        var dx = armLength.in(Meters) * Math.sin(phi.in(Radian));

        var elev = currentState.elevatorHeight;
        var dy = elev.in(Meters) - minPos.getY();

        // Logger.recordOutput("dy", dy);
        // Logger.recordOutput("dx", dx);

        var armAngle = Radian.of(Math.atan(dx / dy));

        Logger.recordOutput("calculated min angle", armAngle);

        return armAngle;
    }

	public static SuperstructureState lookAheadEstimateUp(SuperstructureState currenState){
		return currenState
				.withElevatorHeight(currenState.elevatorHeight.plus(Inches.of(5)))
				.withPivotAngle(currenState.pivotAngle.plus(Degree.of(5)));
				
	}

	public static SuperstructureState lookAheadEstimateDown(SuperstructureState currenState){
		return currenState
				.withElevatorHeight(currenState.elevatorHeight.minus(Inches.of(8)))
				.withPivotAngle(currenState.pivotAngle.minus(Degree.of(8)));
				
	}

	public static boolean shouldWristOutOftheway(SuperstructureState currentState){
		var newState = lookAheadEstimateUp(currentState);
		var trans = forwardKinematics(newState);
		Logger.recordOutput("IK/trans/x", trans.getMeasureX().in(Inches));
		Logger.recordOutput("IK/trans/y", trans.getMeasureY().in(Inches));
		
		var inRect = PoseUtils.inRect(trans, rect);
		return inRect;
	}

	public static Angle getWristOutofTheWayMaxAngle(SuperstructureState currentState, Angle wristStowAngle){
		var newState = lookAheadEstimateUp(currentState);
		var trans = forwardKinematics(newState);
		
		var inRect = PoseUtils.inRect(trans, rect);
		
		return inRect? Degree.of(30) : wristStowAngle;

		
	}




}
