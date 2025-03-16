package team3647.frc2025.robot;

import static edu.wpi.first.units.Units.Degree;
import static edu.wpi.first.units.Units.Inches;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Pose2d;
import team3647.frc2025.Util.AutoDrive;
import team3647.frc2025.Util.InverseKinematics;
import team3647.frc2025.Util.SuperstructureState;
import team3647.frc2025.constants.WristConstants;
import team3647.frc2025.subsystems.Superstructure;
import team3647.lib.team6328.VirtualSubsystem;

public class RobotTracker extends VirtualSubsystem {
	private final Superstructure superstructure;
	private final AutoDrive autoDrive;
	private SuperstructureState currentState = SuperstructureState.kInvalidState;

	public RobotTracker(Superstructure superstructure, AutoDrive autoDrive) {
		super();
		this.superstructure = superstructure;
		this.autoDrive = autoDrive;
	}

	@Override
	public void periodic() {
		Logger.recordOutput(
				"Superstructure/pivotoffset", superstructure.getPivotOffset());
		Logger.recordOutput(
				"Superstructure/elevOffset", superstructure.getElevOffset());
		Logger.recordOutput(
				"Superstructure/Level", superstructure.getWantedLevel());

		Logger.recordOutput("Robot/drive mode", autoDrive.getWantedMode());

		// debug
		// superstructure.getStateScoreAuto();
		currentState = superstructure.getCurrentState();
		// Logger.recordOutput("zeropose", Pose2d.kZero);

		// Logger.recordOutput("IK/lookaheadgood?",
		// 		InverseKinematics.shouldWristOutOftheway(superstructure.getCurrentState()));
		// Logger.recordOutput("IK/potential wrist angle", InverseKinematics.getWristOutofTheWayMaxAngle(superstructure.getCurrentState(), WristConstants.kStowAngle).in(Degree));
		// Logger.recordOutput("IK/wristTopPos/x", InverseKinematics.wristTopPos(superstructure.getCurrentState()).getMeasureX().in(Inches));
		// Logger.recordOutput("IK/wristTopPos/y", InverseKinematics.wristTopPos(superstructure.getCurrentState()).getMeasureY().in(Inches));
		// Logger.recordOutput("IK/wristTopPos/translation", InverseKinematics.wristTopPos(superstructure.getCurrentState()));
		// Logger.recordOutput("thingy", InverseKinematics.getWristOutofTheWayMaxAngle(superstructure.getCurrentState(), WristConstants.kStowAngle));

		Logger.recordOutput("DEBUG/pivot/minangle", InverseKinematics.getMinAngle(currentState));
		

	}


	

}
