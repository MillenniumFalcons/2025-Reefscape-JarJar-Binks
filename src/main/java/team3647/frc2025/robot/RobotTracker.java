package team3647.frc2025.robot;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;

import static edu.wpi.first.units.Units.Meters;

import org.littletonrobotics.junction.Logger;
import team3647.frc2025.Util.AutoDrive;
import team3647.frc2025.Util.SuperstructureState;
import team3647.frc2025.constants.ElevatorConstants;
import team3647.frc2025.subsystems.Superstructure;
import team3647.lib.team6328.VirtualSubsystem;

public class RobotTracker extends VirtualSubsystem {
    private final Superstructure superstructure;
    private final AutoDrive autoDrive;
    private SuperstructureState currentState = SuperstructureState.kInvalidState;

    Translation2d fk = new Translation2d();

    public RobotTracker(Superstructure superstructure, AutoDrive autoDrive) {
        super();
        this.superstructure = superstructure;
        this.autoDrive = autoDrive;
    }

    public double realToSimCoordsPivot(double pivotAngle){
        return -pivotAngle;
      }

    @Override
    public void periodic() {
        Logger.recordOutput("Superstructure/pivotoffset", superstructure.getPivotOffset());
        Logger.recordOutput("Superstructure/elevOffset", superstructure.getElevOffset());
        Logger.recordOutput("Superstructure/Level", superstructure.getWantedLevel());

        Logger.recordOutput("Robot/drive mode", autoDrive.getWantedMode());

        

        Pose3d[] poses = new Pose3d[]{
            new Pose3d(
                    ElevatorConstants.kZeroedElevPose[0].getX(),
                    ElevatorConstants.kZeroedElevPose[0].getY(),
                    superstructure.elevator.getHeight().in(Meters),
                    Rotation3d.kZero), 
            new Pose3d(
                    0.1257,
                    ElevatorConstants.kZeroedElevPose[0].getY(),
                    superstructure.elevator.getHeight().in(Meters),
                    new Rotation3d(0, realToSimCoordsPivot(superstructure.pivot.getAngleRads()), 0))
            };
        Logger.recordOutput("Superstructure/Components", poses);

        // debug
        // superstructure.getStateScoreAuto();
        currentState = superstructure.getCurrentState();

        // fk = InverseKinematics.forwardKinematics(currentState);

        // Logger.recordOutput("DEBUG/autowrist/ssX", fk.getMeasureX().in(Inches));
        // Logger.recordOutput("DEBUG/autowrist/ssY", fk.getMeasureY().in(Inches));
        // Logger.recordOutput(
        //         "DEBUG/autowrist/ssXThresholdLow",
        //         InverseKinematics.rect
        //                 .getCenter()
        //                 .getMeasureX()
        //                 .minus(InverseKinematics.rect.getMeasureXWidth().div(Value.of(2)))
        //                 .in(Inches));
        // Logger.recordOutput(
        //         "DEBUG/autowrist/ssXThresholdHigh",
        //         InverseKinematics.rect
        //                 .getCenter()
        //                 .getMeasureX()
        //                 .plus(InverseKinematics.rect.getMeasureXWidth().div(Value.of(2)))
        //                 .in(Inches));

        // Logger.recordOutput("zeropose", Pose2d.kZero);

        // Logger.recordOutput("IK/lookaheadgood?",
        // 		InverseKinematics.shouldWristOutOftheway(superstructure.getCurrentState()));
        // Logger.recordOutput("IK/potential wrist angle",
        // InverseKinematics.getWristOutofTheWayMaxAngle(superstructure.getCurrentState(),
        // WristConstants.kStowAngle).in(Degree));
        // Logger.recordOutput("IK/wristTopPos/x",
        // InverseKinematics.wristTopPos(superstructure.getCurrentState()).getMeasureX().in(Inches));
        // Logger.recordOutput("IK/wristTopPos/y",
        // InverseKinematics.wristTopPos(superstructure.getCurrentState()).getMeasureY().in(Inches));
        // Logger.recordOutput("IK/wristTopPos/translation",
        // InverseKinematics.wristTopPos(superstructure.getCurrentState()));
        // Logger.recordOutput("thingy",
        // InverseKinematics.getWristOutofTheWayMaxAngle(superstructure.getCurrentState(),
        // WristConstants.kStowAngle));

        // Logger.recordOutput("xpivot/minangle", InverseKinematics.getMinAngle(currentState));

    }
}
