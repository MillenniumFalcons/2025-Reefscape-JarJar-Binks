// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package team3647.frc2025.robot;

import static edu.wpi.first.units.Units.Degree;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Radian;

import org.dyn4j.world.ManifoldCollisionData;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import team3647.frc2025.Util.AutoDrive;
import team3647.frc2025.Util.SuperstructureState;
import team3647.frc2025.autos.AutoCommands;
import team3647.frc2025.commands.CoralerCommands;
import team3647.frc2025.commands.ElevatorCommands;
import team3647.frc2025.commands.SwerveDriveCommands;
import team3647.frc2025.commands.WristCommands;
import team3647.frc2025.constants.AutoConstants;
import team3647.frc2025.constants.CoralerConstants;
import team3647.frc2025.constants.ElevatorConstants;
import team3647.frc2025.constants.FieldConstants;
import team3647.frc2025.constants.GlobalConstants;
import team3647.frc2025.constants.PivotConstants;
import team3647.frc2025.constants.RollersConstants;
import team3647.frc2025.constants.SwerveDriveConstants;
import team3647.frc2025.constants.TunerConstants;
import team3647.frc2025.constants.TunerSimConstants;
import team3647.frc2025.constants.VisionConstants;
import team3647.frc2025.constants.WristConstants;
import team3647.frc2025.subsystems.Coraler;
import team3647.frc2025.subsystems.Elevator;
import team3647.frc2025.subsystems.Pivot;
import team3647.frc2025.subsystems.Superstructure;
import team3647.frc2025.subsystems.Superstructure.Branch;
import team3647.frc2025.subsystems.Superstructure.Level;
import team3647.frc2025.subsystems.Superstructure.Side;
import team3647.frc2025.subsystems.SwerveDrive;
import team3647.frc2025.subsystems.Wrist;
import team3647.frc2025.subsystems.rollers;
import team3647.lib.inputs.Joysticks;
import team3647.lib.team9442.AllianceChecker;
import team3647.lib.team9442.AutoChooser;
import team3647.lib.vision.AprilTagLimelight;
import team3647.lib.vision.AprilTagPhotonVision;
import team3647.lib.vision.VisionController;



public class RobotContainer {
	public final Joysticks mainController = new Joysticks(0);
    public final Joysticks coController = new Joysticks(1);
    public RobotContainer() {
        configureBindings();
        configureDefaultCommands();
        configureAllianceObservers();
        SmartDashboard.putData(autoChooser);
		swerve.setRobotPose(new Pose2d(16, 0.7, Rotation2d.fromRadians(Math.PI)));
        
        superstructure.setIsAlignedFunction(autoDrive::isAlignedToReef);
		elevator.setEncoderHeight(ElevatorConstants.kStartingHeight);
		pivot.setEncoderAngle(PivotConstants.kStartingAngle);
		wrist.setEncoderAngle(WristConstants.kStartingAngle);

		CommandScheduler.getInstance().registerSubsystem(swerve, elevator, coraler, pivot, wrist, rollers);
		
    }

    private void configureAllianceObservers() {
        allianceChecker.registerObservers(
                swerveCommands, swerve, autoCommands, autoChooser);
    }//0.6324678425924254

	

    private void configureBindings() {

		// // sysid
		// mainController.leftMidButton.and(mainController.buttonY).whileTrue(elevator.elevSysidDynamFor());
        // mainController.leftMidButton.and(mainController.buttonX).whileTrue(elevator.elevSysidDynamBack());
        // mainController.rightMidButton.and(mainController.buttonY).whileTrue(elevator.elevSysidQuasiFor());
        // mainController.rightMidButton.and(mainController.buttonX).whileTrue(elevator.elevSysidQuasiBack());
		


		// mainController.leftBumper.whileTrue(superstructure.prepIntake().until(intakeUp));
		// mainController.leftBumper.onFalse(superstructure.stowIntake());
		// intakeUp.and(safeToIntakeUp).onTrue(superstructure.stowFromIntake().withTimeout(3));



		// mainController.rightTrigger.whileTrue(superstructure.autoPrepByWantedLevel());
		// mainController.rightTrigger.and(mainController.buttonX.negate()).onFalse(superstructure.poopCoral());	

		mainController.rightTrigger.whileTrue(superstructure.poopCoral());
		mainController.buttonA.whileTrue(superstructure.elevatorCommands.setHeight(ElevatorConstants.kLevel2Height));
		mainController.buttonX.whileTrue(superstructure.elevatorCommands.setHeight(ElevatorConstants.kLevel1Height));
		mainController.buttonB.whileTrue(superstructure.elevatorCommands.setHeight(ElevatorConstants.kLevel3Height));
		mainController.buttonY.whileTrue(superstructure.elevatorCommands.setHeight(ElevatorConstants.kLevel4Height));

		// mainController.rightMidButton.whileTrue(superstructure.stowElevAndPivot());

		
		// score.and(safeToScore).onTrue(superstructure.autoScoreByLevel());

		// mainController.leftTrigger.negate().and(mainController.buttonX).onTrue(getAutonomousCommand());

		


		

        // cocontroller selecting the branch you wanna score coral on
        // coController
        //         .buttonA
        //         .and(coController.buttonB.negate())
        //         .and(coController.buttonX.negate())
        //         .onTrue(superstructure.setWantedSide(Side.A))
        //         .debounce(0.1);

        // coController.buttonA.and(coController.buttonB).onTrue(superstructure.setWantedSide(Side.B));

        // coController.buttonB.and(coController.buttonY).onTrue(superstructure.setWantedSide(Side.C));

        // coController
        //         .buttonY
        //         .and(coController.buttonB.negate())
        //         .and(coController.buttonX.negate())
        //         .onTrue(superstructure.setWantedSide(Side.D))
        //         .debounce(0.1);


        // coController.buttonY.and(coController.buttonX).onTrue(superstructure.setWantedSide(Side.E));

        // coController.buttonX.and(coController.buttonA).onTrue(superstructure.setWantedSide(Side.F));

        // coController.leftBumper.onTrue(superstructure.setWantedBranch(Branch.ONE));

        // coController.rightBumper.onTrue(superstructure.setWantedBranch(Branch.TWO));

        coController.dPadUp.onTrue(superstructure.setWantedLevel(Level.HIGH));

        coController.dPadRight.onTrue(superstructure.setWantedLevel(Level.MID));

        coController.dPadDown.onTrue(superstructure.setWantedLevel(Level.LOW));

        coController.dPadLeft.onTrue(superstructure.setWantedLevel(Level.TROUGH));

        coController.rightMidButton.onTrue(autoDrive.enableAutoDrive());

        coController.leftMidButton.onTrue(autoDrive.disableAutoDrive());

		
    }

    private void configureDefaultCommands() {
        swerve.setDefaultCommand(
                swerveCommands.driveCmd(
                        mainController::getLeftStickX,
                        mainController::getLeftStickY,
                        mainController::getRightStickX,
                        autoDrive::getVelocities,
                        autoDrive::getWantedMode,
                        autoDrive::getAutoDriveEnabled));
		elevator.setDefaultCommand(superstructure.elevatorCommands.holdPositionAtCall());
		// pivot.setDefaultCommand(superstructure.pivotCommands.holdPositionAtCall());
		coraler.setDefaultCommand(superstructure.coralerCommands.setOpenLoop(0));
		wrist.setDefaultCommand(superstructure.wristCommands.holdPositionAtCall());
    }

    public Command getAutonomousCommand() {
        return autoChooser.getSelected().getAutoCommand();
    }

    @SuppressWarnings("unchecked")
    public final SwerveDrive swerve =
            new SwerveDrive(
                    TunerConstants.DrivetrainConstants,
                    SwerveDriveConstants.kDrivePossibleMaxSpeedMPS,
                    SwerveDriveConstants.kRotPossibleMaxSpeedRadPerSec,
                    GlobalConstants.kDt,
                    AutoConstants.ppRobotConfig,
                    TunerConstants.FrontLeft,
                    TunerConstants.FrontRight,
                    TunerConstants.BackLeft,
                    TunerConstants.BackRight);

    public final Coraler coraler =
            new Coraler(
                    CoralerConstants.kMaster,
                    0,
                    0,
                    GlobalConstants.kNominalVoltage,
                    GlobalConstants.kDt);

    public final Elevator elevator =
            new Elevator(
                    ElevatorConstants.kMaster,
                    ElevatorConstants.kSlave,
                    ElevatorConstants.kNativeToMeters,
                    ElevatorConstants.kNativeToMeters,

                    GlobalConstants.kNominalVoltage,
                    0,
                    ElevatorConstants.kMinHeight.in(Units.Meter),
                    ElevatorConstants.kMaxHeight.in(Units.Meter),
                    GlobalConstants.kDt);
	public final Wrist wrist  = new Wrist(
		WristConstants.kMaster, 
		WristConstants.kNativeToDeg, 
		WristConstants.kNativeToDeg,
		GlobalConstants.kNominalVoltage, 
		WristConstants.kMinAngle, 
		WristConstants.kMaxAngle, 
		GlobalConstants.kDt);

    public final Pivot pivot =
            new Pivot(
                    PivotConstants.kMaster,
                    PivotConstants.kMaxAngle,
                    PivotConstants.kMinAngle,
                    0,
                    PivotConstants.kNativeToRad,
                    PivotConstants.kNativeToRad,
                    GlobalConstants.kNominalVoltage,
					PivotConstants.kClearAngle,
					PivotConstants.kLowClearAngle,
					elevator::getHeight,
                    GlobalConstants.kDt);


        rollers rollers = new rollers(
                RollersConstants.kMaster, 0, 0, GlobalConstants.kNominalVoltage, GlobalConstants.kDt);

    public final Superstructure superstructure = new Superstructure(coraler, elevator, pivot,wrist, rollers, mainController.buttonY);

    public final AutoDrive autoDrive =
            new AutoDrive(
                    swerve::getOdoPose,
                    superstructure::getWantedScoringPos,
                    FieldConstants.redSources,
                    FieldConstants.blueSources,
                    AutoConstants.xController,
                    AutoConstants.yController,
                    AutoConstants.rotController);

    public final SwerveDriveCommands swerveCommands =
            new SwerveDriveCommands(
                    swerve, MetersPerSecond.of(SwerveDriveConstants.kDrivePossibleMaxSpeedMPS));

   

	private Trigger goodToScore = new Trigger(() -> !mainController.buttonX.getAsBoolean());

    public final AllianceChecker allianceChecker = new AllianceChecker();

    public final AutoConstants autoConstants = new AutoConstants();

    public final AutoCommands autoCommands = new AutoCommands(autoConstants, swerve);

    public final AutoChooser autoChooser = new AutoChooser(autoCommands, swerve::setRobotPose);

    AprilTagPhotonVision frontLeft = new AprilTagPhotonVision(
		VisionConstants.frontLeftCamName,
		VisionConstants.FrontLeft,
		VisionConstants.baseStdDevs
	);
	AprilTagPhotonVision frontRight = new AprilTagPhotonVision(
		VisionConstants.frontRightCamName,
		VisionConstants.FrontRight,
		VisionConstants.baseStdDevs
	);

	AprilTagPhotonVision backRight = new AprilTagPhotonVision(
		VisionConstants.backRightCamName,
		VisionConstants.BackRight,
		VisionConstants.baseStdDevs
	);

    AprilTagLimelight xBar = new AprilTagLimelight(
		VisionConstants.kCrossbarLLName,
		VisionConstants.LLCrossMount,
		swerve::getPigeonOrientation,
		VisionConstants.baseStdDevs
	);

	// AprilTagPhotonVision frontRight = 
			// new AprilTagPhotonVision("frontRight", VisionConstants.kRobotToFrontRight , VisionConstants.baseStdDevs);

    public final VisionController controller =
            new VisionController(
                    swerve::addVisionData,
                    swerve::shouldAddData,
                    swerve::resetPose,
                    frontLeft,
					frontRight,
					backRight,
					xBar);
	


	Trigger score = new Trigger(() -> {
		return ((superstructure.isAligned() || mainController.buttonY.getAsBoolean()) && !mainController.buttonX.getAsBoolean());
	});

	Trigger safeToIntakeUp = new Trigger(mainController.leftBumper.and(() -> !DriverStation.isAutonomous()));

	Trigger safeToScore = new Trigger(mainController.rightTrigger.and(goodToScore).and(() -> !DriverStation.isAutonomous()));

	Trigger 
	intakeUp = new Trigger(() -> superstructure.intakeCurrent() && wrist.getAngleDegs() < 10).or(mainController.buttonB);
}
