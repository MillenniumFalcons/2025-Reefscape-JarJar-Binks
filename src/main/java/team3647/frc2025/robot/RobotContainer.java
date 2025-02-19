// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package team3647.frc2025.robot;

import static edu.wpi.first.units.Units.MetersPerSecond;

import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import team3647.frc2025.Util.AutoDrive;
import team3647.frc2025.autos.AutoCommands;
import team3647.frc2025.commands.SwerveDriveCommands;
import team3647.frc2025.constants.AutoConstants;
import team3647.frc2025.constants.CoralerConstants;
import team3647.frc2025.constants.ElevatorConstants;
import team3647.frc2025.constants.FieldConstants;
import team3647.frc2025.constants.GlobalConstants;
import team3647.frc2025.constants.PivotConstants;
import team3647.frc2025.constants.SwerveDriveConstants;
import team3647.frc2025.constants.TunerConstants;
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
import team3647.lib.inputs.Joysticks;
import team3647.lib.team9442.AllianceChecker;
import team3647.lib.team9442.AutoChooser;
import team3647.lib.vision.AprilTagLimelight;
import team3647.lib.vision.AprilTagPhotonVision;
import team3647.lib.vision.VisionController;

public class RobotContainer {
    public RobotContainer() {
        configureBindings();
        configureDefaultCommands();
        configureAllianceObservers();
        SmartDashboard.putData(autoChooser);

        superstructure.setIsAlignedFunction(autoDrive::isAlignedToReef);
        elevator.setEncoderHeight(ElevatorConstants.kStartingHeight);
        pivot.setEncoderAngle(PivotConstants.kStartingAngle);
        wrist.setEncoderAngle(WristConstants.kStartingAngle);

        CommandScheduler.getInstance().registerSubsystem(swerve, elevator, coraler, pivot, wrist);
    }

    private void configureAllianceObservers() {
        allianceChecker.registerObservers(swerveCommands, swerve, autoCommands, autoChooser);
    } // 0.6324678425924254

    private void configureBindings() {

        // elev sysid
        // mainController.leftMidButton.and(mainController.buttonY).whileTrue(elevator.elevSysidDynamFor());
        // mainController.leftMidButton.and(mainController.buttonX).whileTrue(elevator.elevSysidDynamBack());
        // mainController.rightMidButton.and(mainController.buttonY).whileTrue(elevator.elevSysidQuasiFor());
        // mainController.rightMidButton.and(mainController.buttonX).whileTrue(elevator.elevSysidQuasiBack());
        mainController.buttonA.whileTrue(
                superstructure.elevatorCommands.setHeight(ElevatorConstants.kLevel1Height));
        mainController.buttonA.onFalse(superstructure.elevatorCommands.holdPositionAtCall());
        mainController.buttonB.whileTrue(
                superstructure.elevatorCommands.setHeight(ElevatorConstants.kLevel2Height));
        mainController.buttonB.onFalse(superstructure.elevatorCommands.holdPositionAtCall());
        mainController.buttonY.whileTrue(
                superstructure.elevatorCommands.setHeight(ElevatorConstants.kLevel3Height));
        mainController.buttonY.onFalse(superstructure.elevatorCommands.holdPositionAtCall());
        mainController.buttonX.whileTrue(
                superstructure.elevatorCommands.setHeight(ElevatorConstants.kLevel4Height));
        mainController.buttonX.onFalse(superstructure.elevatorCommands.holdPositionAtCall());

        mainController.dPadUp.whileTrue(
                superstructure.pivotCommands.setAngle(PivotConstants.kLevel1Angle));

        mainController.leftTrigger.whileTrue(superstructure.coralerCommands.setOpenLoop(-0.5));
        mainController.leftTrigger.onFalse(superstructure.coralerCommands.setOpenLoop(0));

        mainController.rightTrigger.whileTrue(superstructure.coralerCommands.setOpenLoop(0.5));
        mainController.rightTrigger.onFalse(superstructure.coralerCommands.setOpenLoop(-0.07));

        //if you see this after 2/12/2025 delete it
        mainController.rightBumper.whileTrue(
                superstructure.wristCommands.setAngle(WristConstants.kMaxAngle));
        mainController.leftBumper.whileTrue(
                superstructure.wristCommands.setOpenLoop(0.1));
        mainController.leftBumper.onFalse(
                superstructure.wristCommands.setOpenLoop(0)
        );
        wrist.setDefaultCommand(superstructure.wristCommands.setOpenLoop(0));

        // mainController.dPadUp.whileTrue(superstructure.pivotCommands.holdPositionAtCall());
        // mainController.dPadUp.onFalse(superstructure.pivotCommands.setOpenLoop(() -> 0));

        // mainController.dPadRight.whileTrue(superstructure.elevatorCommands.setOpenLoop(() ->
        // 0.4));
        // mainController.dPadRight.onFalse(superstructure.pivotCommands.setOpenLoop(() -> 0));
        // mainController.dPadLeft.whileTrue(superstructure.elevatorCommands.setOpenLoop(() -> 0));
        // mainController.dPadLeft.onFalse(superstructure.elevatorCommands.setOpenLoop(() -> 0));

        // cocontroller selecting the branch you wanna score coral on
        coController
                .buttonA
                .and(coController.buttonB.negate())
                .and(coController.buttonX.negate())
                .onTrue(superstructure.setWantedSide(Side.A))
                .debounce(0.1);

        coController.buttonA.and(coController.buttonB).onTrue(superstructure.setWantedSide(Side.B));

        coController.buttonB.and(coController.buttonY).onTrue(superstructure.setWantedSide(Side.C));

        coController
                .buttonY
                .and(coController.buttonB.negate())
                .and(coController.buttonX.negate())
                .onTrue(superstructure.setWantedSide(Side.D))
                .debounce(0.1);

        coController.buttonY.and(coController.buttonX).onTrue(superstructure.setWantedSide(Side.E));

        coController.buttonX.and(coController.buttonA).onTrue(superstructure.setWantedSide(Side.F));

        coController.leftBumper.onTrue(superstructure.setWantedBranch(Branch.ONE));

        coController.rightBumper.onTrue(superstructure.setWantedBranch(Branch.TWO));

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
        // coraler.setDefaultCommand(superstructure.coralerCommands.stow());
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

    public final Pivot pivot =
            new Pivot(
                    PivotConstants.kMaster,
                    PivotConstants.kMaxAngle,
                    PivotConstants.kMinAngle,
                    0,
                    PivotConstants.kNativeToRad,
                    PivotConstants.kNativeToRad,
                    GlobalConstants.kNominalVoltage,
                    GlobalConstants.kDt);

    public final Wrist wrist =
            new Wrist(
                    WristConstants.kMaster,
                    WristConstants.kNativeToRad,
                    WristConstants.kNativeToRad,
                    GlobalConstants.kNominalVoltage,
                    WristConstants.kMinAngle,
                    WristConstants.kMaxAngle,
                    GlobalConstants.kDt);

    public final Superstructure superstructure =
            new Superstructure(coraler, elevator, pivot, wrist);

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

    public final Joysticks mainController = new Joysticks(0);
    public final Joysticks coController = new Joysticks(1);

    public final AllianceChecker allianceChecker = new AllianceChecker();

    public final AutoConstants autoConstants = new AutoConstants();

    public final AutoCommands autoCommands = new AutoCommands(autoConstants, swerve);

    public final AutoChooser autoChooser = new AutoChooser(autoCommands, swerve::setRobotPose);

    AprilTagPhotonVision cam1ChangeName =
            new AprilTagPhotonVision(
                    "ballschangename", new Transform3d(), VisionConstants.baseStdDevs);

    AprilTagLimelight ll1ChangeName =
            new AprilTagLimelight(
                    "LL1ChangeName",
                    new Transform3d(),
                    swerve::getPigeonOrientation,
                    VisionConstants.baseStdDevs);

    public final VisionController controller =
            new VisionController(
                    swerve::addVisionData,
                    swerve::shouldAddData,
                    swerve::resetPose,
                    cam1ChangeName,
                    ll1ChangeName);
}
