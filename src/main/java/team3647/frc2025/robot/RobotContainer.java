// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package team3647.frc2025.robot;

import static edu.wpi.first.units.Units.Degree;
import static edu.wpi.first.units.Units.MetersPerSecond;

import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import team3647.frc2025.Util.AutoDrive;
import team3647.frc2025.autos.AutoCommands;
import team3647.frc2025.commands.ClimbCommands;
import team3647.frc2025.commands.SwerveDriveCommands;
import team3647.frc2025.constants.AutoConstants;
import team3647.frc2025.constants.ClimbConstants;
import team3647.frc2025.constants.CoralerConstants;
import team3647.frc2025.constants.ElevatorConstants;
import team3647.frc2025.constants.FieldConstants;
import team3647.frc2025.constants.GlobalConstants;
import team3647.frc2025.constants.PivotConstants;
import team3647.frc2025.constants.RollersConstants;
import team3647.frc2025.constants.SwerveDriveConstants;
import team3647.frc2025.constants.TunerConstants;
import team3647.frc2025.constants.VisionConstants;
import team3647.frc2025.constants.WristConstants;
import team3647.frc2025.subsystems.Climb;
import team3647.frc2025.subsystems.Coraler;
import team3647.frc2025.subsystems.Elevator;
import team3647.frc2025.subsystems.Pivot;
import team3647.frc2025.subsystems.Rollers;
import team3647.frc2025.subsystems.Superstructure;
import team3647.frc2025.subsystems.Superstructure.Branch;
import team3647.frc2025.subsystems.Superstructure.Level;
import team3647.frc2025.subsystems.SwerveDrive;
import team3647.frc2025.subsystems.Wrist;
import team3647.lib.GroupPrinter;
import team3647.lib.inputs.Joysticks;
import team3647.lib.team9442.AllianceChecker;
import team3647.lib.team9442.AutoChooser;
import team3647.lib.vision.AprilTagLimelight;
import team3647.lib.vision.AprilTagPhotonVision;
import team3647.lib.vision.NeuralDetectorLimelight;
import team3647.lib.vision.VisionController;

public class RobotContainer {
    public final Joysticks mainController = new Joysticks(0);
    public final Joysticks coController = new Joysticks(1);

    public Field2d smartDashboardField = new Field2d();

    public RobotContainer() {
        configureBindings();
        configureDefaultCommands();
        configureAllianceObservers();
        configureSmartDashboardLogging();

        superstructure.setIsAlignedFunction(autoDrive::isAlignedToReef);
        elevator.setEncoderHeight(ElevatorConstants.kStartingHeight);
        pivot.setEncoderAngle(PivotConstants.kStartingAngle);
        wrist.setEncoderAngle(WristConstants.kStartingAngle);

        CommandScheduler.getInstance()
                .registerSubsystem(swerve, elevator, pivot, coraler, wrist, rollers, climb);
    }

    private void configureAllianceObservers() {
        allianceChecker.registerObservers(
                swerveCommands, swerve, autoCommands, autoChooser, autoDrive);
    } // 0.6324678425924254

    private void configureBindings() {

        // // sysid
        // mainController.leftMidButton.and(mainController.buttonY).whileTrue(swerve.runDriveDynamTestFOC(Direction.kForward));
        // mainController.leftMidButton.and(mainController.buttonX).whileTrue(swerve.runDriveDynamTestFOC(Direction.kReverse));
        // mainController.rightMidButton.and(mainController.buttonY).whileTrue(swerve.runDriveQuasiTestFOC(Direction.kForward));
        // mainController.rightMidButton.and(mainController.buttonX).whileTrue(swerve.runDriveQuasiTestFOC(Direction.kReverse));

        mainController.rightTrigger.whileTrue(superstructure.autoScoreByLevel());
        mainController.rightTrigger.onFalse(superstructure.stow());

        // mainController.leftTrigger.onTrue(autoDrive.setDriveMode(DriveMode.TEST)).onFalse(autoDrive.setDriveMode(DriveMode.NONE));

        // cocontroller selecting the branch you wanna score coral on
        coController.leftBumper.onTrue(autoDrive.setWantedBranch(Branch.ONE));
        coController.rightBumper.onTrue(autoDrive.setWantedBranch(Branch.TWO));
        coController.buttonX.whileTrue(superstructure.stowElevAndPivot());

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

    private void configureSmartDashboardLogging() {

        SmartDashboard.putData("Autos", autoChooser);
        SmartDashboard.putData("Field", smartDashboardField);
        printer.addDouble("Match Time", Timer::getMatchTime);
    }

    public void updateRobotPoseForSmartdashboard() {
        smartDashboardField.setRobotPose(swerve.getOdoPose());
    }

    private void configureDefaultCommands() {
        swerve.setDefaultCommand(
                swerveCommands.driveVisionTeleop(
                        mainController::getLeftStickX,
                        mainController::getLeftStickY,
                        mainController::getRightStickX,
                        autoDrive::getVelocities,
                        autoDrive::getWantedMode,
                        autoDrive::getAutoDriveEnabled));
        elevator.setDefaultCommand(superstructure.elevatorCommands.holdPositionAtCall());
        pivot.setDefaultCommand(superstructure.pivotCommands.holdPositionAtCall());
        coraler.setDefaultCommand(superstructure.coralerCommands.kill());
        wrist.setDefaultCommand(superstructure.wristCommands.holdPositionAtCall());
        climb.setDefaultCommand(climbCommands.kill());
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
                    SwerveDriveConstants.ppRobotConfig,
                    SwerveDriveConstants.kTeleopKinematicLimits,
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
    public final Wrist wrist =
            new Wrist(
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

    Rollers rollers =
            new Rollers(
                    RollersConstants.kMaster,
                    0,
                    0,
                    GlobalConstants.kNominalVoltage,
                    GlobalConstants.kDt);

    Climb climb =
            new Climb(
                    ClimbConstants.kMaster,
                    0,
                    0,
                    GlobalConstants.kNominalVoltage,
                    GlobalConstants.kDt);

    public final Superstructure superstructure =
            new Superstructure(coraler, elevator, pivot, wrist, rollers, mainController.buttonY);

    NeuralDetectorLimelight detector = new NeuralDetectorLimelight(VisionConstants.kIntakeLLName);

    public final AutoDrive autoDrive =
            new AutoDrive(
                    swerve::getOdoPose,
                    FieldConstants.redSources,
                    FieldConstants.blueSources,
                    AutoConstants.xController,
                    AutoConstants.yController,
                    AutoConstants.rotController,
                    FieldConstants.redReefSides,
                    FieldConstants.blueReefSides,
                    detector);

    public final SwerveDriveCommands swerveCommands =
            new SwerveDriveCommands(
                    swerve, MetersPerSecond.of(SwerveDriveConstants.kDrivePossibleMaxSpeedMPS));
    public final ClimbCommands climbCommands = new ClimbCommands(climb);

    private Trigger goodToScore = new Trigger(() -> !coController.buttonX.getAsBoolean());

    public final AllianceChecker allianceChecker = new AllianceChecker();

    public final AutoConstants autoConstants = new AutoConstants();

    public final AutoCommands autoCommands = new AutoCommands(swerve, superstructure, autoDrive);

    public final AutoChooser autoChooser = new AutoChooser(autoCommands, swerve::setRobotPose);

    private final GroupPrinter printer = GroupPrinter.getInstance();

    AprilTagPhotonVision frontLeft =
            new AprilTagPhotonVision(
                    VisionConstants.frontLeftCamName,
                    VisionConstants.FrontLeft,
                    VisionConstants.baseStdDevs);
    AprilTagPhotonVision frontRight =
            new AprilTagPhotonVision(
                    VisionConstants.frontRightCamName,
                    VisionConstants.FrontRight,
                    VisionConstants.baseStdDevs);

    AprilTagPhotonVision backRight =
            new AprilTagPhotonVision(
                    VisionConstants.backRightCamName,
                    VisionConstants.BackRight,
                    VisionConstants.baseStdDevs);
    AprilTagPhotonVision backLeft =
            new AprilTagPhotonVision(
                    VisionConstants.backLeftCamName,
                    VisionConstants.BackLeft,
                    VisionConstants.baseStdDevs,
                    (pose) -> true);

    AprilTagLimelight xBar =
            new AprilTagLimelight(
                    VisionConstants.kCrossbarLLName,
                    VisionConstants.LLCrossMount,
                    swerve::getPigeonOrientation,
                    VisionConstants.baseStdDevs);

    // AprilTagPhotonVision frontRight =
    // new AprilTagPhotonVision("frontRight", VisionConstants.kRobotToFrontRight ,
    // VisionConstants.baseStdDevs);

    public final VisionController controller =
            new VisionController(
                    swerve::addVisionData,
                    swerve::shouldAddData,
                    swerve::resetPose,
                    frontLeft,
                    frontRight,
                    backRight,
                    xBar);

    Trigger score =
            new Trigger(
                    () -> {
                        return ((superstructure.isAligned()
                                || mainController.buttonY.getAsBoolean()));
                    });

    Trigger safeToIntakeUp =
            new Trigger(mainController.leftBumper.and(() -> !DriverStation.isAutonomous()));

    Trigger safeToScore =
            new Trigger(mainController.rightTrigger.and(() -> !DriverStation.isAutonomous()));

    Trigger intakeUp =
            new Trigger(
                            () ->
                                    superstructure.intakeCurrent().getAsBoolean()
                                            && wrist.getAngleDegs() < 20)
                    .or(mainController.buttonB);
    Trigger algaePrepped =
            new Trigger(
                    () ->
                            pivot.angleReached(PivotConstants.kStowAngleUp, Degree.of(40))
                                    && (superstructure.getWantedLevel() == Level.ALGAEHIGH
                                            || superstructure.getWantedLevel() == Level.ALGAELOW));
}
