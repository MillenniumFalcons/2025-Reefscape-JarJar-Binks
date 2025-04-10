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
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import team3647.frc2025.Util.AutoDrive;
import team3647.frc2025.Util.AutoDrive.DriveMode;
import team3647.frc2025.Util.LEDTriggers;
import team3647.frc2025.Util.SuperstructureState;
import team3647.frc2025.autos.AutoCommands;
import team3647.frc2025.commands.ClimbCommands;
import team3647.frc2025.commands.SwerveDriveCommands;
import team3647.frc2025.constants.AutoConstants;
import team3647.frc2025.constants.ClimbConstants;
import team3647.frc2025.constants.CoralerConstants;
import team3647.frc2025.constants.ElevatorConstants;
import team3647.frc2025.constants.FieldConstants;
import team3647.frc2025.constants.GlobalConstants;
import team3647.frc2025.constants.LEDConstants;
import team3647.frc2025.constants.PivotConstants;
import team3647.frc2025.constants.RollersConstants;
import team3647.frc2025.constants.SwerveDriveConstants;
import team3647.frc2025.constants.TunerConstants;
import team3647.frc2025.constants.VisionConstants;
import team3647.frc2025.constants.WristConstants;
import team3647.frc2025.subsystems.Climb;
import team3647.frc2025.subsystems.Coraler;
import team3647.frc2025.subsystems.Elevator;
import team3647.frc2025.subsystems.LEDs;
import team3647.frc2025.subsystems.Pivot;
import team3647.frc2025.subsystems.Rollers;
import team3647.frc2025.subsystems.Seagull;
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

        superstructure.setIsAlignedFunction(autoDrive.isAlignedToReef());
        elevator.setEncoderHeight(ElevatorConstants.kStartingHeight);
        pivot.setEncoderAngle(PivotConstants.kStartingAngle);
        wrist.setEncoderAngle(WristConstants.kStartingAngle);

        // swerve.resetPose(new Pose2d(1.88, 4.09, Rotation2d.k180deg));

        CommandScheduler.getInstance()
                .registerSubsystem(
                        swerve, elevator, pivot, coraler, wrist, rollers, climb, seagull);
    }

    private void configureAllianceObservers() {
        allianceChecker.registerObservers(
                swerveCommands, swerve, autoCommands, autoChooser, autoDrive);
    } // 0.6324678425924254

    private void configureBindings() {

        // algae stuff

        mainController.rightBumper.whileTrue(
                superstructure
                        .autoTakeOffByLevel()
                        .until(() -> superstructure.coralerAlgaeCurrent())
                        .andThen(
                                superstructure
                                        .coralerCommands
                                        .setOpenLoop(0.2)
                                        .alongWith(superstructure.setHasAlgae())));

        mainController.rightBumper.onFalse(
                superstructure.goToStateParalellNoWrist(() -> SuperstructureState.AlgaeStow));

        // mainController.rightBumper.and(algaeReadyToScore).onFalse(superstructure.paralellStow());

        algaeReadyToScore.whileTrue(superstructure.coralerCommands.setOpenLoop(0.2));

        algaeReadyToScore
                .and(mainController.leftTrigger)
                .whileTrue(superstructure.scoreAlgaeBarge());

        algaeReadyToScore
                .and(mainController.leftTrigger)
                .onFalse(superstructure.stowAlgaeBarge().alongWith(superstructure.setNoAlgae()));

        // real stuff

        mainController.leftBumper.whileTrue(superstructure.intake());
        intakeUp.onTrue(superstructure.transfer()).onTrue(autoDrive.setDriveMode(DriveMode.NONE));
        mainController.buttonB.whileTrue(superstructure.transfer());
        mainController
                .buttonA
                .and(mainController.dPadLeft)
                .whileTrue(autoDrive.setDriveMode(DriveMode.SCORE));

        mainController.buttonA.and(mainController.dPadLeft).onFalse(autoDrive.clearDriveMode());

        seagullCurrent
                .and(() -> !superstructure.intakeCurrent())
                .and(mainController.rightBumper.negate())
                .onTrue(superstructure.handoff().alongWith(superstructure.setPeice()));
        coralerCurrent.and(mainController.rightBumper.negate()).onTrue(superstructure.stow());

        mainController.leftBumper.onFalse(
                superstructure
                        .wristCommands
                        .stow()
                        .alongWith(
                                superstructure.coralerCommands.kill(),
                                superstructure.rollersCommands.kill()));

        mainController
                .rightTrigger
                .whileTrue(Commands.waitUntil(superstructure.wristCommands.angleBelow(50)).andThen(superstructure.autoScoreByLevel()))
                .whileTrue(
                        Commands.sequence(
                                        superstructure.wristCommands.setAngle(Degree.of(46)),
                                        Commands.waitSeconds(0.3),
                                        superstructure.wristCommands.setAngle(
                                                WristConstants.kStowAngle)));
        mainController.rightTrigger.onFalse(
                superstructure.autoStowByLevel());
        // 			.onFalse(Commands.sequence(
        // superstructure.wristCommands.setAngle(Degree.of(30)),
        // Commands.waitSeconds(0.5),
        // superstructure.wristCommands.setAngle(Degree.of(80))));

        mainController
                .buttonX
                .whileTrue(superstructure.rollersCommands.setOpenLoop(-0.3, 0.3))
                .onFalse(superstructure.rollersCommands.setOpenLoop(0, 0));

        coController.buttonB.whileTrue(
                superstructure
                        .goToStateParalell(superstructure::getCurrentState)
                        .alongWith(superstructure.killAll()));

        mainController.rightMidButton.whileTrue(
                Commands.sequence(
                                superstructure.goToStateParalell(() -> SuperstructureState.ToStow),
                                superstructure.goToStateParalell(() -> SuperstructureState.Stow))
                        .alongWith(superstructure.setNoPeice()));

        mainController.leftMidButton.whileTrue(
                superstructure
                        .goToStateParalell(() -> SuperstructureState.ToStow)
                        .alongWith(superstructure.setNoPeice()));

        // cocontroller selecting the branch you wanna score coral on
        coController.leftBumper.onTrue(autoDrive.setWantedBranch(Branch.ONE));
        coController.rightBumper.onTrue(autoDrive.setWantedBranch(Branch.TWO));
        coController
                .leftJoyStickPress
                .whileTrue(superstructure.coralerCommands.spitOut())
                .onFalse(superstructure.coralerCommands.kill());

        // mainController.leftTrigger.and(algaeReadyToScore.negate()).onTrue(autoDrive.setDriveMode(DriveMode.SCORE));
        // mainController.leftTrigger.onFalse(autoDrive.setDriveMode(DriveMode.NONE));

        autoDrive.isAlignedToReef().debounce(0.1).onTrue(autoDrive.clearDriveMode());

        coController.leftTrigger.whileTrue(superstructure.setWantedLevel(Level.ALGAELOW));
        coController.leftTrigger.onFalse(superstructure.setWantedLevel(Level.HIGH));

        // coController
        // .buttonA
        // .and(coController.buttonB.negate())
        // .and(coController.buttonX.negate())
        // .onTrue(superstructure.setWantedSide(Side.A))
        // .debounce(0.1);

        // coController.buttonA.and(coController.buttonB).onTrue(superstructure.setWantedSide(Side.B));

        // coController.buttonB.and(coController.buttonY).onTrue(superstructure.setWantedSide(Side.C));

        // coController
        // .buttonY
        // .and(coController.buttonB.negate())
        // .and(coController.buttonX.negate())
        // .onTrue(superstructure.setWantedSide(Side.D))
        // .debounce(0.1);

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

        coController.rightJoyStickPress.whileTrue(front.setConvergeToMT1());
        coController.rightJoyStickPress.onFalse(front.setConvergeToGyro());

        mainController.dPadUp.whileTrue(climbCommands.climbOut()).onFalse(climbCommands.kill());
        mainController.dPadDown.whileTrue(climbCommands.climbIn()).onFalse(climbCommands.kill());
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
                        autoDrive::getAutoDriveEnabled,
                        autoDrive::hasScoringTarget,
                        new Trigger(() -> false)));
        elevator.setDefaultCommand(superstructure.elevatorCommands.holdPositionAtCall());
        pivot.setDefaultCommand(superstructure.pivotCommands.holdPositionAtCall());
        coraler.setDefaultCommand(superstructure.coralerCommands.kill());
        wrist.setDefaultCommand(superstructure.wristCommands.stow());
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
                    GlobalConstants.kDt,
                    mainController.rightTrigger);

    public final Wrist wrist =
            new Wrist(
                    WristConstants.kMaster,
                    WristConstants.kNativeToDeg,
                    WristConstants.kNativeToDeg,
                    GlobalConstants.kNominalVoltage,
                    WristConstants.kMinAngle,
                    WristConstants.kMaxAngle,
                    elevator::getHeight,
                    pivot::getAngle,
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
    Seagull seagull =
            new Seagull(
                    RollersConstants.kSeagull,
                    GlobalConstants.kNominalVoltage,
                    GlobalConstants.kDt);

    public final Superstructure superstructure =
            new Superstructure(
                    coraler, elevator, pivot, wrist, rollers, seagull, mainController.buttonY);

    private final LEDTriggers triggers = new LEDTriggers(superstructure);
    private final LEDs leds = new LEDs(LEDConstants.m_candle, triggers);

    NeuralDetectorLimelight detector = new NeuralDetectorLimelight(VisionConstants.kIntakeLLName);

    AprilTagLimelight front =
            new AprilTagLimelight(
                    VisionConstants.frontLLName,
                    VisionConstants.FrontLL,
                    swerve::getPigeonOrientation,
                    VisionConstants.baseStdDevs);

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
                    detector,
                    front);

    public final SwerveDriveCommands swerveCommands =
            new SwerveDriveCommands(
                    swerve, MetersPerSecond.of(SwerveDriveConstants.kDrivePossibleMaxSpeedMPS));
    public final ClimbCommands climbCommands = new ClimbCommands(climb);

    public final AllianceChecker allianceChecker = new AllianceChecker();

    public final AutoConstants autoConstants = new AutoConstants();

    public final AutoCommands autoCommands =
            new AutoCommands(swerve, superstructure, autoDrive::getVelocities, detector::hasTarget);

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
                    VisionConstants.baseStdDevs);

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
                    backLeft,
                    front);
    public final RobotTracker tracker = new RobotTracker(superstructure, autoDrive);

    Trigger score =
            new Trigger(
                    () -> {
                        return ((superstructure.isAligned()
                                || mainController.buttonY.getAsBoolean()));
                    });

    Trigger intakeUp =
            new Trigger(() -> superstructure.intakeCurrent() && wrist.getAngleDegs() < 20)
                    .debounce(0.5)
                    .or(mainController.buttonB)
                    .and(() -> !DriverStation.isAutonomous());
    Trigger seagullCurrent =
            new Trigger(() -> superstructure.seagullCurrent())
                    .debounce(0.5)
                    .or(coController.buttonY)
                    .and(() -> !DriverStation.isAutonomous());

    Trigger coralerCurrent =
            superstructure
                    .coralerCommands
                    .current()
                    .debounce(0.4)
                    .and(mainController.leftBumper)
                    .and(() -> pivot.getAngleRads() < -0.5)
                    .or(coController.buttonA)
                    .and(() -> !DriverStation.isAutonomous());

    Trigger algaePrepped =
            new Trigger(
                            () ->
                                    pivot.angleReached(PivotConstants.kStowAngleUp, Degree.of(40))
                                            && (superstructure.getWantedLevel() == Level.ALGAEHIGH
                                                    || superstructure.getWantedLevel()
                                                            == Level.ALGAELOW))
                    .and(() -> !DriverStation.isAutonomous());

    Trigger algaeReadyToScore = new Trigger(() -> superstructure.getAlgae());
}
