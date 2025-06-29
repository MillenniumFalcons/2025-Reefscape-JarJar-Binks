// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package team3647.frc2025.robot;

import static edu.wpi.first.units.Units.Degree;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Radian;

import com.ctre.phoenix6.Utils;

import choreo.auto.AutoFactory;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
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
import team3647.frc2025.constants.ElevatorConstants;
import team3647.frc2025.constants.FieldConstants;
import team3647.frc2025.constants.FieldConstants.ScoringPos;
import team3647.frc2025.constants.GlobalConstants;
import team3647.frc2025.constants.IndexerConstants;
import team3647.frc2025.constants.LEDConstants;
import team3647.frc2025.constants.PivotConstants;
import team3647.frc2025.constants.RollersConstants;
import team3647.frc2025.constants.SwerveDriveConstants;
import team3647.frc2025.constants.VisionConstants;
import team3647.frc2025.constants.WristConstants;
import team3647.frc2025.subsystems.Climb;
import team3647.frc2025.subsystems.Drivetrain.SwerveDriveSim;
import team3647.frc2025.subsystems.Elevator.Elevator;
import team3647.frc2025.subsystems.Elevator.SimElevator;
import team3647.frc2025.subsystems.Coraler;
import team3647.frc2025.subsystems.LEDs;
import team3647.frc2025.subsystems.Rollers;
import team3647.frc2025.subsystems.Seagull;
import team3647.frc2025.subsystems.pivot.Pivot;
import team3647.frc2025.subsystems.pivot.SimPivot;
import team3647.frc2025.subsystems.superstructure.SimSuperstructure;
import team3647.frc2025.subsystems.superstructure.Superstructure;
import team3647.frc2025.subsystems.superstructure.Superstructure.Branch;
import team3647.frc2025.subsystems.superstructure.Superstructure.Level;
import team3647.frc2025.subsystems.wrist.SimWrist;
import team3647.frc2025.subsystems.wrist.Wrist;
import team3647.frc2025.Util.AllianceFlip;
import team3647.lib.GroupPrinter;
import team3647.lib.inputs.Joysticks;
import team3647.lib.team9442.AllianceChecker;
import team3647.lib.team9442.AutoChooser;
//import team3647.lib.team9442.AutoChooser;
import team3647.lib.vision.AprilTagLimelight;
import team3647.lib.vision.AprilTagPhotonVision;
import team3647.lib.vision.NeuralDetectorLimelight;
import team3647.lib.vision.SimVision;
import team3647.lib.vision.VisionController;

public class RobotContainer {
    public final Joysticks mainController = new Joysticks(0);
    public final Joysticks coController = new Joysticks(1);

    public final Joysticks controller2 = new Joysticks(2);
    public final Joysticks controller3 = new Joysticks(3);
    public final Joysticks controller4 = new Joysticks(4);

    public Field2d smartDashboardField = new Field2d();

    public RobotContainer() {
        scuffedBindings();
        configureDefaultCommands();
        configureAllianceObservers();
        configureSmartDashboardLogging();

        superstructure.setIsAlignedFunction(autoDrive.isAlignedToReef());
        elevator.setEncoderHeight(ElevatorConstants.kStartingHeight.in(Meters));
        pivot.setEncoderAngle(PivotConstants.kStartingAngle);
        wrist.setEncoderAngle(WristConstants.kStartingAngle);

        CommandScheduler.getInstance()
                .registerSubsystem(
                        swerve,
                        elevator,
                        pivot,
                        coraler,
                        wrist,
                        rollers,
                        climb,
                        seagull,
                        superstructure);
    }

    private void configureAllianceObservers() {
        allianceChecker.registerObservers(
                swerveCommands, swerve, autoCommands, autoDrive, autoChooser);
    } // 0.6324678425924254

    private void scuffedBindings() {
        // real stuff

        controller2.buttonA.whileTrue(superstructure.intake());
        controller2.buttonB.whileTrue(autoDrive.setDriveMode(DriveMode.SCORE));
        controller2.buttonB.onFalse(autoDrive.clearDriveMode());

        seagullCurrent
                .and(() -> !superstructure.intakeCurrent())
                .and(mainController.rightBumper.negate())
                .onTrue(superstructure.handoff().alongWith(superstructure.setPeice()));
        coralerCurrent.and(mainController.rightBumper.negate()).onTrue(superstructure.Stow());

        controller2.buttonA.onFalse(
                superstructure
                        .wristCommands
                        .stow()
                        .alongWith(
                                superstructure.coralerCommands.kill(),
                                superstructure.rollersCommands.kill(),
                                superstructure.stopIntaking())
                        .andThen(superstructure.Stow()));

        controller2.buttonX.whileTrue(
                superstructure.autoScoreByLevel()
                );
        controller2.buttonX.onFalse(
                Commands.parallel(
                superstructure.poopCoral().withTimeout(0.5),
                superstructure.Stow()
                        
                        ));
        controller2.buttonY.whileTrue(
                Commands.run(()-> {
                        swerve.drive(
                                autoDrive.getXSingleTag(DriveMode.SCORE, ScoringPos.A1),
                                autoDrive.getYSingleTag(DriveMode.SCORE, ScoringPos.A1),
                                autoDrive.getRotSingleTag(DriveMode.SCORE, ScoringPos.A1)
                        );
                }, swerve
                ));

        mainController
                .buttonX
                .whileTrue(superstructure.rollersCommands.setOpenLoop(-0.3, 0.3))
                .onFalse(superstructure.rollersCommands.setOpenLoop(0, 0));

        coController.buttonB.whileTrue(
                superstructure
                        .goToState(superstructure::getCurrentState)
                        .alongWith(superstructure.killAll()));

        controller3.buttonA.onTrue(autoDrive.setWantedBranch(Branch.TWO));
        controller3.buttonB.onTrue(autoDrive.setWantedBranch(Branch.ONE));

        // cocontroller selecting the branch you wanna score coral on
        coController
                .leftJoyStickPress
                .whileTrue(superstructure.coralerCommands.spitOut())
                .onFalse(superstructure.coralerCommands.kill());

        autoDrive.isAlignedToReef().debounce(0.1).onTrue(autoDrive.clearDriveMode());

        coController.leftTrigger.whileTrue(superstructure.setWantedLevel(Level.ALGAELOW));
        coController.leftTrigger.onFalse(superstructure.setWantedLevel(Level.HIGH));

        // coController.rightBumper.onTrue(superstructure.setWantedBranch(Branch.TWO));

        controller4.buttonA.onTrue(superstructure.setWantedLevel(Level.HIGH));

        controller4.buttonB.onTrue(superstructure.setWantedLevel(Level.MID));

        controller4.buttonX.onTrue(superstructure.setWantedLevel(Level.LOW));

        controller4.buttonY.onTrue(superstructure.setWantedLevel(Level.TROUGH));

        coController.rightMidButton.onTrue(autoDrive.enableAutoDrive());

        coController.leftMidButton.onTrue(autoDrive.disableAutoDrive());

        coController.rightJoyStickPress.whileTrue(front.setConvergeToMT1());
        coController.rightJoyStickPress.onFalse(front.setConvergeToGyro());

        mainController.dPadUp.whileTrue(climbCommands.climbOut()).onFalse(climbCommands.kill());
        mainController.dPadDown.whileTrue(climbCommands.climbIn()).onFalse(climbCommands.kill());

        mainController.buttonX.onTrue(superstructure.poopCoral());
        controller3.buttonY.onTrue(
                Commands.runOnce(
                        () -> swerve.setRobotPose(new Pose2d(2, 2, Rotation2d.k180deg)), swerve));
    }

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
                superstructure.goToState(() -> SuperstructureState.AlgaeStow));

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
        // intakeUp.onTrue(superstructure.transfer()).onTrue(autoDrive.setDriveMode(DriveMode.NONE));
        mainController.buttonB.whileTrue(superstructure.transfer());
        if (Utils.isSimulation()) {
            mainController.leftJoyStickPress.whileTrue(autoDrive.setDriveMode(DriveMode.SCORE));
            mainController.leftJoyStickPress.onFalse(autoDrive.clearDriveMode());
        } else {
            mainController
                    .buttonA
                    .and(mainController.dPadLeft)
                    .whileTrue(autoDrive.setDriveMode(DriveMode.SCORE));
            mainController.buttonA.and(mainController.dPadLeft).onFalse(autoDrive.clearDriveMode());
        }

        seagullCurrent
                .and(() -> !superstructure.intakeCurrent())
                .and(mainController.rightBumper.negate())
                .onTrue(superstructure.handoff().alongWith(superstructure.setPeice()));
        coralerCurrent.and(mainController.rightBumper.negate()).onTrue(superstructure.Stow());

        mainController.leftBumper.onFalse(
                superstructure
                        .wristCommands
                        .stow()
                        .alongWith(
                                superstructure.coralerCommands.kill(),
                                superstructure.rollersCommands.kill(),
                                superstructure.stopIntaking())
                        .andThen(superstructure.Stow()));

        // mainController
        //         .rightTrigger
        //         .whileTrue(Commands.waitSeconds(0.05).andThen(superstructure.autoScoreByLevel()))
        //         .whileTrue(
        //                 Commands.sequence(
        //                                 superstructure.wristCommands.setAngle(Degree.of(40)),
        //                                 Commands.waitSeconds(0.3),
        //                                 superstructure.wristCommands.setAngle(
        //                                         WristConstants.kStowAngle))
        //                         .alongWith(
        //                                 superstructure.coralerCommands.setOpenLoop(0.1),
        //                                 superstructure
        //                                         .rollersCommands
        //                                         .setOpenLoop(0.1, 0)
        //                                         .withTimeout(1)));

        mainController.rightTrigger.whileTrue(superstructure.autoScoreByLevel());
        mainController.rightTrigger.onFalse(
                superstructure.Stow()
                        .alongWith(
                                superstructure.poopCoral().withTimeout(0.5),
                                superstructure.setNoPeice().withTimeout(0.01)));
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
                        .goToState(superstructure::getCurrentState)
                        .alongWith(superstructure.killAll()));

        // mainController.rightMidButton.whileTrue(
        //         Commands.sequence(
        //                         superstructure.goToStateParalell(() ->
        // SuperstructureState.ToStow),
        //                         superstructure.goToStateParalell(() -> SuperstructureState.Stow))
        //                 .alongWith(superstructure.setNoPeice()));

        // mainController.leftMidButton.whileTrue(
        //         superstructure
        //                 .goToStateParalell(() -> SuperstructureState.ToStow)
        //                 .alongWith(superstructure.setNoPeice()));

        mainController.rightMidButton.onTrue(autoDrive.setWantedBranch(Branch.TWO));
        mainController.leftMidButton.onTrue(autoDrive.setWantedBranch(Branch.ONE));

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

        mainController.buttonX.onTrue(superstructure.poopCoral());
        mainController.dPadRight.onTrue(
                Commands.runOnce(
                        () -> swerve.setRobotPose(new Pose2d(2, 2, Rotation2d.k180deg)), swerve));
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
                        mainController.rightJoyStickPress));
        elevator.setDefaultCommand(superstructure.elevatorCommands.holdPositionAtCall());
        pivot.setDefaultCommand(superstructure.pivotCommands.holdPositionAtCall());
        coraler.setDefaultCommand(superstructure.coralerCommands.kill());
        wrist.setDefaultCommand(superstructure.wristCommands.stow());
        climb.setDefaultCommand(climbCommands.kill());
    }

    @SuppressWarnings("unchecked")
    public final SwerveDriveSim swerve =
            new SwerveDriveSim(
                    SwerveDriveConstants.driveTrainSimulationConfig, GlobalConstants.kDt);

    public final Coraler coraler =
            new Coraler(
                    IndexerConstants.kMaster,
                    0,
                    0,
                    GlobalConstants.kNominalVoltage,
                    GlobalConstants.kDt);

    public final Elevator elevator =
            new SimElevator(
                    ElevatorConstants.kMinHeight.in(Meters),
                    ElevatorConstants.kMaxHeight.in(Meters),
                    GlobalConstants.kNominalVoltage,
                    GlobalConstants.kDt);
    // new ElevatorReal(
    //                 ElevatorConstants.kMaster,
    //                 ElevatorConstants.kSlave,
    //                 ElevatorConstants.kNativeToMeters,
    //                 ElevatorConstants.kNativeToMeters,
    //                 GlobalConstants.kNominalVoltage,
    //                 0,
    //                 ElevatorConstants.kMinHeight.in(Units.Meter),
    //                 ElevatorConstants.kMaxHeight.in(Units.Meter),
    //                 GlobalConstants.kDt);

    public final Pivot pivot =
            new SimPivot(
                    PivotConstants.kMinAngle.in(Radian),
                    PivotConstants.kMaxAngle.in(Radian),
                    elevator::getHeight,
                    PivotConstants.kClearAngle,
                    GlobalConstants.kNominalVoltage,
                    GlobalConstants.kDt);
    // new PivotReal(
    //         PivotConstants.kMaster,
    //         PivotConstants.kMaxAngle,
    //         PivotConstants.kMinAngle,
    //         0,
    //         PivotConstants.kNativeToRad,
    //         PivotConstants.kNativeToRad,
    //         GlobalConstants.kNominalVoltage,
    //         PivotConstants.kClearAngle,
    //         PivotConstants.kLowClearAngle,
    //         elevator::getHeight,
    //         GlobalConstants.kDt,
    //         mainController.rightTrigger);

    public final Wrist wrist =
            new SimWrist(
                    WristConstants.kMaster,
                    WristConstants.kNativeToDeg,
                    WristConstants.kNativeToDeg,
                    GlobalConstants.kNominalVoltage,
                    WristConstants.kMinAngle,
                    WristConstants.kMaxAngle,
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
            new SimSuperstructure(
                    coraler,
                    elevator,
                    pivot,
                    wrist,
                    rollers,
                    seagull,
                    mainController.buttonY,
                    swerve::getOdoPose,
                    swerve.getDTSim());

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
                    () -> {
                                switch(superstructure.getWantedLevel()){
                                        case ALGAEHIGH:
                                                return false;
                                        case ALGAELOW:
                                                return false;
                                        case HIGH:
                                                break;
                                        case LOW:
                                                break;
                                        case MID:
                                                break;
                                        case NONE:
                                                return false;
                                        case TROUGH:
                                                break;
                                        default:
                                                break;

                                }
                                return superstructure.getStateScoreAuto().equalsWithTolerance(superstructure.getCurrentState());
                        } , 
                    FieldConstants.redSources,
                    FieldConstants.blueSources,
                    AutoConstants.slowerXController,
                    AutoConstants.slowerYController,
                    AutoConstants.rotController,
                    FieldConstants.redReefSides,
                    FieldConstants.blueReefSides,
                    detector,
                    front);

    public final SwerveDriveCommands swerveCommands =
            new SwerveDriveCommands(
                    swerve,
                    SwerveDriveConstants.kDrivePossibleMaxSpeedMPS,
                    SwerveDriveConstants.kRotPossibleMaxSpeedRadPerSec);
    public final ClimbCommands climbCommands = new ClimbCommands(climb);

    public final AllianceChecker allianceChecker = new AllianceChecker();

    public final AutoConstants autoConstants = new AutoConstants();

    public final AutoCommands autoCommands =
            new AutoCommands(swerve, superstructure, autoDrive::getAutoVelocities, detector::hasTarget);

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

    // sim purposes
    AprilTagPhotonVision frontPhotonCam =
            new AprilTagPhotonVision(
                    VisionConstants.frontLLName,
                    VisionConstants.FrontLL,
                    VisionConstants.baseStdDevs,
                    pose -> false,
                    VisionConstants.k2025AprilTags);

    // AprilTagPhotonVision frontRight =
    // new AprilTagPhotonVision("frontRight", VisionConstants.kRobotToFrontRight ,
    // VisionConstants.baseStdDevs);

    public final VisionController controller =
            new VisionController(
                    swerve::addVisionData,
                    swerve::shouldAddData,
                    swerve::setRobotPose,
                    backLeft,
                    frontPhotonCam);
    public final RobotTracker tracker = new RobotTracker(superstructure, autoDrive);

    public final SimVision simVision =
            new SimVision(
                    VisionConstants.k2025AprilTags, swerve::getRealPose, backRight, frontPhotonCam);

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
                                    pivot.angleReached(PivotConstants.kStowAngle, Degree.of(40))
                                            && (superstructure.getWantedLevel() == Level.ALGAEHIGH
                                                    || superstructure.getWantedLevel()
                                                            == Level.ALGAELOW))
                    .and(() -> !DriverStation.isAutonomous());

    Trigger algaeReadyToScore = new Trigger(() -> superstructure.getAlgae());

    public Command getAutonomousCommand(){
        return autoChooser.getSelected().getAutoCommand();
    }

}
