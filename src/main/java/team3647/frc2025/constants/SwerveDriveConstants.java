package team3647.frc2025.constants;

import static edu.wpi.first.units.Units.Meters;

import com.ctre.phoenix6.configs.*;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.ctre.phoenix6.signals.StaticFeedforwardSignValue;
import com.pathplanner.lib.config.ModuleConfig;
import com.pathplanner.lib.config.RobotConfig;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.DriverStation;
import org.ironmaple.simulation.drivesims.COTS;
import team3647.lib.team254.geometry.Translation2d;
import team3647.lib.team254.swerve.SwerveDriveKinematics;
import team3647.lib.team254.swerve.SwerveKinematicLimits;

public class SwerveDriveConstants {
    // default falcon rotates counter clockwise (CCW)
    // make sure gyro -CW, +CCW
    public static final SensorDirectionValue canCoderInvert =
            SensorDirectionValue.CounterClockwise_Positive;
    public static final boolean kDriveMotorInvertedLeftSide = false;
    public static final boolean kDriveMotorInvertedRightSide = true;
    public static final boolean kTurnMotorInvertedBoolean = false;

    // physical possible max speed
    public static final double kDrivePossibleMaxSpeedMPS = 5.6;
    public static final double kRotPossibleMaxSpeedRadPerSec = 11.2;

    // public static final TalonFX kFrontLeftDrive =
    //         new TalonFX(GlobalConstants.SwerveDriveIds.kFrontLeftDriveId, "rio");
    // public static final TalonFX kFrontLeftTurn =
    //         new TalonFX(GlobalConstants.SwerveDriveIds.kFrontLeftTurnId, "rio");
    // public static final CANcoder kFrontLeftAbsEncoder =
    //         new CANcoder(GlobalConstants.SwerveDriveIds.kFrontLeftAbsEncoderPort, "rio");

    // public static final TalonFX kFrontRightDrive =
    //         new TalonFX(GlobalConstants.SwerveDriveIds.kFrontRightDriveId, "rio");
    // public static final TalonFX kFrontRightTurn =
    //         new TalonFX(GlobalConstants.SwerveDriveIds.kFrontRightTurnId, "rio");
    // public static final CANcoder kFrontRightAbsEncoder =
    //         new CANcoder(GlobalConstants.SwerveDriveIds.kFrontRightAbsEncoderPort, "rio");

    // public static final TalonFX kBackLeftDrive =
    //         new TalonFX(GlobalConstants.SwerveDriveIds.kBackLeftDriveId, "rio");
    // public static final TalonFX kBackLeftTurn =
    //         new TalonFX(GlobalConstants.SwerveDriveIds.kBackLeftTurnId, "rio");
    // public static final CANcoder kBackLeftAbsEncoder =
    //         new CANcoder(GlobalConstants.SwerveDriveIds.kBackLeftAbsEncoderPort, "rio");

    // public static final TalonFX kBackRightDrive =
    //         new TalonFX(GlobalConstants.SwerveDriveIds.kBackRightDriveId, "rio");
    // public static final TalonFX kBackRightTurn =
    //         new TalonFX(GlobalConstants.SwerveDriveIds.kBackRightTurnId, "rio");
    // public static final CANcoder kBackRightAbsEncoder =
    //         new CANcoder(GlobalConstants.SwerveDriveIds.kBackRightAbsEncoderPort, "rio");

    // public static final Pigeon2 kGyro = new Pigeon2(GlobalConstants.SwerveDriveIds.gyroPin);

    // config swerve module reversed here, module class doens't reverse for you

    // distance between right and left wheels
    public static final double kTrackWidth = Units.Inches.of(22).in(Units.Meter);
    // distance between front and back wheels

    public static final double kWheelBase = Units.Inches.of(22).in(Units.Meter);
    // translations are locations of each module wheel
    // 0 --> ++ --> front left
    // 1 --> +- --> front right
    // 2 --> -+ --> back left
    // 3 --> -- --> back right
    // c is center of robot,
    // +x towards front of robot, +y towards left of robot
    // +x
    // ^
    // |
    // +y<--c
    public static final SwerveDriveKinematics kDriveKinematics =
            new SwerveDriveKinematics(
                    new Translation2d(kWheelBase / 2.0, kTrackWidth / 2.0),
                    new Translation2d(kWheelBase / 2.0, -kTrackWidth / 2.0),
                    new Translation2d(-kWheelBase / 2.0, kTrackWidth / 2.0),
                    new Translation2d(-kWheelBase / 2.0, -kTrackWidth / 2.0));

    public static final SwerveKinematicLimits kTeleopKinematicLimits = new SwerveKinematicLimits();

    // public static final double defaultAccel = TunerConstants.kSpeedAt12VoltsMps / 0.1;
    public static final double shootingAccel = 13;

    static {
        kTeleopKinematicLimits.kMaxDriveVelocity = 5.6; // TunerConstants.kSpeedAt12VoltsMps;
        kTeleopKinematicLimits.kMaxDriveAcceleration = 5.6 / 0.1; // defaultAccel;
        kTeleopKinematicLimits.kMaxSteeringVelocity = Units.Degree.of(1500).in(Units.Radians);
    }

    public static ModuleConfig ppModuleConfig =
            new ModuleConfig(
                    Meters.of(TunerConstants.BackLeft.WheelRadius),
                    TunerConstants.kSpeedAt12Volts,
                    COTS.WHEELS.DEFAULT_NEOPRENE_TREAD.cof,
                    DCMotor.getKrakenX60Foc(1).withReduction(5.684210526315789),
                    Units.Amps.of(90),
                    1);

    public static RobotConfig ppRobotConfig;

    static {
        try {
            ppRobotConfig = RobotConfig.fromGUISettings();
        } catch (Exception e) {
            ppRobotConfig = new RobotConfig(74, 6.883, ppModuleConfig, kTrackWidth);

            DriverStation.reportError(
                    "problem setting pp robot config from gui", e.getStackTrace());
        }
    }

    // config conversion factors here for each module. in meters for postiion and
    // radians for
    // rotation.

    // from motor to output shaft
    public static final double kDriveMotorGearRatio = 1 / 6.12;
    public static final double kTurnMotorGearRatio = 1 / 12.8; // 7.0 / 150.0;
    public static final double kCouplingGearRatio = 3.57;
    public static final double kWheelDiameterMeters = 0.097; // 97mm
    public static final double kWheelRadiusInches = 1.9;

    // // divide for tick to deg
    public static final double kTurnMotorNativeToDeg = kTurnMotorGearRatio * 360.0;

    public static final double kTurnMotorNativeToDPS = kTurnMotorNativeToDeg; // RPS / Native/10ms

    public static final double kWheelRotationToMetersDrive =
            kWheelDiameterMeters * Math.PI * kDriveMotorGearRatio;

    // Multiply by 10 because velocity is in ticks/100ms
    public static final double kFalconVelocityToMpS = kWheelRotationToMetersDrive;

    public static final double kFalconTicksToMeters = kWheelRotationToMetersDrive;

    public static final double kNominalVoltage = 10;
    public static final double kStallCurrent = 35;
    public static final double kMaxCurrent = 60;

    // comp bot
    private static final double kFrontLeftEncoderOffset = -0.738037109375;
    private static final double kFrontRightEncoderOffset = -0.18115234375;
    private static final double kBackLeftEncoderOffset = -0.618408203125;
    private static final double kBackRightEncoderOffset = -0.34228515625;

    // max speed limits that we want
    public static final double kTeleopDriveMaxAccelUnitsPerSec = kDrivePossibleMaxSpeedMPS / 2;
    public static final double kTeleopDriveMaxAngularAccelUnitsPerSec =
            kRotPossibleMaxSpeedRadPerSec / 3;

    // public static final Pigeon2Configurator kGyroConfig = kGyro.getConfigurator();

    // master FF for drive for all modules
    public static final double kS = 0.22; // (0.56744 / 12); // 0.56744; // Volts
    public static final double kV = 0.47; // (2.5 / 12.0); // Volts
    public static final double kA = 0.0; // (0.0 / 12); // Volts

    public static final SimpleMotorFeedforward kMasterDriveFeedforward =
            new SimpleMotorFeedforward(kS, kV, kA);

    // master PID constants for turn and drive for all modules
    public static final double kDriveP = 0.02; // 0.00014;
    public static final double kDriveI = 0.0;
    public static final double kDriveD = 0.0;

    public static final Slot0Configs kDriveGains =
            new Slot0Configs()
                    .withKP(kDriveP)
                    .withKI(kDriveI)
                    .withKD(kDriveD)
                    .withKS(kS)
                    .withKA(kA)
                    .withKV(kV);

    public static final double kTurnP = 0.4;
    public static final double kTurnI = 0.0;
    public static final double kTurnD = 0;

    public static final Slot0Configs kTurnGains =
            new Slot0Configs().withKP(kTurnP).withKI(kTurnI).withKD(kTurnD);

    public static final double kYP = 1;
    public static final double kYI = 0.0;
    public static final double kYD = 0;

    public static final PIDController kYController = new PIDController(kYP, kYI, kYD);

    public static final PIDController kAutoSteerXYPIDController = new PIDController(0.05, 0, 0);
    // 3*Pi = move at 10 rads per second if we are 180* away from target heading
    public static final PIDController kAutoSteerHeadingController = new PIDController(0.03, 0, 0);

    public static final Slot0Configs BRSteerGains =
            new Slot0Configs()
                    .withKP(12.003)
                    .withKI(0)
                    .withKD(0.20649)
                    .withKS(0.043482)
                    .withKV(1.3384)
                    .withKA(0.022826)
                    .withStaticFeedforwardSign(StaticFeedforwardSignValue.UseClosedLoopSign);

    public static final Slot0Configs FLSteerGains =
            new Slot0Configs()
                    .withKP(16.447)
                    .withKI(0)
                    .withKD(0.31333)
                    .withKS(0.0023325)
                    .withKV(1.3182)
                    .withKA(0.028693)
                    .withStaticFeedforwardSign(StaticFeedforwardSignValue.UseClosedLoopSign);

    public static final Slot0Configs BLSteerGains =
            new Slot0Configs()
                    .withKP(12.003)
                    .withKI(0)
                    .withKD(0.20649)
                    .withKS(0.0083355)
                    .withKV(1.3171)
                    .withKA(0.020266)
                    .withStaticFeedforwardSign(StaticFeedforwardSignValue.UseClosedLoopSign);

    public static final Slot0Configs FRSteerGains =
            new Slot0Configs()
                    .withKP(8.353)
                    .withKI(0)
                    .withKD(0.53316)
                    .withKS(0.046629)
                    .withKV(1.3239)
                    .withKA(0.030673)
                    .withStaticFeedforwardSign(StaticFeedforwardSignValue.UseClosedLoopSign);

    // PID constants for roll and yaw

    // private static final SwerveModuleConstantsFactory ConstantCreator =
    //         new SwerveModuleConstantsFactory()
    //                 .withDriveMotorGearRatio(kDriveMotorGearRatio)
    //                 .withSteerMotorGearRatio(kTurnMotorGearRatio)
    //                 .withWheelRadius(kWheelRadiusInches)
    //                 .withSlipCurrent(300)
    //                 .withSteerMotorGains(kTurnGains)
    //                 .withDriveMotorGains(kDriveGains)
    //                 .withSteerMotorClosedLoopOutput(ClosedLoopOutputType.Voltage)
    //                 .withDriveMotorClosedLoopOutput(ClosedLoopOutputType.Voltage)
    //                 .withSpeedAt12Volts(
    //                         LinearVelocity.ofRelativeUnits(
    //                                 kDrivePossibleMaxSpeedMPS,
    //                                 edu.wpi.first.units.Units.MetersPerSecond))
    //                 .withFeedbackSource(SteerFeedbackType.FusedCANcoder)
    //                 .withCouplingGearRatio(kCouplingGearRatio);

    // is stored as reference?

    // public static final SwerveDrivetrainConstants kDrivetrainConstants =
    //         new SwerveDrivetrainConstants()
    //                 .withPigeon2Id(kGyro.getDeviceID())
    //                 .withCANBusName("rio");

    // public static final SwerveModuleConstants kFrontLeftConstants =
    //         ConstantCreator.createModuleConstants(
    //                 kFrontLeftTurn.getDeviceID(),
    //                 kFrontLeftDrive.getDeviceID(),
    //                 kFrontLeftAbsEncoder.getDeviceID(),
    //                 kFrontLeftEncoderOffset,
    //                 Units.Inches.of(kWheelBase / 2.0).in(Units.Meters),
    //                 Units.Inches.of(kTrackWidth / 2.0).in(Units.Meters),
    //                 kDriveMotorInvertedLeftSide,
    //                 kTurnMotorInvertedBoolean);
    // public static final SwerveModuleConstants kFrontRightConstants =
    //         ConstantCreator.createModuleConstants(
    //                 kFrontRightTurn.getDeviceID(),
    //                 kFrontRightDrive.getDeviceID(),
    //                 kFrontRightAbsEncoder.getDeviceID(),
    //                 kFrontRightEncoderOffset,
    //                 Units.Inches.of(kWheelBase / 2.0).in(Units.Meters),
    //                 Units.Inches.of(-kTrackWidth / 2.0).in(Units.Meters),
    //                 kDriveMotorInvertedRightSide,
    //                 kTurnMotorInvertedBoolean);
    // public static final SwerveModuleConstants kBackLeftConstants =
    //         ConstantCreator.createModuleConstants(
    //                 kBackLeftTurn.getDeviceID(),
    //                 kBackLeftDrive.getDeviceID(),
    //                 kBackLeftAbsEncoder.getDeviceID(),
    //                 kBackLeftEncoderOffset,
    //                 Units.Inches.of(-kWheelBase / 2.0).in(Units.Meters),
    //                 Units.Inches.of(kTrackWidth / 2.0).in(Units.Meters),
    //                 kDriveMotorInvertedLeftSide,
    //                 kTurnMotorInvertedBoolean);
    // public static final SwerveModuleConstants kBackRightConstants =
    //         ConstantCreator.createModuleConstants(
    //                 kBackRightTurn.getDeviceID(),
    //                 kBackRightDrive.getDeviceID(),
    //                 kBackRightAbsEncoder.getDeviceID(),
    //                 kBackRightEncoderOffset,
    //                 Units.Inches.of(-kWheelBase / 2.0).in(Units.Meters),
    //                 Units.Inches.of(-kTrackWidth / 2.0).in(Units.Meters),
    //                 kDriveMotorInvertedRightSide,
    //                 kTurnMotorInvertedBoolean);

    // private static void printError(StatusCode error) {
    //     if (error.value == 0) {
    //         return;
    //     }

    //     System.out.println(error);
    // }

    static double driveMOI =
            Units.Pound.of(120).in(Units.Kilogram)
                    * (1 / 6)
                    * Units.Inch.of(19).in(Units.Meter)
                    * Units.Inch.of(19).in(Units.Meter);

    // public static final DriveTrainSimulationConfig simConfig =
    //         DriveTrainSimulationConfig.Default()
    //                 .withTrackLengthTrackWidth(Units.Inch.of(27), Units.Inches.of(27))
    //                 .withGyro(COTS.ofPigeon2())
    //                 .withRobotMass(Units.Pound.of(125))
    //                 .withSwerveModule(
    //                         new SwerveModuleSimulationConfig(
    //                                 DCMotor.getFalcon500(1),
    //                                 DCMotor.getFalcon500(1),
    //                                 TunerConstants.FrontLeft.DriveMotorGearRatio,
    //                                 TunerConstants.FrontLeft.SteerMotorGearRatio,
    //                                 Volts.of(TunerConstants.FrontLeft.DriveFrictionVoltage),
    //                                 Volts.of(TunerConstants.FrontLeft.SteerFrictionVoltage),
    //                                 Inches.of(2),
    //
    // KilogramSquareMeters.of(TunerConstants.FrontLeft.SteerInertia),
    //                                 COTS.WHEELS.DEFAULT_NEOPRENE_TREAD.cof))
    //                 .withBumperSize(Units.Inches.of(32), Units.Inch.of(32));

    // public static final DriveTrainSimulationConfig kBetterSimConfig =
    //         DriveTrainSimulationConfig.Default()
    //                 .withTrackLengthTrackWidth(Units.Inch.of(19), Units.Inches.of(19))
    //                 .withGyro(COTS.ofPigeon2())
    //                 .withRobotMass(Units.Pound.of(120))
    //                 .withSwerveModule(
    //                         COTS.ofMark4i(
    //                                 DCMotor.getKrakenX60Foc(1),
    //                                 DCMotor.getFalcon500Foc(1),
    //                                 COTS.WHEELS.DEFAULT_NEOPRENE_TREAD.cof,
    //                                 4))
    //                 .withBumperSize(Units.Inches.of(32), Units.Inch.of(32));

    static {
    }

    private SwerveDriveConstants() {}
}
