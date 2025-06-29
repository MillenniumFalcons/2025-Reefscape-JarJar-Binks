package team3647.frc2025.constants;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Distance;

public class ElevatorConstants {

    public static final double kSpoolDiam = Units.inchesToMeters(2);

    public static final Distance kStartingHeight = Meters.of(0);

    public static final Distance kHandoffHeight = kStartingHeight.plus(Inches.of(8.175));
    public static final Distance kStowHeight = kHandoffHeight.plus(Inches.of(4));
    public static final Distance kMaxHeight = Inches.of(56.375);

    //     public static final double kMaxPossibleHeightM = Units.inchesToMeters(61);
    //     public static final Distance kLowScoreHeight = kStartingHeight.plus(Meters.of(0.03));

    // all heights measured as the pivot point ?
    public static final Distance kNetHeight = Inches.of(56.125);
    public static final Distance kLevel4Height = Inches.of(56.125);
    public static final Distance kThreeHeight = Inches.of(23); // 17.402
    public static final Distance kLevel2Height = Inches.of(7);
    public static final Distance kLevel1Height = Inches.of(0);

    // high needs tuning, low doesn't; tune for same angle at both heights

    // change this one to low algae cuz the old low algae worked for high
    //     public static final Distance kHighAlgaeHeight = Meters.of(1.368);

    public static final Distance kHighAlgaeHeight = kStartingHeight.plus(Meters.of(0.25));
    // carriage bottom to piv point
    public static final Distance annoyinAssCarriageToPivotPointOffest = Inches.of(7.719);

    public static final Distance kCarriageHeight = Inches.of(4.625);
    public static final Distance kStage2Threshold = Inches.of(28).minus(kCarriageHeight);

    public static final Distance kMinHeight = kStartingHeight;

    public static final Distance kClearHeight = kStowHeight;
    // this is when the claw pivots with an algae in it, and the algae has to clear the elevator top
    // bar
    public static final double kAlgaeElevatorClear = Units.inchesToMeters(12);

    public static final TalonFX kMaster =
            new TalonFX(
                    GlobalConstants.ElevatorIds.kMasterId, GlobalConstants.kSubsystemCanbusName);
    public static final TalonFX kSlave =
            new TalonFX(GlobalConstants.ElevatorIds.kSlaveId, GlobalConstants.kSubsystemCanbusName);

    public static final TalonFXConfiguration kMasterConfig = new TalonFXConfiguration();

    public static final TalonFXConfiguration kSlaveConfig;

    public static final double kGearRatio = 5.6667;
    public static final double kElevatorDrumRadius = Units.inchesToMeters(2);
    public static final double kCarriageMass =
            Units.lbsToKilograms(15); // arm & claw + carriage + gearbox
    public static final Pose3d[] kZeroedElevPose =
            new Pose3d[] {
                new Pose3d(0, 0, 0, new Rotation3d()), new Pose3d(0, 0, 0, new Rotation3d())
            };

    // multiply native by this to get meters 121 - 85 cm @ 19.855 native

    static {
        // slot 0  = voltage configs
        kMasterConfig.Slot0.withKP(3);
        kMasterConfig.Slot0.withKI(0);
        kMasterConfig.Slot0.withKD(0);
        kMasterConfig.Slot0.withGravityType(GravityTypeValue.Elevator_Static);
        kMasterConfig.Slot0.withKS(0);
        kMasterConfig.Slot0.withKV(0);
        kMasterConfig.Slot0.withKA(0);
        kMasterConfig.Slot0.withKG(0.22);
        kMasterConfig.MotionMagic.MotionMagicExpo_kV = 0.01;
        kMasterConfig.MotionMagic.MotionMagicExpo_kA = 0.01000000149011612;
        // 121cm - 85cm @19.855

        // wanted pos: 40

        kMasterConfig.MotorOutput.withInverted(InvertedValue.CounterClockwise_Positive);
        kMasterConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        kMasterConfig.CurrentLimits.StatorCurrentLimit = 60.0;
        kMasterConfig.CurrentLimits.StatorCurrentLimitEnable = true;
        kMasterConfig.CurrentLimits.SupplyCurrentLimitEnable = false;
        kMasterConfig.CurrentLimits.SupplyCurrentLimit = 70.0;
        kMasterConfig.CurrentLimits.SupplyCurrentLowerLimit = 55.0;
        kMasterConfig.CurrentLimits.SupplyCurrentLowerTime = 1;

        kMasterConfig.MotionMagic.MotionMagicCruiseVelocity = 100;
        kMasterConfig.MotionMagic.MotionMagicAcceleration = 500;

        kMasterConfig.SoftwareLimitSwitch.ForwardSoftLimitThreshold =
                kMaxHeight.in(Meters) / kSpoolDiam; // native units
        kMasterConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
        kMasterConfig.SoftwareLimitSwitch.ReverseSoftLimitThreshold =
                kStartingHeight.in(Meters) / kSpoolDiam; // native units
        kMasterConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;

        kSlaveConfig = kMasterConfig;
        kSlaveConfig.SoftwareLimitSwitch.ForwardSoftLimitThreshold = 36.5;
        kSlaveConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable = false;
        kSlaveConfig.SoftwareLimitSwitch.ReverseSoftLimitThreshold = 0;
        kSlaveConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable = false;

        kMaster.getConfigurator().apply(kMasterConfig);

        // diff inverts should be handled by follower code
        kSlave.getConfigurator().apply(kSlaveConfig);
    }
}
