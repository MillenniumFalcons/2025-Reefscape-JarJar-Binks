package team3647.frc2025.constants;

import static edu.wpi.first.units.Units.Centimeter;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;

import com.ctre.phoenix6.configs.SoftwareLimitSwitchConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.units.measure.Distance;

public class ElevatorConstants {

    public static final double kNativeToMeters = (1.21 - 0.85) / 19.855;

    public static final Distance kStartingHeight = Meters.of(0.85);
    public static final Distance kStowHeight = kStartingHeight.plus(Centimeter.of(5));

    public static final Distance kHandoffHeight = kStowHeight.plus(Inches.of(3));

    public static final Distance kMaxHeight = Meters.of(84.27 * kNativeToMeters);

    // all heights measured as center dist from the pivot pivoting bar
    public static final Distance kLevel1Height = kStartingHeight;
    public static final Distance kLevel2Height = kStartingHeight;
    public static final Distance kLevel3Height = kStartingHeight;
    public static final Distance kLevel4Height = Meters.of(1.529);

    // high needs tuning, low doesn't; tune for same angle at both heights
    public static final Distance kHighAlgaeHeight = Meters.of(1.368);
    public static final Distance kLowAlgaeHeight = kStartingHeight;

    public static final Distance kIntakeHeight = kStartingHeight;

    public static final Distance kMinHeight = kStartingHeight;

    public static final Distance kClearHeight = Meters.of(1.254);

    public static final TalonFX kMaster =
            new TalonFX(
                    GlobalConstants.ElevatorIds.kMasterId, GlobalConstants.kSubsystemCanbusName);
    public static final TalonFX kSlave =
            new TalonFX(GlobalConstants.ElevatorIds.kSlaveId, GlobalConstants.kSubsystemCanbusName);

    public static final TalonFXConfiguration kMasterConfig = new TalonFXConfiguration();

	public static final TalonFXConfiguration kSlaveConfig;

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
                kMaxHeight.in(Meters) / kNativeToMeters; // native units
        kMasterConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
        kMasterConfig.SoftwareLimitSwitch.ReverseSoftLimitThreshold =
                kStartingHeight.in(Meters) / kNativeToMeters; // native units
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
