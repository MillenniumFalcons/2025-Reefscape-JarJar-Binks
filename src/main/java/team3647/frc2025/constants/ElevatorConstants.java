package team3647.frc2025.constants;

import static edu.wpi.first.units.Units.Meters;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.units.measure.Distance;

public class ElevatorConstants {
    public static final Distance kLevel1Height = Meters.of(0);
    public static final Distance kLevel2Height = Meters.of(0);
    public static final Distance kLevel3Height = Meters.of(0);
    public static final Distance kLevel4Height = Meters.of(0);

    public static final Distance kIntakeHeight = Meters.of(0);
    public static final Distance kStowHeight = Meters.of(0);

    public static final TalonFX kMaster =
            new TalonFX(
                    GlobalConstants.ElevatorIds.kMasterId, GlobalConstants.kSubsystemCanbusName);
    public static final TalonFX kSlave =
            new TalonFX(GlobalConstants.ElevatorIds.kSlaveId, GlobalConstants.kSubsystemCanbusName);

    public static final TalonFXConfiguration kMasterConfig = new TalonFXConfiguration();

	//multiply native by this to get meters
	public static final double nativeToMeters = 0.446 * Math.PI / 15.0; 

    static {
        kMasterConfig.Slot0.withKP(0);
        kMasterConfig.Slot0.withKI(0);
        kMasterConfig.Slot0.withKD(0);
        kMasterConfig.Slot0.withGravityType(GravityTypeValue.Elevator_Static);
        kMasterConfig.Slot0.withKS(0);
        kMasterConfig.Slot0.withKV(0);
        kMasterConfig.Slot0.withKA(0);
        kMasterConfig.Slot0.withKG(0);

        // prob not right, verify
        kMasterConfig.MotorOutput.withInverted(InvertedValue.CounterClockwise_Positive);
        kMasterConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        kMasterConfig.CurrentLimits.StatorCurrentLimit = 90.0;
        kMasterConfig.CurrentLimits.StatorCurrentLimitEnable = true;
        kMasterConfig.CurrentLimits.SupplyCurrentLimit = 70.0;
        kMasterConfig.CurrentLimits.SupplyCurrentLowerLimit = 55.0;
        kMasterConfig.CurrentLimits.SupplyCurrentLowerTime = 1;

        // need figuring out
        kMasterConfig.SoftwareLimitSwitch.ForwardSoftLimitThreshold = 0;
        kMasterConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable = false; // change after real value
        kMasterConfig.SoftwareLimitSwitch.ReverseSoftLimitThreshold = 0;
        kMasterConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable = true; // change after real value

        kMaster.getConfigurator().apply(kMasterConfig);

        // diff inverts should be handled by follower code
        kSlave.getConfigurator().apply(kMasterConfig);
    }
}
