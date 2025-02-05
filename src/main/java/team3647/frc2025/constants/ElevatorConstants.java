package team3647.frc2025.constants;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.units.measure.Distance;

public class ElevatorConstants {
	//all heights measured as center dist from the pivot pivoting bar
    public static final Distance kLevel1Height = Meters.of(0);
    public static final Distance kLevel2Height = Meters.of(0);
    public static final Distance kLevel3Height = Meters.of(0);
    public static final Distance kLevel4Height = Meters.of(0);

    public static final Distance kIntakeHeight = Meters.of(0);
    public static final Distance kStowHeight = Meters.of(0);

	public static final Distance kMinHeight = Meters.of(0);
	
	public static final Distance kStartingHeight = Meters.of(0.18161);
	public static final Distance kStartingHeightWithBlock = Meters.of(0.3221);
	

    public static final TalonFX kMaster =
            new TalonFX(
                    GlobalConstants.ElevatorIds.kMasterId, GlobalConstants.kSubsystemCanbusName);
    public static final TalonFX kSlave =
            new TalonFX(GlobalConstants.ElevatorIds.kSlaveId, GlobalConstants.kSubsystemCanbusName);

    public static final TalonFXConfiguration kMasterConfig = new TalonFXConfiguration();

	//multiply native by this to get meters 0.7477125 meters / 20 native
	public static final double kNativeToMeters = 0.7477125/19.97; 

    static {
        kMasterConfig.Slot0.withKP(5);
        kMasterConfig.Slot0.withKI(0);
        kMasterConfig.Slot0.withKD(0);
        kMasterConfig.Slot0.withGravityType(GravityTypeValue.Elevator_Static);
        kMasterConfig.Slot0.withKS(0);
        kMasterConfig.Slot0.withKV(0);
        kMasterConfig.Slot0.withKA(0);
        kMasterConfig.Slot0.withKG(0.21);
//wanted pos: 40

        kMasterConfig.MotorOutput.withInverted(InvertedValue.CounterClockwise_Positive);
        kMasterConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        kMasterConfig.CurrentLimits.StatorCurrentLimit = 90.0;
        kMasterConfig.CurrentLimits.StatorCurrentLimitEnable = true;
        kMasterConfig.CurrentLimits.SupplyCurrentLimit = 70.0;
        kMasterConfig.CurrentLimits.SupplyCurrentLowerLimit = 55.0;
        kMasterConfig.CurrentLimits.SupplyCurrentLowerTime = 1;

        // need figuring out
        kMasterConfig.SoftwareLimitSwitch.ForwardSoftLimitThreshold = 75;
        kMasterConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable = false; // change after real value
        kMasterConfig.SoftwareLimitSwitch.ReverseSoftLimitThreshold = 3; // native units
        kMasterConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable = true; // change after real value

		

        kMaster.getConfigurator().apply(kMasterConfig);

        // diff inverts should be handled by follower code
        kSlave.getConfigurator().apply(kMasterConfig);
    }
}
