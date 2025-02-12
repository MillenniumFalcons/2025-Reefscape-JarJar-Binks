package team3647.frc2025.constants;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meter;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Rotation;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.units.measure.Distance;

public class ElevatorConstants {

	public static final double kNativeToMeters = (1.21-0.85)/19.855; 

	public static final Distance kStartingHeight = Meters.of(0.85);

	public static final Distance kMaxHeight = Meters.of(83.87989807128906 * kNativeToMeters);

	//all heights measured as center dist from the pivot pivoting bar
    public static final Distance kLevel1Height = kStartingHeight;
    public static final Distance kLevel2Height = kStartingHeight;
    public static final Distance kLevel3Height = kStartingHeight
	;
    public static final Distance kLevel4Height = Meters.of(1.459962658964996);

    public static final Distance kIntakeHeight = Meters.of(0);
    public static final Distance kStowHeight = Meters.of(0);

	public static final Distance kMinHeight = Meters.of(0);

	

    public static final TalonFX kMaster =
            new TalonFX(
                    GlobalConstants.ElevatorIds.kMasterId, GlobalConstants.kSubsystemCanbusName);
    public static final TalonFX kSlave =
            new TalonFX(GlobalConstants.ElevatorIds.kSlaveId, GlobalConstants.kSubsystemCanbusName);

    public static final TalonFXConfiguration kMasterConfig = new TalonFXConfiguration();

	//multiply native by this to get meters 121 - 85 cm @ 19.855 native



    static {
		//slot 0  = voltage configs
        kMasterConfig.Slot0.withKP(3);
        kMasterConfig.Slot0.withKI(0);
        kMasterConfig.Slot0.withKD(0);
        kMasterConfig.Slot0.withGravityType(GravityTypeValue.Elevator_Static);
        kMasterConfig.Slot0.withKS(0.048); 
        kMasterConfig.Slot0.withKV(0);
        kMasterConfig.Slot0.withKA(0);
        kMasterConfig.Slot0.withKG(0.379);
		//121cm - 85cm @19.855
		
//wanted pos: 40

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
		
        
        kMasterConfig.SoftwareLimitSwitch.ForwardSoftLimitThreshold = 37 + (kStartingHeight.in(Meter)/kNativeToMeters); //native units
        kMasterConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable = true; 
        kMasterConfig.SoftwareLimitSwitch.ReverseSoftLimitThreshold = kStartingHeight.in(Meters)/kNativeToMeters; // native units
        kMasterConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable = true; 

		


        kMaster.getConfigurator().apply(kMasterConfig);

        // diff inverts should be handled by follower code
        kSlave.getConfigurator().apply(kMasterConfig);
    }
}
