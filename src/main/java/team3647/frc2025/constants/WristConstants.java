package team3647.frc2025.constants;

import static edu.wpi.first.units.Units.Degree;
import static edu.wpi.first.units.Units.KilogramSquareMeters;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;

public class WristConstants {
	

	public static final Angle kIntakeAngle = Units.Degree.of(0);
	public static final Angle kStowAngle = Units.Degree.of(122.21940047191667);
	public static final Angle kStartingAngle = Units.Degree.of(0);

	public static final Angle kMaxAngle = Units.Degree.of(128.55);
	public static final Angle kMinAngle = Units.Degree.of(0);

	public static final Angle kHandoffAngle = Degree.of(66.5);


	//90 degs/20 revolutions
	public static final double kNativeToDeg = 90/20.508812;

	public static final TalonFX kMaster = new TalonFX(GlobalConstants.WristIds.kMasterId, GlobalConstants.kSubsystemCanbusName);
	public static final TalonFXConfiguration kMasterConfig = new TalonFXConfiguration();

	static{
		kMasterConfig.Slot0.kP = 0;

		kMasterConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
		kMasterConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
		;
		
		kMasterConfig.SoftwareLimitSwitch.ForwardSoftLimitThreshold = 30;
		kMasterConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
		kMasterConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
		kMasterConfig.SoftwareLimitSwitch.ReverseSoftLimitThreshold = 0;

		kMasterConfig.CurrentLimits.StatorCurrentLimit = 0;
		kMasterConfig.CurrentLimits.StatorCurrentLimitEnable = true;

		kMasterConfig.MotorOutput.PeakForwardDutyCycle = 0;
		kMasterConfig.MotorOutput.PeakReverseDutyCycle = 0;

		kMaster.getConfigurator().apply(kMasterConfig);
		
	}
}
