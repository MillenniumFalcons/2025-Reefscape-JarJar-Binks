package team3647.frc2025.constants;

import static edu.wpi.first.units.Units.Degree;
import static edu.wpi.first.units.Units.Radian;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.units.measure.Angle;

public class PivotConstants {
    public static final Angle kMinAngle = Degree.of(-90);
    public static final Angle kLevel1Angle = Radian.of(-0.3266473090985561);
    public static final Angle kLevel2Angle = Radian.of(0.6324678425924254);
    public static final Angle kLevel3Angle = Radian.of(1.1994256443859082);
    public static final Angle kLevel4Angle = Radian.of(1.2537999814655818);

	public static final Angle kClearAngle = Radian.of(-0.4);
	//lowclearangle = max angle when the pivot is blocked by the intake going up 
	public static final Angle kLowClearAngle = Radian.of(0);

    public static final Angle kStowAngle = kMinAngle;
	public static final Angle kStowAngleUp = Radian.of(0);
    public static final Angle kIntakeAngle = Degree.of(0);

    public static final Angle kMaxAngle = Radian.of(1.6036416572298584);
    

	public static final Angle kStartingAngle = kMinAngle;

    public static final Angle kHandoffAngle = Radian.of(-1.350);

	// public static final Angle kBadAngle = Radian.of(0);
	// public static final Angle kBadTolerance = Radian.of(0);


	//multiply by native to get to degs (it's 40.9 rots per 90 degs)
	public static final double kNativeToRad = 1/(38.226562 / (Math.PI/2));

    // wait for intake to get done 38.225652


    public static final Angle kMaxLowAngleIntakeUp = Degree.of(0);
    public static final Angle kMinHighAngleIntakeUp = Degree.of(0);

    public static final TalonFX kMaster =
            new TalonFX(GlobalConstants.PivotIds.kMasterId, GlobalConstants.kSubsystemCanbusName);

    public static final TalonFXConfiguration kMasterConfig = new TalonFXConfiguration();

    static {
        kMasterConfig.Slot0.withKP(2);
        kMasterConfig.Slot0.withKI(0);
        kMasterConfig.Slot0.withKD(0);
        kMasterConfig.Slot0.withGravityType(GravityTypeValue.Arm_Cosine);
        kMasterConfig.Slot0.withKS(0);
        kMasterConfig.Slot0.withKV(0);
        kMasterConfig.Slot0.withKA(0);
        kMasterConfig.Slot0.withKG(0.40);

        // prob not right, verify
        kMasterConfig.MotorOutput.withInverted(InvertedValue.Clockwise_Positive);
        kMasterConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;

        kMasterConfig.CurrentLimits.StatorCurrentLimit = 90.0;
        kMasterConfig.CurrentLimits.StatorCurrentLimitEnable = true;
        kMasterConfig.CurrentLimits.SupplyCurrentLimit = 70.0;
        kMasterConfig.CurrentLimits.SupplyCurrentLowerLimit = 55.0;
        kMasterConfig.CurrentLimits.SupplyCurrentLowerTime = 1;

        // need figuring out
        kMasterConfig.SoftwareLimitSwitch.ForwardSoftLimitThreshold = kMaxAngle.in(Radian)/kNativeToRad;
        kMasterConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable = false; 
        kMasterConfig.SoftwareLimitSwitch.ReverseSoftLimitThreshold = kMinAngle.in(Radian)/kNativeToRad;
        kMasterConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable = false; 

		kMasterConfig.MotionMagic.MotionMagicCruiseVelocity = 120;
		kMasterConfig.MotionMagic.MotionMagicAcceleration = 1200;
		kMasterConfig.MotionMagic.MotionMagicJerk = 5000;
		kMasterConfig.MotionMagic.MotionMagicExpo_kA = 0.10000000149011612;
		kMasterConfig.MotionMagic.MotionMagicExpo_kV = 0.11999999731779099;


        kMaster.getConfigurator().apply(kMasterConfig);
    }
}
