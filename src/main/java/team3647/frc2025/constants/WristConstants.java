package team3647.frc2025.constants;

import static edu.wpi.first.units.Units.Degree;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.StaticFeedforwardSignValue;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;

public class WristConstants {

    public static final Angle kIntakeAngle = Units.Degree.of(12);
    public static final Angle kStowAngle = Units.Degree.of(104.5);

    public static final Angle kStartingAngle = Units.Degree.of(119.3658320339946);

    public static final Angle kMaxAngle = kStartingAngle;
    public static final Angle kMinAngle = Units.Degree.of(0);

    public static final Angle kHandoffAngle = Degree.of(70.64007184240722);

    public static final Angle kStowWithPiece = Degree.of(87.68);

    // tunnnnnoon
    public static final Angle kSourceIntakeAngle = Degree.of(100);

    // 90 degs/20 revolutions
    public static final double kNativeToDeg = 90 / 21.9182;

    public static final TalonFX kMaster =
            new TalonFX(GlobalConstants.WristIds.kMasterId, GlobalConstants.kSubsystemCanbusName);
    public static final TalonFXConfiguration kMasterConfig = new TalonFXConfiguration();

    static {
        kMasterConfig.Slot0.kP = 20;
        kMasterConfig.Slot0.kS = 0.31;
        kMasterConfig.Slot0.StaticFeedforwardSign = StaticFeedforwardSignValue.UseVelocitySign;
        kMasterConfig.Slot0.GravityType = GravityTypeValue.Arm_Cosine;

        kMasterConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        kMasterConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        kMasterConfig.MotionMagic.MotionMagicExpo_kA = 0.0500000001490;
		kMasterConfig.MotionMagic.MotionMagicExpo_kV = 0.119;

        kMasterConfig.SoftwareLimitSwitch.ForwardSoftLimitThreshold =
                kMaxAngle.in(Degree) / kNativeToDeg;
        kMasterConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
        kMasterConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
        kMasterConfig.SoftwareLimitSwitch.ReverseSoftLimitThreshold = 0;

        kMasterConfig.CurrentLimits.StatorCurrentLimit = 40;
        kMasterConfig.CurrentLimits.StatorCurrentLimitEnable = true;

        kMaster.getConfigurator().apply(kMasterConfig);
    }
}
