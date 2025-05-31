package team3647.frc2025.constants;

import static edu.wpi.first.units.Units.Degree;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.StaticFeedforwardSignValue;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;

public class WristConstants {

    public static final Angle kIntakeAngle = Units.Degree.of(2);
    public static final Angle kStowAngle = Units.Degree.of(110);

    public static final Angle kStartingAngle = Degree.of(116); // Units.Degree.of(125.38)

    public static final Pose3d kZeroedIntakePose = new Pose3d(
        -.2816606,
        0,
        .28575,
        new Rotation3d(
            0,
            0,
            0
        )
    );

 

    public static final Angle kMaxAngle = kStartingAngle;
    public static final Angle kMinAngle = Units.Degree.of(-7);

    public static final Angle kHandoffAngle = Degree.of(35);

    public static final Angle kStowWithPiece = Degree.of(55);

    public static final double kGearRatio = 42;
    public static final double kArmLengthM = 0.40132;
    public static final double kArmMassKg = 4.876;
    public static final double kArmMoiKg2M = 0.017847302;


    public static final Angle idrc = Degree.of(-100);

    // tunnnnnoon
    public static final Angle kSourceIntakeAngle = Degree.of(90);

    // 90 degs/20 revolutions
    public static final double kNativeToDeg = 90 / 10.9633;

    public static final TalonFX kMaster =
            new TalonFX(GlobalConstants.WristIds.kMasterId, GlobalConstants.kSubsystemCanbusName);
    public static final TalonFXConfiguration kMasterConfig = new TalonFXConfiguration();

    static {
        kMasterConfig.Slot0.kP = 20;
        kMasterConfig.Slot0.kD = 0.2;
        kMasterConfig.Slot0.kS = 0.1;
        kMasterConfig.Slot0.StaticFeedforwardSign = StaticFeedforwardSignValue.UseVelocitySign;
        kMasterConfig.Slot0.GravityType = GravityTypeValue.Arm_Cosine;

        kMasterConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        kMasterConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        kMasterConfig.MotionMagic.MotionMagicExpo_kA = 0.0500000001490;
        kMasterConfig.MotionMagic.MotionMagicExpo_kV = 0.119;

        kMasterConfig.SoftwareLimitSwitch.ForwardSoftLimitThreshold =
                kMaxAngle.in(Degree) / kNativeToDeg;
        kMasterConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
        kMasterConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
        kMasterConfig.SoftwareLimitSwitch.ReverseSoftLimitThreshold = 0;

        kMasterConfig.CurrentLimits.StatorCurrentLimit = 30;
        kMasterConfig.CurrentLimits.StatorCurrentLimitEnable = true;

        kMaster.getConfigurator().apply(kMasterConfig);
    }
}
