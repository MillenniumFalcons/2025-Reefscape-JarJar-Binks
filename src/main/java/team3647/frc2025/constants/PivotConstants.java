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
    public static final Angle kMinAngle = Radian.of(-1.584);
    public static final Angle kLevel1Angle = Radian.of(-0.3266473090985561).minus(Degree.of(10));
    public static final Angle kLevel2Angle = Radian.of(-0.279);
    public static final Angle kLevel3Angle = Radian.of(0.50);
    public static final Angle kLevel4Angle = Radian.of(0.344);

    public static final Angle kL2Prep = Radian.of(-0.1);
    public static final Angle KL1Prep = kLevel1Angle;
    public static final Angle kStartingAngle = kMinAngle;

    public static final Angle klowLevelsStow = Radian.of(-0.045);

    public static final Angle kAlgaeAngleHigh = Radian.of(0.1988);
    public static final Angle kAlgaeAngleLow = Radian.of(0.290);

    public static final Angle kClearAngle = Radian.of(-0.4);
    // lowclearangle = max angle when the pivot is blocked by the intake going up
    public static final Angle kLowClearAngle = Radian.of(0);

    public static final Angle kStowAngle = Degree.of(-90);
    public static final Angle kStowAngleUp = Radian.of(1.3638929658036516);

    public static final double kNativeToRad = 1 / (10.081055 / (Math.PI / 2));

    public static final Angle kMaxAngle = Radian.of(9.60 / kNativeToRad);

    // -1.402692317215491
    public static final Angle kHandoffAngle = Radian.of(-1.407);

    // public static final Angle kBadAngle = Radian.of(0);
    // public static final Angle kBadTolerance = Radian.of(0);

    // multiply by native to get to degs (it's 40.9 rots per 90 degs)

    // wait for intake to get done 38.225652

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
        kMasterConfig.Slot0.withKG(0.3);

        kMasterConfig.MotorOutput.withInverted(InvertedValue.Clockwise_Positive);
        kMasterConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        kMasterConfig.CurrentLimits.StatorCurrentLimit = 60.0;
        kMasterConfig.CurrentLimits.StatorCurrentLimitEnable = true;

        kMasterConfig.SoftwareLimitSwitch.ForwardSoftLimitThreshold =
                kMaxAngle.in(Radian) / kNativeToRad;
        kMasterConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
        kMasterConfig.SoftwareLimitSwitch.ReverseSoftLimitThreshold =
                kMinAngle.in(Radian) / kNativeToRad;
        kMasterConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;

        kMasterConfig.MotionMagic.MotionMagicCruiseVelocity = 120;
        kMasterConfig.MotionMagic.MotionMagicAcceleration = 1200;
        kMasterConfig.MotionMagic.MotionMagicJerk = 5000;
        kMasterConfig.MotionMagic.MotionMagicExpo_kA = 0.10000000149011612;
        kMasterConfig.MotionMagic.MotionMagicExpo_kV = 0.11999999731779099;

        kMaster.getConfigurator().apply(kMasterConfig);
    }
}
