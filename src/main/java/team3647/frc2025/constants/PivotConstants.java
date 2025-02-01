package team3647.frc2025.constants;

import static edu.wpi.first.units.Units.Degree;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.units.measure.Angle;

public class PivotConstants {
    public static final Angle kLevel1Angle = Degree.of(0);
    public static final Angle kLevel2Angle = Degree.of(0);
    public static final Angle kLevel3Angle = Degree.of(0);
    public static final Angle kLevel4Angle = Degree.of(0);

    public static final Angle kStowAngle = Degree.of(0);
    public static final Angle kIntakeAngle = Degree.of(0);

    public static final Angle kMaxAngle = Degree.of(90);
    public static final Angle kMinAngle = Degree.of(-90);

    // wait for intake to get done

    public static final Angle kMaxLowAngleIntakeUp = Degree.of(0);
    public static final Angle kMinHighAngleIntakeUp = Degree.of(0);

    public static final TalonFX kMaster =
            new TalonFX(GlobalConstants.PivotIds.kMasterId, GlobalConstants.kSubsystemCanbusName);

    public static final TalonFXConfiguration kMasterConfig = new TalonFXConfiguration();

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
        kMasterConfig.MotorOutput.withInverted(InvertedValue.Clockwise_Positive);
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
        kMasterConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable = false; // change after real value

        kMaster.getConfigurator().apply(kMasterConfig);
    }
}
