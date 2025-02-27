package team3647.frc2025.constants;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

public class CoralerConstants {

    public static final TalonFX kMaster =
            new TalonFX(GlobalConstants.CoralerIds.kMasterId, GlobalConstants.kSubsystemCanbusName);

    public static final TalonFXConfiguration kMasterConfig = new TalonFXConfiguration();

    static {
        kMasterConfig.MotorOutput.withNeutralMode(NeutralModeValue.Coast);

        kMasterConfig.MotorOutput.withInverted(InvertedValue.CounterClockwise_Positive);

        kMasterConfig.CurrentLimits.StatorCurrentLimit = 60.0;
        kMasterConfig.CurrentLimits.StatorCurrentLimitEnable = true;

        kMaster.getConfigurator().apply(kMasterConfig);
    }
}
