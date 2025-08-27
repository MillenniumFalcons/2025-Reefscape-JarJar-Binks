package team3647.frc2025.constants;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;

public class KickerConstants {
    public static final TalonFX kMaster =
            new TalonFX(GlobalConstants.KickerIds.kMasterId, GlobalConstants.kSubsystemCanbusName);

    public static final TalonFXConfiguration kMasterConfig = new TalonFXConfiguration();

    static {
        kMasterConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

        // kMasterConfig.Slot0.kP = 0.5;

        kMaster.getConfigurator().apply(kMasterConfig);
    }
}
