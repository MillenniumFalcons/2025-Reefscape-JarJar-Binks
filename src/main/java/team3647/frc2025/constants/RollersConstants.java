package team3647.frc2025.constants;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;

public class RollersConstants {
    public static final TalonFX kMaster =
            new TalonFX(GlobalConstants.RollersIds.kMasterId, GlobalConstants.kSubsystemCanbusName);
    public static final TalonFX kSeagull =
            new TalonFX(
                    GlobalConstants.RollersIds.kSeagullId, GlobalConstants.kSubsystemCanbusName);

    public static final TalonFXConfiguration kmasterConfg = new TalonFXConfiguration();
    public static final TalonFXConfiguration kSeagullConfig = new TalonFXConfiguration();

    static {
        kmasterConfg.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

        // kmasterConfg.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        kmasterConfg.Slot0.kP = 0.5;

        kMaster.getConfigurator().apply(kmasterConfg);
        kSeagull.getConfigurator().apply(kSeagullConfig);
    }
}
