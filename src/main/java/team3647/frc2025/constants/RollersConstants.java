package team3647.frc2025.constants;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;

public class RollersConstants {
    public static final TalonFX kMaster =
            new TalonFX(GlobalConstants.RollersIds.kMasterId, GlobalConstants.kSubsystemCanbusName);

    public static final TalonFXConfiguration kmasterConfg = new TalonFXConfiguration();

    static {
		
        kMaster.getConfigurator().apply(kmasterConfg);
    }
}
