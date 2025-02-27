package team3647.frc2025.constants;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

public class ClimbConstants {
    public static final TalonFX kMaster =
            new TalonFX(GlobalConstants.ClimbIds.kMasterId, GlobalConstants.kSubsystemCanbusName);

    public static final TalonFXConfiguration MasterConfig = new TalonFXConfiguration();

    static {
        MasterConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    }
}
