package team3647.frc2025.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import team3647.lib.TalonFXSubsystem;

public class Coraler extends TalonFXSubsystem {

    public Coraler(
            TalonFX master,
            double velocityConversion,
            double positionConversion,
            double nominalVoltage,
            double kDt) {
        super(master, velocityConversion, positionConversion, nominalVoltage, kDt);
    }

    public void setDutyCycle(double percent) {
        setOpenloop(percent);
    }

    @Override
    public String getName() {
        return "Indexer";
    }
}
