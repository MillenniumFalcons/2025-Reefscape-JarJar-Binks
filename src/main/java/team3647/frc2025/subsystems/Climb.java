package team3647.frc2025.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import team3647.lib.TalonFXSubsystem;

public class Climb extends TalonFXSubsystem {

    public Climb(
            TalonFX master,
            double positionConversion,
            double velocityConversion,
            double nominalVoltage,
            double kDt) {
        super(master, velocityConversion, positionConversion, nominalVoltage, kDt);
    }

    public void setOpenLoop(double percent) {
        super.setOpenloop(percent);
    }

    @Override
    public String getName() {
        return "climb";
    }
}
