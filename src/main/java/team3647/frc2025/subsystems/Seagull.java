package team3647.frc2025.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import team3647.lib.TalonFXSubsystem;

public class Seagull extends TalonFXSubsystem {

    public Seagull(TalonFX master, double nominalVoltage, double kDt) {
        super(master, 1, 1, nominalVoltage, kDt);
    }

    public void setOpenLoop(double dutycycle) {
        this.setOpenloop(dutycycle);
    }

    public double getCurrent() {
        return this.getMasterCurrent();
    }

    @Override
    public String getName() {
        return "seagull";
    }
}
