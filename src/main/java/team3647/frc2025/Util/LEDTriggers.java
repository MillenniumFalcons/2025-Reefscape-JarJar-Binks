package team3647.frc2025.Util;

import edu.wpi.first.wpilibj2.command.button.Trigger;
import team3647.frc2025.subsystems.Superstructure;

public class LEDTriggers {

    Superstructure superstructure;

    public LEDTriggers(Superstructure superstructure) {
        this.superstructure = superstructure;
    }

    public final Trigger alignedTrigger = new Trigger(() -> superstructure.isAligned());
    public final Trigger pieceTrigger = new Trigger(() -> superstructure.hasPeice());
    public final Trigger intakingTrigger = new Trigger(() -> superstructure.isIntaking());
    // public final Trigger climbingTrigger = new Trigger(() -> );
}
