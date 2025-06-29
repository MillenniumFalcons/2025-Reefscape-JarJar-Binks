package team3647.frc2025.subsystems.superstructure;

import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import team3647.frc2025.subsystems.Elevator.Elevator;
import team3647.frc2025.subsystems.Coraler;
import team3647.frc2025.subsystems.Rollers;
import team3647.frc2025.subsystems.Seagull;
import team3647.frc2025.subsystems.pivot.Pivot;
import team3647.frc2025.subsystems.wrist.Wrist;

public class SuperstructureReal extends Superstructure {

    public SuperstructureReal(
            Coraler coraler,
            Elevator elevator,
            Pivot pivot,
            Wrist wrist,
            Rollers rollers,
            Seagull seagull,
            Trigger pieceOverride,
            Supplier<Pose2d> robotPose) {
        super(coraler, elevator, pivot, wrist, rollers, seagull, pieceOverride, robotPose);
    }

    @Override
    public String getName() {
        // TODO Auto-generated method stub
        return "Real Supestructure";
    }
}
