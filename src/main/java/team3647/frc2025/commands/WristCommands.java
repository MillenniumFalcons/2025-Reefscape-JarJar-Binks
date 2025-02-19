package team3647.frc2025.commands;

import static edu.wpi.first.units.Units.Degree;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.Subsystem;
import java.util.Set;
import team3647.frc2025.constants.WristConstants;
import team3647.frc2025.subsystems.Wrist;

public class WristCommands {
    public Command setAngle(Angle angleRads) {
        return Commands.run(() -> wrist.setAngle(angleRads), wrist)
                .until(() -> wrist.angleReached(angleRads, Degree.of(2)));
    }

    public Command setOpenLoop(double output) {
        return Commands.run(() -> wrist.setOpenLoop(output), wrist);
    }

    public Command holdPositionAtCall() {
        return new Command() {
            Angle degreeAtStart = WristConstants.kStartingAngle;

            @Override
            public void initialize() {
                degreeAtStart = wrist.getAngle();
            }

            @Override
            public void execute() {
                wrist.setAngle(degreeAtStart);
            }

            @Override
            public Set<Subsystem> getRequirements() {
                return Set.of(wrist);
            }
        };
    }

    final Wrist wrist;

    public WristCommands(Wrist wrist) {
        this.wrist = wrist;
    }
}
