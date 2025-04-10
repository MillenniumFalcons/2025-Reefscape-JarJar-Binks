package team3647.frc2025.commands;

import static edu.wpi.first.units.Units.Degree;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.Subsystem;
import java.util.Set;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;
import team3647.frc2025.constants.WristConstants;
import team3647.frc2025.subsystems.Wrist;

public class WristCommands {
    public final Wrist wrist;

    private double offset = 0;

    public WristCommands(Wrist wrist) {
        this.wrist = wrist;
    }

    public Command goToIntake() {
        return Commands.run(
                () -> wrist.setAngle(WristConstants.kIntakeAngle.plus(Degree.of(offset))), wrist);
    }

    public Command offsetUp() {
        return Commands.runOnce(() -> offset++);
    }

    public Command offsetDown() {
        return Commands.runOnce(() -> offset--);
    }

    public Command stow() {
        return setAngle(WristConstants.kStowAngle);
    }

    public Command setAngle(Angle angle) {
        return Commands.run(() -> wrist.setAngle(angle), wrist)
                .until(() -> wrist.angleReached(angle.in(Degree), 2));
    }

    public Command setAngle(Supplier<Angle> angle) {
        return Commands.run(() -> wrist.setAngle(angle.get()), wrist)
                .until(() -> wrist.angleReached(angle.get().in(Degree), 2));
    }

    public BooleanSupplier angleBelow(double angle){
        return  () -> wrist.getAngleDegs() < angle;
    }

    public Command holdPositionAtCall() {
        return new Command() {
            Angle degreeAtStart = WristConstants.kStowAngle;

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
}
