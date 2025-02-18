package team3647.frc2025.commands;

import static edu.wpi.first.units.Units.Degree;
import static edu.wpi.first.units.Units.Radian;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.Subsystem;

import java.util.Set;
import java.util.function.DoubleSupplier;

import team3647.frc2025.constants.PivotConstants;
import team3647.frc2025.subsystems.Pivot;

public class PivotCommands {

    public Command setAngle(Angle angleRads) {
        return Commands.run(() -> pivot.setAngle(angleRads), pivot)
                .until(() -> pivot.angleReached(angleRads, Degree.of(2)));
    }

	public Command setAngle(double angleRads) {
        return Commands.run(() -> pivot.setAngleRads(angleRads), pivot)
                .until(() -> pivot.angleReached(Radian.of(angleRads), Degree.of(2)));
    }

    public Command setOpenLoop(DoubleSupplier out) {
        return Commands.run(() -> pivot.setOpenLoop(out.getAsDouble()), pivot);
    }

	public Command holdPositionAtCall() {
        return new Command() {
            Angle degreeAtStart = PivotConstants.kStartingAngle;

            @Override
            public void initialize() {
                degreeAtStart = pivot.getAngle();
            }

            @Override
            public void execute() {
                pivot.setAngle(degreeAtStart);
            }

            @Override
            public Set<Subsystem> getRequirements() {
                return Set.of(pivot);
            }
		};
	}

	
    final Pivot pivot;

    public PivotCommands(Pivot pivot) {
        this.pivot = pivot;
    }
}
