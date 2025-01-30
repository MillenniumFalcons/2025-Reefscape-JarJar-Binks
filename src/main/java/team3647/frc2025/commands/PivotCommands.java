package team3647.frc2025.commands;

import static edu.wpi.first.units.Units.Degree;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import java.util.function.DoubleSupplier;
import team3647.frc2025.subsystems.Pivot;

public class PivotCommands {

    public Command setAngle(Angle angleRads) {
        return Commands.run(() -> pivot.setAngle(angleRads), pivot)
                .until(() -> pivot.angleReached(angleRads, Degree.of(2)));
    }

    public Command setOpenLoop(DoubleSupplier out) {
        return Commands.run(() -> pivot.setOpenLoop(out.getAsDouble()), pivot);
    }

    final Pivot pivot;

    public PivotCommands(Pivot pivot) {
        this.pivot = pivot;
    }
}
