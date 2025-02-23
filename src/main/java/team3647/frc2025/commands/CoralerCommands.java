package team3647.frc2025.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import team3647.frc2025.subsystems.Coraler;

public class CoralerCommands {

    public Command intake() {
        return Commands.run(() -> coraler.setDutyCycle(-0.8), coraler);
    }

    public Command spitOut() {
        return Commands.run(() -> coraler.setDutyCycle(0.5), coraler);
    }

    public Command stow() {
        return Commands.run(() -> coraler.setDutyCycle(-0.2), coraler);
    }

	public Command kill(){
		return setOpenLoop(0).withTimeout(0.1);
	}

    public Command setOpenLoop(double percent) {
        return Commands.run(() -> coraler.setDutyCycle(percent), coraler);
    }

    public Command setOpenLoop(DoubleSupplier percent) {
        return Commands.run(() -> coraler.setDutyCycle(percent.getAsDouble()), coraler);
    }

    public Trigger current(){
        return new Trigger(() -> coraler.getMasterCurrent() > 50).debounce(0.04);
    }

    public Coraler coraler;

    public CoralerCommands(Coraler coraler) {
        this.coraler = coraler;
    }
}
