package team3647.frc2025.subsystems.superstructure;

import static edu.wpi.first.units.Units.Degree;
import static edu.wpi.first.units.Units.Inch;
import static edu.wpi.first.units.Units.Meter;
import static edu.wpi.first.units.Units.Radian;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.MutAngle;
import edu.wpi.first.units.measure.MutDistance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import java.util.Map;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;
import team3647.frc2025.Util.InverseKinematics;
import team3647.frc2025.Util.SuperstructureState;
import team3647.frc2025.commands.CoralerCommands;
import team3647.frc2025.commands.ElevatorCommands;
import team3647.frc2025.commands.PivotCommands;
import team3647.frc2025.commands.RollersCommands;
import team3647.frc2025.commands.WristCommands;
import team3647.frc2025.constants.ElevatorConstants;
import team3647.frc2025.constants.FieldConstants.ScoringPos;
import team3647.frc2025.constants.PivotConstants;
import team3647.frc2025.constants.WristConstants;
import team3647.frc2025.subsystems.Indexer;
import team3647.frc2025.subsystems.Rollers;
import team3647.frc2025.subsystems.Seagull;
import team3647.frc2025.subsystems.Elevator.Elevator;
import team3647.frc2025.subsystems.pivot.Pivot;
import team3647.frc2025.subsystems.wrist.Wrist;

public class SuperstructureReal extends Superstructure{

    public SuperstructureReal(
            Indexer coraler,
            Elevator elevator,
            Pivot pivot,
            Wrist wrist,
            Rollers rollers,
            Seagull seagull,
            Trigger pieceOverride) {
                super(coraler, elevator, pivot, wrist, rollers, seagull, pieceOverride);
    }

    @Override
    public String getName() {
        // TODO Auto-generated method stub
        return "Real Supestructure";
    }
}