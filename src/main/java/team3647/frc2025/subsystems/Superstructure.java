package team3647.frc2025.subsystems;

import static edu.wpi.first.units.Units.Degree;

import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import java.util.function.BooleanSupplier;
import team3647.frc2025.Util.SuperstructureState;
import team3647.frc2025.commands.CoralerCommands;
import team3647.frc2025.commands.ElevatorCommands;
import team3647.frc2025.commands.PivotCommands;
import team3647.frc2025.constants.ElevatorConstants;
import team3647.frc2025.constants.FieldConstants.ScoringPos;
import team3647.frc2025.constants.PivotConstants;

public class Superstructure {

    public final Coraler coraler;
    public final Elevator elevator;
    public final Pivot pivot;

    public final CoralerCommands coralerCommands;
    public final ElevatorCommands elevatorCommands;
    public final PivotCommands pivotCommands;

    private final BooleanSupplier isAligned;

    private Level wantedLevel;

    public ScoringPos wantedScoringPos = ScoringPos.NONE;

    private Side wantedSide;

    private Branch wantedBranch;

    public enum Side {
        A,
        B,
        C,
        D,
        E,
        F
    }

    public enum Branch {
        ONE,
        TWO
    }

    public Superstructure(
            Coraler coraler, Elevator elevator, Pivot pivot, BooleanSupplier isAligned) {
        this.coraler = coraler;
        this.elevator = elevator;
        this.pivot = pivot;

        this.coralerCommands = new CoralerCommands(this.coraler);
        this.elevatorCommands = new ElevatorCommands(this.elevator);
        this.pivotCommands = new PivotCommands(this.pivot);

        this.wantedLevel = Level.NONE;
        this.isAligned = isAligned;
    }

    public enum Level {
        TROUGH,
        LOW,
        MID,
        HIGH,
        INTAKE,
        NONE
    }

    public SuperstructureState getWantedStateByLevel(Level level) {
        switch (level) {
            case TROUGH:
                return new SuperstructureState(
                        PivotConstants.kLevel1Angle, ElevatorConstants.kLevel1Height, -0.7);
            case LOW:
                return new SuperstructureState(
                        PivotConstants.kLevel2Angle, ElevatorConstants.kLevel2Height, -0.7);
            case MID:
                return new SuperstructureState(
                        PivotConstants.kLevel3Angle, ElevatorConstants.kLevel3Height, -0.7);
            case HIGH:
                return new SuperstructureState(
                        PivotConstants.kLevel4Angle, ElevatorConstants.kLevel4Height, -0.7);
            case INTAKE:
                return new SuperstructureState(
                        PivotConstants.kIntakeAngle, ElevatorConstants.kIntakeHeight, 1);
            default:
                return new SuperstructureState(
                        PivotConstants.kStowAngle, ElevatorConstants.kStowHeight, 0);
        }
    }

    public SuperstructureState getWantedStateByWantedLevel() {
        return getWantedStateByLevel(wantedLevel);
    }

    /**
     * NOTE: WILL CHANGE BASED ON INTAKE GEOMETRY
     *
     * @return
     */
    public Command goToStateParalell(SuperstructureState state) {
        return Commands.parallel(
                elevatorCommands.setHeight(state.elevatorHeight),
                pivotCommands.setAngle(state.pivotAngle),
                coralerCommands.setOpenLoop(state.rollers).withTimeout(0.5));
    }

    // TUNE MINUS VALUE ONCE ROBOT
    /**
     * For Scoring only
     *
     * @param state
     * @return
     */
    public Command goToStatePerpendicular(SuperstructureState state) {
        return Commands.sequence(
                pivotCommands.setAngle(state.pivotAngle),
                Commands.waitUntil(isAligned),
                elevatorCommands.setHeight(state.elevatorHeight),
                pivotCommands.setAngle(state.pivotAngle.minus(Degree.of(10))),
                Commands.waitSeconds(0.01), // basically 0.02
                pivotCommands
                        .setAngle(state.pivotAngle)
                        .alongWith(coralerCommands.setOpenLoop(state.rollers))
                        .withTimeout(Units.Seconds.of(0.6)));
    }

    /**
     * REMEMBER TO TUNE THIS
     *
     * @param state
     * @return
     */
    public Command goToStateIntake(SuperstructureState state) {
        return goToStateParalell(state);
    }

    public Command setWantedBranch(Branch wantedBranch) {
        return Commands.runOnce(
                () -> {
                    this.wantedBranch = wantedBranch;
                });
    }

    public Command setWantedSide(Side wantedSide) {
        return Commands.runOnce(
                () -> {
                    this.wantedSide = wantedSide;
                });
    }

    public Command setWantedLevel(Level wantedLevel) {
        return Commands.runOnce(
                () -> {
                    this.wantedLevel = wantedLevel;
                });
    }

    public Level getWantedLevel() {
        return wantedLevel;
    }

    public Side getWantedSide() {
        return wantedSide;
    }

    public Branch getWantedBranch() {
        return wantedBranch;
    }
}
