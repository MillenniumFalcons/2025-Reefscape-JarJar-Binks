package team3647.frc2025.subsystems;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import java.util.function.BooleanSupplier;
import org.littletonrobotics.junction.Logger;
import team3647.frc2025.Util.SuperstructureState;
import team3647.frc2025.commands.CoralerCommands;
import team3647.frc2025.commands.ElevatorCommands;
import team3647.frc2025.commands.PivotCommands;
import team3647.frc2025.commands.WristCommands;
import team3647.frc2025.constants.ElevatorConstants;
import team3647.frc2025.constants.FieldConstants.ScoringPos;
import team3647.frc2025.constants.PivotConstants;

public class Superstructure {

    private final Coraler coraler;
    private final Elevator elevator;
    private final Pivot pivot;
    private final Wrist wrist;

    public final CoralerCommands coralerCommands;
    public final ElevatorCommands elevatorCommands;
    public final PivotCommands pivotCommands;
    public final WristCommands wristCommands;

    private BooleanSupplier isAligned;

    private Level wantedLevel;

    private SuperstructureState wantedSuperstructureState = SuperstructureState.kInvalidState;

    private ScoringPos wantedScoringPos = ScoringPos.NONE;

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

    public Superstructure(Coraler coraler, Elevator elevator, Pivot pivot, Wrist wrist) {
        this.coraler = coraler;
        this.elevator = elevator;
        this.pivot = pivot;
        this.wrist = wrist;

        this.coralerCommands = new CoralerCommands(this.coraler);
        this.elevatorCommands = new ElevatorCommands(this.elevator);
        this.pivotCommands = new PivotCommands(this.pivot);
        this.wristCommands = new WristCommands(this.wrist);

        this.wantedLevel = Level.NONE;
        this.isAligned =
                () -> {
                    DriverStation.reportError(
                            "Haven't added isAligned Function yet, check for added message. "
                                    + Timer.getFPGATimestamp(),
                            false);
                    return false;
                };
    }

    public enum Level {
        TROUGH,
        LOW,
        MID,
        HIGH,
        INTAKE,
        NONE
    }

    public void setIsAlignedFunction(BooleanSupplier isAligned) {
        this.isAligned = isAligned;
        DriverStation.reportError("IsAligned function has been set, all is well", false);
    }

    public SuperstructureState getWantedStateByLevel(Level level) {
        switch (level) {
            case TROUGH:
                return new SuperstructureState(
                        PivotConstants.kLevel1Angle, ElevatorConstants.kLevel1Height);
            case LOW:
                return new SuperstructureState(
                        PivotConstants.kLevel2Angle, ElevatorConstants.kLevel2Height);
            case MID:
                return new SuperstructureState(
                        PivotConstants.kLevel3Angle, ElevatorConstants.kLevel3Height);
            case HIGH:
                return new SuperstructureState(
                        PivotConstants.kLevel4Angle, ElevatorConstants.kLevel4Height);
            case INTAKE:
                return new SuperstructureState(
                        PivotConstants.kIntakeAngle, ElevatorConstants.kIntakeHeight);
            default:
                return new SuperstructureState(
                        PivotConstants.kStowAngle, ElevatorConstants.kStowHeight);
        }
    }

    public SuperstructureState getWantedStateByWantedLevel() {
        return getWantedStateByLevel(wantedLevel);
    }

    public void setWantedScoringPosBySideBranch() {
        switch (wantedSide) {
            case A:
                switch (wantedBranch) {
                    case ONE:
                        wantedScoringPos = ScoringPos.A1;
                        break;
                    case TWO:
                        wantedScoringPos = ScoringPos.A2;
                        break;
                    default:
                        DriverStation.reportError(
                                "Cannot set Wanted Scoring pos, no branch provided", false);
                        Logger.recordOutput(
                                "robot/robotErrors",
                                "Cannot set Wanted Scoring pos, no branch provided"
                                        + Timer.getFPGATimestamp());
                        break;
                }
                break;
            case B:
                switch (wantedBranch) {
                    case ONE:
                        wantedScoringPos = ScoringPos.B1;
                        break;
                    case TWO:
                        wantedScoringPos = ScoringPos.B2;
                        break;
                    default:
                        DriverStation.reportError(
                                "Cannot set Wanted Scoring pos, no branch provided", false);
                        Logger.recordOutput(
                                "robot/robotErrors",
                                "Cannot set Wanted Scoring pos, no branch provided"
                                        + Timer.getFPGATimestamp());
                        break;
                }
            case C:
                switch (wantedBranch) {
                    case ONE:
                        wantedScoringPos = ScoringPos.C1;
                        break;
                    case TWO:
                        wantedScoringPos = ScoringPos.C2;
                        break;
                    default:
                        DriverStation.reportError(
                                "Cannot set Wanted Scoring pos, no branch provided", false);
                        Logger.recordOutput(
                                "robot/robotErrors",
                                "Cannot set Wanted Scoring pos, no branch provided"
                                        + Timer.getFPGATimestamp());
                        break;
                }
            case D:
                switch (wantedBranch) {
                    case ONE:
                        wantedScoringPos = ScoringPos.D1;
                        break;
                    case TWO:
                        wantedScoringPos = ScoringPos.D2;
                        break;
                    default:
                        DriverStation.reportError(
                                "Cannot set Wanted Scoring pos, no branch provided", false);
                        Logger.recordOutput(
                                "robot/robotErrors",
                                "Cannot set Wanted Scoring pos, no branch provided"
                                        + Timer.getFPGATimestamp());
                        break;
                }
            case E:
                switch (wantedBranch) {
                    case ONE:
                        wantedScoringPos = ScoringPos.E1;
                        break;
                    case TWO:
                        wantedScoringPos = ScoringPos.E2;
                        break;
                    default:
                        DriverStation.reportError(
                                "Cannot set Wanted Scoring pos, no branch provided", false);
                        Logger.recordOutput(
                                "robot/robotErrors",
                                "Cannot set Wanted Scoring pos, no branch provided"
                                        + Timer.getFPGATimestamp());
                        break;
                }
            case F:
                switch (wantedBranch) {
                    case ONE:
                        wantedScoringPos = ScoringPos.F1;
                        break;
                    case TWO:
                        wantedScoringPos = ScoringPos.F2;
                        break;
                    default:
                        DriverStation.reportError(
                                "Cannot set Wanted Scoring pos, no branch provided", false);
                        Logger.recordOutput(
                                "robot/robotErrors",
                                "Cannot set Wanted Scoring pos, no branch provided"
                                        + Timer.getFPGATimestamp());
                        break;
                }
            default:
                DriverStation.reportError("Cannot set Wanted scoring pos, no SIDE provided", false);
                Logger.recordOutput(
                        "robot/robotErrors", "Cannot set Wanted Scoring pos, no SIDE provided");
                break;
        }
    }

    public ScoringPos getWantedScoringPos() {
        return wantedScoringPos;
    }

    public void logError(String message) {
        Logger.recordOutput("robot/robotErrors", message + Timer.getFPGATimestamp());
    }

    /**
     * NOTE: WILL CHANGE BASED ON INTAKE GEOMETRY
     *
     * @return
     */
    public Command goToStateParalell(SuperstructureState state) {
        if (state.equals(SuperstructureState.kInvalidState)) {
            logError("Invalid state given to gotoState function!!");
            return Commands.none();
        }
        return Commands.parallel(
                elevatorCommands.setHeight(state.elevatorHeight),
                pivotCommands.setAngle(state.pivotAngle));
    }

    /**
     * For Scoring only
     *
     * @param state
     * @return
     */
    public Command goToStatePerpendicular(SuperstructureState state) {
        if (state.equals(SuperstructureState.kInvalidState)) {
            logError("Invalid state given to gotoState function!!");
            return Commands.none();
        }
        return Commands.sequence(
                pivotCommands.setAngle(state.pivotAngle),
                Commands.waitUntil(isAligned),
                elevatorCommands.setHeight(state.elevatorHeight));
    }

    public Command score(SuperstructureState state) {
        return Commands.sequence(goToStatePerpendicular(state), coralerCommands.spitOut());
    }

    public Command scoreAuto() {
        return score(wantedSuperstructureState);
    }

    /**
     * REMEMBER TO TUNE THIS
     *
     * @param state
     * @return
     */
    public Command prepIntake(SuperstructureState state) {
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

    // called when score button pressed on mainController
    public Command setWantedSuperstructureState() {
        return Commands.runOnce(() -> setSSStateAuto());
    }

    private void setSSStateAuto() {
        switch (wantedLevel) {
            case TROUGH:
                wantedSuperstructureState = SuperstructureState.troughScore;
                break;
            case LOW:
                wantedSuperstructureState = SuperstructureState.lowScore;
            case MID:
                wantedSuperstructureState = SuperstructureState.midScore;
            case HIGH:
                wantedSuperstructureState = SuperstructureState.highScore;

            default:
                break;
        }
    }
}
