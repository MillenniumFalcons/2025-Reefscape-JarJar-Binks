package team3647.frc2025.subsystems;

import java.util.Map;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.MathUtil;
import static edu.wpi.first.units.Units.Degree;
import static edu.wpi.first.units.Units.Inch;
import static edu.wpi.first.units.Units.Meter;
import static edu.wpi.first.units.Units.Radian;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.MutAngle;
import edu.wpi.first.units.measure.MutDistance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;
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

public class Superstructure {

    private final Coraler coraler;
    private final Elevator elevator;
    private final Pivot pivot;
    private final Wrist wrist;

    public final CoralerCommands coralerCommands;
    public final ElevatorCommands elevatorCommands;
    public final PivotCommands pivotCommands;
    public final WristCommands wristCommands;
    public final RollersCommands rollersCommands;

    private BooleanSupplier isAligned;

    private MutDistance elevOffset;

    private MutAngle pivotOffset;

    private Level wantedLevel;

    private boolean hasPeice = true;

    private ScoringPos wantedScoringPos = ScoringPos.NONE;

    private Trigger overridePiece;

    private double currentLimit = 35;

    private double wristOffset = 0;

    private Map<Level, SuperstructureState> kLevelToScoreMap;

    public enum Side {
        A,
        B,
        C,
        D,
        E,
        F
    }

    public enum Branch {
        ONE(-1),
        TWO(1);

        public final int sign;

        Branch(int sign) {
            this.sign = sign;
        }
    }

    public Superstructure(
            Coraler coraler,
            Elevator elevator,
            Pivot pivot,
            Wrist wrist,
            Rollers rollers,
            Trigger pieceOverride) {
        this.coraler = coraler;
        this.elevator = elevator;
        this.pivot = pivot;
        this.wrist = wrist;

        this.coralerCommands = new CoralerCommands(this.coraler);
        this.elevatorCommands = new ElevatorCommands(this.elevator);
        this.pivotCommands = new PivotCommands(this.pivot);
        this.rollersCommands = new RollersCommands(rollers);
        this.wristCommands = new WristCommands(wrist);

        this.elevOffset = Meter.of(0).mutableCopy();
        this.pivotOffset = Radian.of(0).mutableCopy();

        this.overridePiece = pieceOverride;

        this.wantedLevel = Level.NONE;
        this.isAligned =
                () -> {
                    DriverStation.reportError(
                            "Haven't added isAligned Function yet, check for added message. "
                                    + Timer.getFPGATimestamp(),
                            false);
                    return false;
                };

        this.kLevelToScoreMap =
                Map.of(
                        Level.TROUGH,
                        SuperstructureState.troughScore,
                        Level.LOW,
                        SuperstructureState.lowScore,
                        Level.MID,
                        SuperstructureState.midScore,
                        Level.HIGH,
                        SuperstructureState.highScore);
    }

    public enum Level {
        TROUGH,
        LOW,
        MID,
        HIGH,
        ALGAEHIGH,
        ALGAELOW,
        NONE
    }

    public void setIsAlignedFunction(BooleanSupplier isAligned) {
        this.isAligned = isAligned;
        DriverStation.reportError("IsAligned function has been set, all is well", false);
    }

    public boolean shouldClear() {
        return pivot.getAngle().gt(Radian.of(-0.7))
                && elevator.getHeight().lt(ElevatorConstants.kClearHeight);
    }

    public boolean isAligned() {
        return isAligned.getAsBoolean();
    }

    public ScoringPos getWantedScoringPos() {
        return wantedScoringPos;
    }

    public void logError(String message) {
        Logger.recordOutput("robot/robotErrors", message + Timer.getFPGATimestamp());
        DriverStation.reportError(message, true);
    }

    public Command goToStateParalell(Supplier<SuperstructureState> state) {

        return Commands.either(
                Commands.parallel(
                        // Commands.run(() -> Logger.recordOutput("monkeyballs",
                        // state.get().elevatorHeight)),
                        elevatorCommands.setHeight(() -> state.get().elevatorHeight),
                        pivotCommands.setAngle(
                                () ->
                                        MathUtil.clamp(
                                                state.get().pivotAngle.in(Radian),
                                                getPivotMinAngle(),
                                                pivot.getMaxAngle().in(Radian))),
                        wristCommands.setAngle(
                                () -> {
                                    return state.get().wristAngle.equals(WristConstants.idrc)
                                            ? getCurrentState().wristAngle
                                            : state.get().wristAngle;
                                })
                        // Commands.run(() ->
                        // Logger.recordOutput("ballsmonkey",state.get().pivotAngle))

                        ),
                Commands.none(),
                () -> !state.get().equals(SuperstructureState.kInvalidState));
    }

    /**
     * delayed elevator
     *
     * @param state
     * @return gotostate with a delayed elevator
     */
    public Command goToStatePerpendicular(Supplier<SuperstructureState> state) {

        return Commands.either(
                Commands.parallel(
                        // Commands.run(() -> Logger.recordOutput("monkeyballs",
                        // state.get().elevatorHeight)),
                        Commands.waitSeconds(0.1)
                                .andThen(
                                        elevatorCommands.setHeight(
                                                () -> state.get().elevatorHeight)),
                        pivotCommands.setAngle(
                                () ->
                                        MathUtil.clamp(
                                                state.get().pivotAngle.in(Radian),
                                                getPivotMinAngle(),
                                                pivot.getMaxAngle().in(Radian))),
                        wristCommands.setAngle(
                                () -> {
                                    return state.get().wristAngle.equals(WristConstants.idrc)
                                            ? getCurrentState().wristAngle
                                            : state.get().wristAngle;
                                })
                        // Commands.run(() ->
                        // Logger.recordOutput("ballsmonkey",state.get().pivotAngle))

                        ),
                Commands.none(),
                () -> !state.get().equals(SuperstructureState.kInvalidState));
    }

    /**
     * delayed elevator
     *
     * @param state
     * @return gotostate with a delayed elevator
     */
    public Command goToStatePerpendicular(
            Supplier<SuperstructureState> state, DoubleSupplier delay) {

        return Commands.either(
                Commands.parallel(
                        // Commands.run(() -> Logger.recordOutput("monkeyballs",
                        // state.get().elevatorHeight)),
                        Commands.waitSeconds(delay.getAsDouble())
                                .andThen(
                                        elevatorCommands.setHeight(
                                                () -> state.get().elevatorHeight)),
                        pivotCommands.setAngle(
                                () ->
                                        MathUtil.clamp(
                                                state.get().pivotAngle.in(Radian),
                                                getPivotMinAngle(),
                                                pivot.getMaxAngle().in(Radian))),
                        wristCommands.setAngle(
                                () -> {
                                    return state.get().wristAngle.equals(WristConstants.idrc)
                                            ? getCurrentState().wristAngle
                                            : state.get().wristAngle;
                                })
                        // Commands.run(() ->
                        // Logger.recordOutput("ballsmonkey",state.get().pivotAngle))

                        ),
                Commands.none(),
                () -> !state.get().equals(SuperstructureState.kInvalidState));
    }

    public boolean hasPeice() {
        // TODO: implememtation neded
        return hasPeice;
        // for now no logic cause no handoff code
    }

    public Command setPeice() {
        return Commands.runOnce(() -> this.hasPeice = true);
    }

    public Command setNoPeice() {
        return Commands.runOnce(() -> this.hasPeice = false);
    }

    public double getPivotMinAngle() {
        return InverseKinematics.getMinAngle(getCurrentState()).in(Radian);
    }

    public SuperstructureState getCurrentState() {
        return new SuperstructureState(pivot.getAngle(), elevator.getHeight(), wrist.getAngle());
    }

    public Command takeOffAlgaeLow() {
        return Commands.sequence(
                elevatorCommands.setHeight(ElevatorConstants.kLowAlgaeHeight),
                pivotCommands.setAngle(PivotConstants.kAlgaeAngleLow));
    }

    public boolean shouldClearGoingUp() {
        return pivot.angleWithin(
                        PivotConstants.kMinAngle.minus(Degree.of((5))).in(Radian),
                        PivotConstants.kClearAngle.in(Radian))
                && elevator.getHeight().lt(ElevatorConstants.kClearHeight);
    }

    //

    public Command stowc() {
        return Commands.sequence(
                elevatorCommands.setHeight(ElevatorConstants.kClearHeight),
                pivotCommands.setAngle(PivotConstants.kStartingAngle),
                elevatorCommands.setHeight(ElevatorConstants.kStowHeight));
    }

    public Command clearElevatorGoingUpNoDown() {
        return Commands.either(
                Commands.sequence(
                        Commands.deadline(
                                elevatorCommands.setHeight(ElevatorConstants.kClearHeight),
                                pivotCommands.setAngle(PivotConstants.kStartingAngle)),
                        pivotCommands.setAngle(PivotConstants.kClearAngle)),
                Commands.none(),
                () ->
                        pivot.angleWithin(
                                        PivotConstants.kStartingAngle.in(Radian),
                                        PivotConstants.kClearAngle.in(Radian))
                                && elevator.getHeight().lt(ElevatorConstants.kClearHeight));
    }

    public Command clearElevatorGoingUpNoDown(Angle finalPivotAngle) {
        return Commands.either(
                Commands.sequence(
                        Commands.deadline(
                                elevatorCommands.setHeight(ElevatorConstants.kClearHeight),
                                pivotCommands.setAngle(PivotConstants.kStowAngle),
                                wristCommands.stow()),
                        pivotCommands.setAngle(
                                MathUtil.clamp(
                                        finalPivotAngle.in(Radian),
                                        PivotConstants.kClearAngle.in(Radian),
                                        PivotConstants.kMaxAngle.in(Radian)))),
                Commands.none(),
                () ->
                        pivot.angleWithin(
                                        PivotConstants.kStartingAngle.in(Radian),
                                        PivotConstants.kClearAngle.in(Radian))
                                && elevator.getHeight().lt(ElevatorConstants.kClearHeight));
    }

    public Command clearElevatorGoingUpNoDownNew(Angle finalPivotAngle) {
        return Commands.either(
                Commands.sequence(
                        Commands.deadline(
                                elevatorCommands.setHeight(ElevatorConstants.kClearHeight),
                                wristCommands.stow()),
                        pivotCommands.setAngle(
                                MathUtil.clamp(
                                        finalPivotAngle.in(Radian),
                                        PivotConstants.kClearAngle.in(Radian),
                                        PivotConstants.kMaxAngle.in(Radian)))),
                Commands.none(),
                () ->
                        pivot.angleWithin(
                                        PivotConstants.kStartingAngle.in(Radian),
                                        PivotConstants.kClearAngle.in(Radian))
                                && elevator.getHeight().lt(ElevatorConstants.kClearHeight));
    }

    public Command scoreL4() {
        return Commands.sequence(
                clearElevatorGoingUpNoDown(PivotConstants.kStowAngleUp),
                elevatorCommands.setHeight(ElevatorConstants.kLevel4Height),
                pivotCommands.setAngle(PivotConstants.kL4Prep));
    }

    public Command scoreL3() {
        return Commands.sequence(
                clearElevatorGoingUpNoDown(PivotConstants.kStowAngleUp),
                elevatorCommands.setHeight(ElevatorConstants.kLevel3Height),
                pivotCommands.setAngle(PivotConstants.kL3prep));
    }

    public Command scoreL2() {
        return Commands.sequence(
                clearElevatorGoingUpNoDown(PivotConstants.kL2Prep),
                elevatorCommands.setHeight(ElevatorConstants.kLevel2Height),
                pivotCommands.setAngle(PivotConstants.kLevel2Angle));
    }

    public Command poopCoral() {
        return Commands.waitSeconds(0.15)
                .andThen(coralerCommands.setOpenLoop(-0.3).withTimeout(0.1))
                .withTimeout(0.3);
    }

    public Command stowFromL4() {
        // put it on the reef
        return Commands.sequence(pivotCommands.setAngle(PivotConstants.kLevel4Angle), poopCoral());
    }

    public Command stowFromL3() {
        return Commands.parallel(pivotCommands.setAngle(PivotConstants.kLevel3Angle), poopCoral());
    }

    public MutDistance getElevOffset() {
        return elevOffset;
    }

    public MutAngle getPivotOffset() {
        return pivotOffset;
    }

    public Trigger intakeCurrent() {
        return new Trigger(() -> rollersCommands.currentGreater(currentLimit)).debounce(0.0);
    }

    public Command stowFromIntake() {
        return Commands.sequence(
                wristCommands
                        .setAngle(WristConstants.kHandoffAngle)
                        .alongWith(
                                pivotCommands.setAngle(PivotConstants.kHandoffAngle),
                                rollersCommands.setOpenLoop(-0.17).withTimeout(0.1)),
                Commands.waitSeconds(0.3),
                elevatorCommands
                        .setHeight(ElevatorConstants.kStartingHeight)
                        .alongWith(
                                coralerCommands
                                        .setOpenLoop(0.5)
                                        .until(coralerCommands.current().or(overridePiece))),
                wristCommands.setAngle(Degree.of(50.14)),
                elevatorCommands.setHeight(ElevatorConstants.kClearHeight),
                pivotCommands.setAngle(PivotConstants.kStowAngleUp),
                elevatorCommands.setHeight(ElevatorConstants.kStowHeight),
                wristCommands.setAngle(WristConstants.kStowAngle));
    }

    public Angle getPivotAngleByLevel() {
        switch (wantedLevel) {
            case TROUGH:
                return PivotConstants.KL1Prep;
            case LOW:
                return PivotConstants.kL2Prep;
            case MID:
                return PivotConstants.kStowAngleUp;
            case HIGH:
                return PivotConstants.kStowAngleUp;
            default:
                return PivotConstants.kStowAngleUp;
        }
    }

    public Command stowIntake() {
        return Commands.parallel(
                Commands.either(
                        pivotCommands.setAngle(PivotConstants.kStowAngle),
                        pivotCommands.setAngle(PivotConstants.kStowAngleUp),
                        pivot::needToClearElevator),
                wristCommands.setAngle(WristConstants.kStowWithPiece),
                elevatorCommands.setHeight(ElevatorConstants.kStowHeight),
                rollersCommands.kill(),
                coralerCommands.kill());
    }

    public Command handoff() {
        return Commands.sequence(
                wristCommands.setAngle(WristConstants.kHandoffAngle).withTimeout(1),
                pivotCommands.setAngle(PivotConstants.kHandoffAngle),
                coralerCommands.setOpenLoop(0.5).withTimeout(2),
                pivotCommands.setAngle(PivotConstants.kStowAngle),
                wristCommands.setAngle(WristConstants.kStowAngle));
    }

    // public Command intake(){
    // return Commands.parallel(
    // wristcommands.setAngle(kHandoffAngle),
    // pivotCommands.setAngle(PivotConstants.kIntakeAngle),
    // rollersCommands.intake()
    // );
    // }

    public Command autoScoreByLevel() {

        return goToStateParalell(
                () ->
                        kLevelToScoreMap.getOrDefault(
                                getWantedLevel(), SuperstructureState.kInvalidState));
    }

    public Command stow() {
        return Commands.either(
                Commands.sequence(
                        goToStatePerpendicular(() -> SuperstructureState.toStow),
                        goToStateParalell(() -> SuperstructureState.stow)),
                goToStatePerpendicular(() -> SuperstructureState.stow, () -> 0.3),
                this::shouldClear);
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

    public Command pivotOffsetUp() {
        return Commands.runOnce(() -> pivotOffset.mut_plus(Degree.of(1)));
    }

    public Command pivotOffsetDown() {
        return Commands.runOnce(() -> pivotOffset.mut_minus(Degree.of(1)));
    }

    public Command elevOffsetUp() {
        return Commands.runOnce(() -> elevOffset.mut_plus(Inch.of(1)));
    }

    public Command elevOffsetDown() {
        return Commands.runOnce(() -> elevOffset.mut_minus(Inch.of(1)));
    }

    public boolean isIntaking() {
        return wrist.angleReached(WristConstants.kIntakeAngle.in(Degree), 5);
    }
}
