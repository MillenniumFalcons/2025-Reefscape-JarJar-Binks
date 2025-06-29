package team3647.frc2025.subsystems.superstructure;

import static edu.wpi.first.units.Units.Degree;
import static edu.wpi.first.units.Units.Inch;
import static edu.wpi.first.units.Units.Meter;
import static edu.wpi.first.units.Units.Radian;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.MutAngle;
import edu.wpi.first.units.measure.MutDistance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import java.util.Map;
import java.util.Set;
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
import team3647.frc2025.subsystems.Elevator.Elevator;
import team3647.frc2025.subsystems.Coraler;
import team3647.frc2025.subsystems.Rollers;
import team3647.frc2025.subsystems.Seagull;
import team3647.frc2025.subsystems.pivot.Pivot;
import team3647.frc2025.subsystems.wrist.Wrist;
import team3647.frc2025.Util.AllianceFlip;
import team3647.lib.PeriodicSubsystem;

public abstract class Superstructure implements PeriodicSubsystem {

    public final Coraler coraler;
    public final Elevator elevator;
    public final Pivot pivot;
    public final Wrist wrist;
    public final Seagull seagull;
    public final Rollers rollers;

    public final CoralerCommands coralerCommands;
    public final ElevatorCommands elevatorCommands;
    public final PivotCommands pivotCommands;
    public final WristCommands wristCommands;
    public final RollersCommands rollersCommands;

    protected BooleanSupplier isAligned;

    protected Supplier<Pose2d> robotPose;

    protected MutDistance elevOffset;

    protected MutAngle pivotOffset;

    protected Level wantedLevel;

    protected boolean hasPeice = true;

    public boolean preloaded;

    protected boolean hasAlgae = false;

    protected ScoringPos wantedScoringPos = ScoringPos.NONE;

    protected Trigger overridePiece;

    protected double currentLimit = 57;
    protected double SeagullCurrentLimit = 30;
    protected double algaeCurrentLimit = 47;

    protected double wristOffset = 0;

    protected Map<Level, SuperstructureState> kLevelToScoreMap;

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
            Seagull seagull,
            Trigger pieceOverride,
            Supplier<Pose2d> robotPose) {
        this.coraler = coraler;
        this.elevator = elevator;
        this.pivot = pivot;
        this.wrist = wrist;
        this.seagull = seagull;
        this.rollers = rollers;
        this.robotPose = robotPose;

        this.coralerCommands = new CoralerCommands(this.coraler);
        this.elevatorCommands = new ElevatorCommands(this.elevator);
        this.pivotCommands = new PivotCommands(this.pivot);
        this.rollersCommands = new RollersCommands(rollers, seagull);
        this.wristCommands = new WristCommands(wrist);

        this.elevOffset = Meter.of(0).mutableCopy();
        this.pivotOffset = Radian.of(0).mutableCopy();

        this.overridePiece = pieceOverride;

        this.wantedLevel = Level.HIGH;
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
                        SuperstructureState.TroughScore,
                        Level.LOW,
                        SuperstructureState.LowScore,
                        Level.MID,
                        SuperstructureState.MidScore,
                        Level.HIGH,
                        SuperstructureState.HighScore);
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

    public Command goToState(Supplier<SuperstructureState> finalState){
        return new GoToState(finalState);
    }

    public Command goToState(SuperstructureState finalState){
        return new GoToState(() -> finalState).until(()->getCurrentState().equalsWithTolerance(finalState));
    }

    public Command goToState(SuperstructureState finalState, boolean overridden){
        return new GoToState(() -> finalState, overridden).until(()->getCurrentState().equalsWithTolerance(finalState));
    }

    public void setIsAlignedFunction(BooleanSupplier isAligned) {
        this.isAligned = isAligned;
        DriverStation.reportError("IsAligned function has been set, all is well", false);
    }

    public boolean isAligned() {
        return isAligned.getAsBoolean();
    }

    public ScoringPos getWantedScoringPos() {
        return wantedScoringPos;
    }

    public void logError(String message) {
        Logger.recordOutput("Robot/robotErrors", message + Timer.getFPGATimestamp());
        DriverStation.reportError(message, true);
    }

    public SuperstructureState getStateScoreAuto() {
        var prelimWantedState =
                kLevelToScoreMap.getOrDefault(getWantedLevel(), SuperstructureState.kInvalidState);
        Logger.recordOutput("Superstructure/wanted state", prelimWantedState.name);
        return prelimWantedState;
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

    public SuperstructureState getCurrentState() {
        return new SuperstructureState(pivot.getAngle(), elevator.getHeight(), wrist.getAngle());
    }

    // algae stuff

    // unimplemented lmao
    public Command autoTakeOffByLevel() {
        return Commands.either(
                takeOffAlgaeLow(), takeOffAlgaeHigh(), () -> getWantedLevel() == Level.ALGAELOW);
    }

    public Command takeOffAlgaeHigh() {
        return Commands.parallel(
                goToState(() -> SuperstructureState.HighAlgae), coralerCommands.intake());
    }

    public Command takeOffAlgaeLow() {
        return Commands.parallel(
                goToState(() -> SuperstructureState.LowAlgae), coralerCommands.intake());
    }

    public Command scoreAlgaeBarge() {
        return Commands.parallel(
                elevatorCommands.setHeight(ElevatorConstants.kMaxHeight),
                Commands.sequence(
                        Commands.waitSeconds(0.3),
                        pivotCommands.setAngle(PivotConstants.kStowAngle)),
                Commands.sequence(
                        Commands.waitSeconds(0.39),
                        coralerCommands.setOpenLoop(-1.0).withTimeout(0.5)));
    }

    public Command stowAlgaeBarge() {
        return Commands.parallel(
                Stow(),
                Commands.sequence(
                        Commands.waitSeconds(0.025),
                        coralerCommands.setOpenLoop(-1.0).withTimeout(0.5)));
    }

    public Command setHasAlgae() {
        return Commands.runOnce(() -> this.hasAlgae = true);
    }

    public Command setNoAlgae() {
        return Commands.runOnce(() -> this.hasAlgae = false);
    }

    public boolean getAlgae() {
        return hasAlgae;
    }

    public Command intake() {
        return intake(false);
    }

    public Command intake(boolean overridden){
        return Commands.parallel(
                                goToState(SuperstructureState.Intake, overridden),
                                rollersCommands.setOpenLoop(0.25))
                        .withTimeout(0.3).andThen( // add TOF sensor here .until(),
                transfer());
    }

    public Command transfer() {
        return Commands.parallel(
                goToState(SuperstructureState.Transfer),
                rollersCommands.setOpenLoop(0.6, -0.1))//.until(TOF)
                .andThen(Stow());
    }

    public Command handoff() {
        return Commands.parallel(
                rollersCommands.kill(),
                goToState(() -> SuperstructureState.Handoff),
                coralerCommands.intake());
    }

    public Command stowc() {
        return Commands.sequence(
                elevatorCommands.setHeight(ElevatorConstants.kClearHeight),
                pivotCommands.setAngle(PivotConstants.kStowAngle),
                elevatorCommands.setHeight(ElevatorConstants.kStowHeight));
    }

    public Command scoreL4() {
        return Commands.sequence(
                elevatorCommands.setHeight(ElevatorConstants.kLevel4Height),
                pivotCommands.setAngle(PivotConstants.kL4Prep));
    }

    public Command scoreL3() {
        return Commands.sequence(
                elevatorCommands.setHeight(ElevatorConstants.kThreeHeight),
                pivotCommands.setAngle(PivotConstants.kL3prep));
    }

    public Command scoreL2() {
        return Commands.sequence(
                elevatorCommands.setHeight(ElevatorConstants.kLevel2Height),
                pivotCommands.setAngle(PivotConstants.kLevel2Angle));
    }

    public Command poopCoral() {
        return Commands.waitSeconds(0.15)
                .andThen(coralerCommands.setOpenLoop(-0.3).withTimeout(0.1))
                .withTimeout(0.3);
    }

    public MutDistance getElevOffset() {
        return elevOffset;
    }

    public MutAngle getPivotOffset() {
        return pivotOffset;
    }

    public boolean intakeCurrent() {
        return rollers.getMasterCurrent() > currentLimit;
    }

    public boolean coralerAlgaeCurrent() {
        return coraler.getMasterCurrent() > algaeCurrentLimit;
    }

    public Command stowIntake() {
        return Commands.parallel(
                wristCommands.setAngle(WristConstants.kStowWithPiece),
                rollersCommands.kill(),
                coralerCommands.kill());
    }

    public Command autoScoreByLevel() {
        return goToState(this::getStateScoreAuto);
        // return goToStatePerpendicular(this::getStateScoreAuto, () ->
        // getDelayByLevel(wantedLevel));
    }

    public double getDelayByLevel(Level level) {
        switch (level) {
            case HIGH:
                return 0;

            default:
                return 0.3;
        }
    }

    public Command Stow() {
        return goToState(() -> SuperstructureState.Stow);
    }


    public Command killAll() {
        return Commands.parallel(rollersCommands.kill(), coralerCommands.kill());
    }

    public Command setWantedLevel(Level wantedLevel) {
        return Commands.runOnce(
                        () -> {
                            this.wantedLevel = wantedLevel;
                        })
                .ignoringDisable(true);
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

    public boolean seagullCurrent() {
        return rollersCommands.seagullCurrentGreater(SeagullCurrentLimit);
    }

    public Command stopIntaking() {
        return Commands.none();
    }
    
    class GoToState extends Command {
        Supplier<SuperstructureState> finalState;
        Supplier<SuperstructureState> getState;
        SuperstructureState freezeState;
        BooleanSupplier freeze;
        boolean frozen;

        GoToState(Supplier<SuperstructureState> finalState, boolean overridden){
            this.finalState = finalState;
            freeze = overridden ? () -> false : () -> 
            {
                return team3647.frc2025.Util.PoseUtils.inCircle(
                robotPose.get(),
                new Pose2d(team3647.frc2025.constants.FieldConstants.kBlueReefOrigin, Rotation2d.kZero),
                Inch.of(93.5/2.0 + 13.0)) || 
                team3647.frc2025.Util.PoseUtils.inCircle(robotPose.get(),
                new Pose2d(AllianceFlip.flip(team3647.frc2025.constants.FieldConstants.kBlueReefOrigin), Rotation2d.kZero),
                Inch.of(93.5/2.0 + 13.0));
            };

            getState = overridden ? finalState::get : () -> InverseKinematics.interpolate(()-> getCurrentState(), finalState, robotPose);
            
        }

        GoToState(Supplier<SuperstructureState> finalState){
            this(finalState, false);
        }

        @Override 
        public void initialize(){
        // CommandScheduler.getInstance().schedule(
        //     goSubsystems(getState)
        // );
        }
        @Override
        public void execute(){
            if(!frozen && freeze.getAsBoolean()){
                freezeState = getCurrentState();
                frozen = true;
            } else if(frozen && !freeze.getAsBoolean()){
                frozen = false;
            }
            
            Logger.recordOutput("Superstructure/Frozen", frozen);

            if (frozen){
                goSubsystems(freezeState);
                return;
            } 
            goSubsystems(getState.get());
        }
        
        @Override
            public Set<Subsystem> getRequirements() {
                return Set.of(elevator, pivot, wrist);
            }

        public void goSubsystems(SuperstructureState state){
            pivot.setAngle(state.pivotAngle);
            elevator.setHeight(state.elevatorHeight);
            wrist.setAngle(state.wristAngle);
        }
    }
}
