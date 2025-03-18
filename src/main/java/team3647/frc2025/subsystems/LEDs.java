package team3647.frc2025.subsystems;

import com.ctre.phoenix.led.Animation;
import com.ctre.phoenix.led.CANdle;
import com.ctre.phoenix.led.CANdle.LEDStripType;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import java.util.Map;
import team3647.frc2025.Util.LEDTriggers;
import team3647.frc2025.constants.LEDConstants;
import team3647.lib.team6328.VirtualSubsystem;

public class LEDs extends VirtualSubsystem {

    /** Creates a new LEDSubsystem. */
    private Map<String, Animation> colors =
            Map.of(
                    "blue",
                    LEDConstants.SOLID_BLUE,
                    "red",
                    LEDConstants.SOLID_RED,
                    "aligned",
                    LEDConstants.FLASH_GREEN,
                    "intaking",
                    LEDConstants.FLASH_YELLOW,
                    "hasPiece",
                    LEDConstants.SOLID_YELLOW,
                    "climbing",
                    LEDConstants.SOLID_PURPLE);

    String defaultState = "red";

    String LEDState = defaultState;

    private LEDTriggers triggers;

    private CANdle m_candle;

    public LEDs(CANdle candle, LEDTriggers triggers) {
        super();
        this.m_candle = candle;
        m_candle.configBrightnessScalar(1);
        m_candle.configLEDType(LEDStripType.RGB);

        this.triggers = triggers;

        triggers.alignedTrigger.onTrue(setState("aligned"));
        triggers.alignedTrigger.onFalse(setState(defaultState));
        triggers.intakingTrigger.onTrue(setState("intaking"));
        triggers.intakingTrigger.onFalse(setState(defaultState));
        triggers.pieceTrigger.onTrue(setState("hasPiece"));
        triggers.pieceTrigger.onFalse(setState(defaultState));
    }

    private void setAnimation(Animation animation) {
        m_candle.animate(animation);
    }

    public Command setState(String state) {
        return Commands.runOnce(() -> this.LEDState = state);
    }

    @Override
    public void periodic() {
        DriverStation.getAlliance()
                .ifPresent((alliance) -> defaultState = alliance == Alliance.Blue ? "blue" : "red");
        if (LEDState == "blue" || LEDState == "red") {
            LEDState = defaultState;
        }
        setAnimation(colors.get(LEDState));
    }
}
