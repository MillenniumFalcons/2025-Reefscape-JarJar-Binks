package team3647.frc2025.subsystems;

import com.ctre.phoenix.led.Animation;
import com.ctre.phoenix.led.CANdle;
import com.ctre.phoenix.led.CANdle.LEDStripType;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import java.util.Map;
import team3647.frc2025.constants.LEDConstants;
import team3647.lib.team6328.VirtualSubsystem;

public class LEDs extends VirtualSubsystem {

    /** Creates a new LEDSubsystem. */
    private Map<String, Animation> colors =
            Map.of("aligned", LEDConstants.BREATHE_GREEN, "intaking", LEDConstants.SOLID_YELLOW);

    String defaultState = "red";

    String LEDState = defaultState;

    private CANdle m_candle;

    public LEDs(CANdle candle) {
        this.m_candle = candle;
        m_candle.configBrightnessScalar(1);
        m_candle.configLEDType(LEDStripType.GRB);
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
