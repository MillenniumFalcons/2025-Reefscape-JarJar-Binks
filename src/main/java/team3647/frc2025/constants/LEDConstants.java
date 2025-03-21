package team3647.frc2025.constants;

import com.ctre.phoenix.led.Animation;
import com.ctre.phoenix.led.CANdle;
import com.ctre.phoenix.led.CANdle.LEDStripType;
import com.ctre.phoenix.led.CANdle.VBatOutputMode;
import com.ctre.phoenix.led.ColorFlowAnimation;
import com.ctre.phoenix.led.ColorFlowAnimation.Direction;
import com.ctre.phoenix.led.LarsonAnimation;
import com.ctre.phoenix.led.LarsonAnimation.BounceMode;
import com.ctre.phoenix.led.RainbowAnimation;
import com.ctre.phoenix.led.SingleFadeAnimation;
import com.ctre.phoenix.led.StrobeAnimation;

public class LEDConstants {
    public static final int CANdleID = 17;
    public static final CANdle m_candle =
            new CANdle(LEDConstants.CANdleID, GlobalConstants.kSubsystemCanbusName);

    static {
        m_candle.configVBatOutput(VBatOutputMode.On);
        m_candle.configLEDType(LEDStripType.RGB);
        m_candle.configV5Enabled(true);
    }

    // LED Counts
    public static final int candleLEDS = 8;
    public static final int stripLEDS = 600;
    public static final int LEDCOUNT = candleLEDS + 2 * stripLEDS;
    // Animations List
    // Rainbows
    public static final Animation RAINBOW = new RainbowAnimation(1, 0.1, LEDCOUNT);

    public static final Animation RAINBOWCONTROLLER = new RainbowAnimation(1, 0.5, LEDCOUNT);
    // Unused
    public static final Animation GREEN_STROBE =
            new StrobeAnimation(0, 127, 0, 0, 56.0 / 256.0, LEDCOUNT);

    public static final Animation LARSON =
            new LarsonAnimation(127, 0, 0, 0, 0.75, LEDCOUNT, BounceMode.Front, 50);

    public static final Animation COLOR_FLOW =
            new ColorFlowAnimation(255, 0, 0, 0, 0.7, LEDCOUNT, Direction.Forward);

    // Solid Color

    public static final Animation SOLID_YELLOW = new StrobeAnimation(246/2, 190/2, 0, 128/2, 1, LEDCOUNT);

    public static final Animation SOLID_ORANGE = new StrobeAnimation(255/2, 140/2, 0, 128/2, 1, LEDCOUNT);

    public static final Animation SOLID_PURPLE =
            new StrobeAnimation(162/2, 25/2, 255/2, 128/2, 1, LEDCOUNT);

    public static final Animation SOLID_PINK = new StrobeAnimation(255/2, 0, 255/2, 128/2, 1, LEDCOUNT);

    public static final Animation SOLID_WHITE =
            new StrobeAnimation(255/2, 255/2, 255/2, 128/2, 1, LEDCOUNT);

    public static final Animation SOLID_BLUE = new StrobeAnimation(0, 0, 255/2, 128/2, 1, LEDCOUNT);

    public static final Animation SOLID_RED = new StrobeAnimation(255/2, 0, 0, 128/2, 1, LEDCOUNT);

    public static final Animation SOLID_GREEN = new StrobeAnimation(0, 255/2, 0, 128/2, 1, LEDCOUNT);

    public static final Animation SOLID_BROWN = new StrobeAnimation(181/2, 101/2, 30/2, 128/2, 1, LEDCOUNT);

    // Breathing/Flashing Animations

    public static final Animation BREATHE_RED =
            new SingleFadeAnimation(255/2, 0, 0, 0, 0.8, LEDCOUNT);

    public static final Animation BREATHE_GREEN =
            new SingleFadeAnimation(0, 255/2, 0, 0, 0.65, LEDCOUNT);

    public static final Animation BREATHE_YELLOW =
            new StrobeAnimation(246/2, 190/2, 0, 128/2, 0.3, LEDCOUNT);

    public static final Animation BREATHE_PINK =
            new StrobeAnimation(255/2, 0, 255/2, 128/2, 0.5, LEDCOUNT);

    public static final Animation FLASH_PURPLE = new StrobeAnimation(49/2, 0, 71/2, 128/2, 0.3, LEDCOUNT);

    public static final Animation FLASH_ORANGE =
            new StrobeAnimation(255/2, 50/2, 0, 128/2, 0.5, LEDCOUNT);

    public static final Animation FLASH_YELLOW =
            new StrobeAnimation(246/2, 190/2, 0, 128/2, 0.5, LEDCOUNT);

    public static final Animation FLASH_GREEN = new StrobeAnimation(0, 255/2, 0, 0, 0.5, LEDCOUNT);

    public static final Animation FLASH_BROWN =
            new StrobeAnimation(181/2, 101/2, 30, 128/2, 0.3, LEDCOUNT);
}
