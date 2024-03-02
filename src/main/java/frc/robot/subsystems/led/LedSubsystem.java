package frc.robot.subsystems.led;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LedSubsystem extends SubsystemBase {

  public static enum LEDPattern {
    SOLID,
    FLASH,
    BREATH,
    // B: This used to be called PONG in the old code!
    OFF,
    LOADING,
    // B: Not sure what the next one is supposed to do!
    WAVE,
  }

  public static enum CustomColor {
    RED(255, 0, 0),
    GREEN(0, 255, 0),
    BLUE(0, 0, 255),
    WHITE(255, 255, 255);

    CustomColor(int r, int g, int b) {
      // Color constucture internally takes care of out of range values.
      color = new Color(r, g, b);
    }

    Color getColor() {
      return color;
    }
    private final Color color;
  }

  private AddressableLED m_led = new AddressableLED(LedConstants.PWM_PORT);
  // The color of each LED if it is on (depending on the pattern).
  private AddressableLEDBuffer m_ledColourBuffer = new AddressableLEDBuffer(LedConstants.LENGTH);
  // Same as above but some LEDs migh be off depending on the pattern set.
  private AddressableLEDBuffer m_ledPatternBuffer = new AddressableLEDBuffer(LedConstants.LENGTH);
  private AddressableLEDBuffer m_ledColourBufferStored =
      new AddressableLEDBuffer(LedConstants.LENGTH);
  // The output buffer to be applied to m_led; usually a copy of m_ledPatternBuffer unless
  // m_isFrontLEDsOff in which case some LEDs are turned off.
  private AddressableLEDBuffer m_outputBuffer = new AddressableLEDBuffer(LedConstants.LENGTH);

  private CustomColor m_color;
  private boolean m_isNewColor;
  private LEDPattern m_pattern;
  private LEDPattern m_patternStored;

  // The following three variables represent time in seconds.
  private double m_period;
  private double m_periodStored;
  private double m_periodStart;

  private boolean m_isNewPattern;
  private boolean m_isFrontLEDsOff;

  public LedSubsystem() {
    m_isNewPattern = false;
    m_isFrontLEDsOff = false;
    m_led.setLength(LedConstants.LENGTH);
    setZero(m_ledColourBuffer);
    setColor(CustomColor.WHITE);
    m_color = CustomColor.WHITE;
    // Set this such that the color buffer is filled in the first call of periodic.
    m_isNewColor = true;
    setPattern(LEDPattern.SOLID, 0);
    m_led.start();
  }

  public void setColor(CustomColor color) {
    m_color = color;
    m_isNewColor = true;
  }

  private void copyBuffer(AddressableLEDBuffer from, AddressableLEDBuffer to) {
    for (int i = 0; i < LedConstants.LENGTH; i++) {
      to.setLED(i, from.getLED(i));
    }
  }

  /**
   * Will be called periodically whenever the CommandScheduler runs.
   * The general flow is that the "curren pattern" is applied on the color buffer
   * then the output buffer is set accordingly.
   */
  @Override
  public void periodic() {
    // If there is a new colore, set it first.
    if (m_isNewColor) {
      m_isNewColor = false;
      for (int i = 0; i < LedConstants.LENGTH; i++) {
        m_ledColourBuffer.setLED(i, m_color.getColor());
      }
    }
    // If there is a new pattern since last cycle, copy the colour buffer to the pattern buffer.
    if (m_isNewPattern) {
      m_isNewPattern = false;
      m_periodStart = Timer.getFPGATimestamp();
      if (m_pattern == LEDPattern.LOADING || m_pattern == LEDPattern.OFF) {
        setZero(m_ledPatternBuffer);
      } else {
        copyBuffer(m_ledColourBuffer, m_ledPatternBuffer);
      }
    }

    // Perform the next animation step on the pattern buffer.
    switch (m_pattern) {
      case SOLID:
        break;
      case FLASH:
        if (Timer.getFPGATimestamp() > m_periodStart + (m_period / 2)) {
          setZero(m_ledPatternBuffer);
          if (Timer.getFPGATimestamp() > m_periodStart + m_period) {
            copyBuffer(m_ledColourBuffer, m_ledPatternBuffer);
            m_periodStart = Timer.getFPGATimestamp();
          }
        }
        break;
      case BREATH:
        double ledBrightness = ((Timer.getFPGATimestamp() - m_periodStart) / m_period) * 2;
        if (ledBrightness > 1.0) {
          ledBrightness = 2.0 - ledBrightness;
        }

        if (ledBrightness < 0) {
          ledBrightness = 0;
        }

        for (int i = 0; i < LedConstants.LENGTH; i++) {
          // Note colors returned by getLED are in [0-1] range.
          m_ledPatternBuffer.setLED(i, new Color(
            m_ledColourBuffer.getLED(i).red * ledBrightness,
            m_ledColourBuffer.getLED(i).green * ledBrightness,
            m_ledColourBuffer.getLED(i).blue * ledBrightness));
        }

        if (Timer.getFPGATimestamp() > m_periodStart + m_period) {
          m_periodStart = Timer.getFPGATimestamp();
        }
        break;
      case WAVE:
        if (Timer.getFPGATimestamp() > m_periodStart + m_period) {
          m_periodStart = Timer.getFPGATimestamp();
        }
        break;
      case OFF:
        if (Timer.getFPGATimestamp() > m_periodStart + m_period) {
          m_periodStart = Timer.getFPGATimestamp();
        }
        break;
      case LOADING:
        double columnSize = LedConstants.LENGTH / 4.0;
        double percentDone = Math.min(((Timer.getFPGATimestamp() - m_periodStart) / m_period), 1.0);
        int ledsToTurnOn = (int) Math.round(columnSize * percentDone);
        int twoColumns = (int) Math.round(columnSize * 2);

        for (int i = 0; i < ledsToTurnOn; i++) {
          m_ledPatternBuffer.setLED(i, m_ledColourBuffer.getLED(i));
          m_ledPatternBuffer.setLED(twoColumns - i - 1, m_ledColourBuffer.getLED(twoColumns - i - 1));
          m_ledPatternBuffer.setLED(i + twoColumns, m_ledColourBuffer.getLED(i + twoColumns));
          m_ledPatternBuffer.setLED(
              2 * twoColumns - i - 1, m_ledColourBuffer.getLED(2 * twoColumns - i - 1));
        }

        if (Timer.getFPGATimestamp() > m_periodStart + m_period) {
          m_periodStart = Timer.getFPGATimestamp();
          setZero(m_ledPatternBuffer);
        }
        break;
    }

    copyBuffer(m_ledPatternBuffer, m_outputBuffer);

    if (m_isFrontLEDsOff) {
      for (int i = 0; i < (LedConstants.LENGTH / 4) + 1; i++) {
        m_outputBuffer.setRGB(i, 0, 0, 0);
      }
      for (int i = (LedConstants.LENGTH / 4) * 3; i < LedConstants.LENGTH; i++) {
        m_outputBuffer.setRGB(i, 0, 0, 0);
      }
    } else {
      for (int i = 0; i < (LedConstants.LENGTH / 4) + 1; i++) {
        m_outputBuffer.setLED(i, m_ledPatternBuffer.getLED(i));
      }
      for (int i = (LedConstants.LENGTH / 4) * 3; i < LedConstants.LENGTH; i++) {
        m_outputBuffer.setLED(i, m_ledPatternBuffer.getLED(i));
      }
    }

    // Write the pattern buffer to the LED.
    m_led.setData(m_outputBuffer);
  }

  /**
   * Sets the entire given colour buffer to zero.
   */
  private void setZero(AddressableLEDBuffer buffer) {
    for (int i = 0; i < LedConstants.LENGTH; i++) {
      buffer.setRGB(i, 0, 0, 0);
    }
  }

  /**
   * Sets which pattern to use to display the colour buffer.
   *
   * @param pattern Which LEDPattern to run on the LED's.
   * @param period The period over which to run the pattern (seconds).
   */
  public void setPattern(LEDPattern pattern, double period) {
    m_pattern = pattern;
    m_period = period;
    m_isNewPattern = true;
  }

  // The next methods are kept from the old code (but not used in this project).

  /** Stores the pattern & colours to be recalled at a later time. */
  public void storePattern() {
    copyBuffer(m_ledColourBuffer, m_ledColourBufferStored);
    m_patternStored = m_pattern;
    m_periodStored = m_period;
  }

  public void restorePattern() {
    copyBuffer(m_ledColourBufferStored, m_ledColourBuffer);
    m_pattern = m_patternStored;
    m_isNewPattern = true;
    m_period = m_periodStored;
  }
}
