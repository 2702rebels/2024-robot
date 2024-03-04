package frc.robot.subsystems.led;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.Misc.ColorHSV;
import java.util.function.BiConsumer;

public class LedSubsystem extends SubsystemBase {
  private final AddressableLED strip;
  private final AddressableLEDBuffer buffer;

  private Color color;
  private BiConsumer<AddressableLEDBuffer, Double> pattern;

  private double period = 0;
  private double periodStart;

  /** Pattern that does nothing */
  private static void nop(AddressableLEDBuffer buffer, double t) {}

  public LedSubsystem(int length, int port) {
    // initialize the physical LED strip length
    strip = new AddressableLED(port);
    strip.setLength(length);

    // black color means LEDs are in the off state
    color = Color.kBlack;
    buffer = new AddressableLEDBuffer(length);

    pattern = LedSubsystem::nop;
    periodStart = Timer.getFPGATimestamp();

    // start the physical LED strip operation
    strip.start();
  }

  /**
   * Sets the solid color to use with standard patterns.
   *
   * <p>You can set this without changing the pattern to switch to a different color.
   */
  public void setColor(Color color) {
    this.color = color;
  }

  /**
   * Sets the period in seconds.
   *
   * <p>You can set this without changing the pattern to switch to a different period.
   */
  public void setPeriod(double period) {
    this.period = period;
    this.periodStart = Timer.getFPGATimestamp();
  }

  /**
   * Sets the pattern to the specified functional implementation and period.
   *
   * <p>Setting the pattern this way allows for arbitrary implementations and will disregard current
   * color settings.
   */
  public void setPattern(BiConsumer<AddressableLEDBuffer, Double> pattern, double period) {
    this.pattern = pattern;
    setPeriod(period);
  }

  @Override
  public void periodic() {
    final var timestamp = Timer.getFPGATimestamp();
    final var t =
        MathUtil.clamp(period > 0 ? ((timestamp - periodStart) % period) / period : 0, 0, 1);

    // populate the buffer using pattern implementation
    // and write the buffer to the physical LED strip
    pattern.accept(buffer, Double.valueOf(t));
    strip.setData(buffer);
  }

  /** Sets the solid color pattern. */
  public void solid(Color color, double period) {
    // initialize buffer here (once) and use 'nop' pattern,
    // since we don't need to update the buffer periodically
    // when solid color is requested
    LedPatterns.setColor(buffer, color);
    setColor(color);
    setPattern(LedSubsystem::nop, period);
  }

  /** Sets the flashing color pattern. */
  public void flash(Color color, double period) {
    setColor(color);
    setPattern((buffer, t) -> LedPatterns.flash(buffer, t, this.color), period);
  }

  /** Sets the countdown color pattern. */
  public void countdown(Color color, double period, int chunks) {
    setColor(color);
    setPattern((buffer, t) -> LedPatterns.countdown(buffer, t, this.color, chunks), period);
  }

  /** Sets the countup color pattern. */
  public void countup(Color color, double period, int chunks) {
    setColor(color);
    setPattern((buffer, t) -> LedPatterns.countup(buffer, t, this.color, chunks), period);
  }

  /** Sets the breath color pattern. */
  public void breath(Color color, double period) {
    setColor(color);
    setPattern((buffer, t) -> LedPatterns.breath(buffer, t, this.color, 0.25, 1), period);
  }

  /** Sets the breath color pattern using HSV color scheme. */
  public void breath(ColorHSV color, double period) {
    setPattern((buffer, t) -> LedPatterns.breath(buffer, t, color, 0.25, 1), period);
  }
}
