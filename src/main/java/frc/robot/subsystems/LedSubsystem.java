package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.LedConstants;

public class LedSubsystem extends SubsystemBase {
  public enum LedMode {
    IDLE, AIMING, SPINNING_UP, READY, FIRING, CLIMBING, ERROR
  }

  private final AddressableLED led = new AddressableLED(LedConstants.kPwmPort);
  private final AddressableLEDBuffer buf = new AddressableLEDBuffer(LedConstants.kLength);

  private LedMode mode = LedMode.IDLE;
  private int tick = 0;

  public LedSubsystem() {
    led.setLength(buf.getLength());
    led.setData(buf);
    led.start();
  }

  public void setMode(LedMode m) { mode = m; }

  @Override
  public void periodic() {
    tick++;

    switch (mode) {
      case IDLE -> solid(0, 0, 10);         // koyu mavi
      case AIMING -> breathe(255, 80, 0);   // turuncu nefes
      case SPINNING_UP -> chase(180, 0, 255); // mor chase
      case READY -> solid(0, 255, 0);       // yeşil
      case FIRING -> strobe(255, 255, 255); // beyaz strobe
      case CLIMBING -> solid(255, 0, 0);    // kırmızı
      case ERROR -> strobe(255, 0, 0);      // kırmızı strobe
    }

    led.setData(buf);
  }

  private void solid(int r, int g, int b) {
    for (int i = 0; i < buf.getLength(); i++) buf.setRGB(i, r, g, b);
  }

  private void strobe(int r, int g, int b) {
    boolean on = (tick / 4) % 2 == 0;
    solid(on ? r : 0, on ? g : 0, on ? b : 0);
  }

  private void breathe(int r, int g, int b) {
    double phase = (Math.sin(tick * 0.12) + 1.0) / 2.0;
    solid((int)(r * phase), (int)(g * phase), (int)(b * phase));
  }

  private void chase(int r, int g, int b) {
    solid(0,0,0);
    int head = tick % buf.getLength();
    for (int k = 0; k < 8; k++) {
      int i = (head - k + buf.getLength()) % buf.getLength();
      double scale = 1.0 - (k / 8.0);
      buf.setRGB(i, (int)(r*scale), (int)(g*scale), (int)(b*scale));
    }
  }
}