package frc.robot.subsystems;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.motorcontrol.VictorSP;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstants;

public class HoodSubsystem extends SubsystemBase {

  private final VictorSP hoodMotor = new VictorSP(ShooterConstants.kHoodMotorPort);
  private final AnalogInput pot = new AnalogInput(ShooterConstants.kPotentiometerPort);

  // P/I/D saha üstünde tune edilir
  private final PIDController pid =
      new PIDController(ShooterConstants.kHoodP, ShooterConstants.kHoodI, ShooterConstants.kHoodD);

  private final InterpolatingDoubleTreeMap hoodTable = new InterpolatingDoubleTreeMap();

  // en son hedefi sakla (log/debug)
  private double lastTargetDeg = 0.0;

  public HoodSubsystem() {
    hoodTable.put(1.5, 10.0);
    hoodTable.put(3.0, 25.0);
    hoodTable.put(5.0, 45.0);

    pid.setTolerance(ShooterConstants.kHoodToleranceDeg);

  }

  /** Pot voltajını gerçek min/max ile dereceye çevir (hassaslık burada başlıyor). */
  public double getAngleDeg() {
    double v = pot.getVoltage();

    // Voltajı pot’un gerçek aralığında clamp et
    v = MathUtil.clamp(v, ShooterConstants.kPotMinV, ShooterConstants.kPotMaxV);

    // lineer map: [minV..maxV] -> [minDeg..maxDeg]
    return MathUtil.interpolate(
        ShooterConstants.kHoodMinDeg,
        ShooterConstants.kHoodMaxDeg,
        (v - ShooterConstants.kPotMinV) / (ShooterConstants.kPotMaxV - ShooterConstants.kPotMinV));
  }

  /** Direkt açı hedefi (degree). */
  public void setTargetAngleDeg(double targetDeg) {
    // mekanik limitlerde tut
    targetDeg = MathUtil.clamp(targetDeg, ShooterConstants.kHoodMinDeg, ShooterConstants.kHoodMaxDeg);
    lastTargetDeg = targetDeg;

    double currentDeg = getAngleDeg();

    // PID output
    double out = pid.calculate(currentDeg, targetDeg);

    // küçük hatada motoru titretmesin diye deadband
    out = MathUtil.applyDeadband(out, ShooterConstants.kHoodOutputDeadband);

    // stiction compensation: hareket etmesi için minimum güç ekle
    if (Math.abs(out) > 0) {
      out = Math.copySign(Math.abs(out) + ShooterConstants.kHoodKs, out);
    }

    // son clamp: VictorSP -1..+1
    out = MathUtil.clamp(out, -ShooterConstants.kHoodMaxOutput, ShooterConstants.kHoodMaxOutput);

    // soft limit koruması (ek güvenlik)
    if (currentDeg >= ShooterConstants.kHoodMaxDeg && out > 0) out = 0;
    if (currentDeg <= ShooterConstants.kHoodMinDeg && out < 0) out = 0;

    hoodMotor.set(out);
  }

  /** Mesafeden hedef açı. */
  public void setAngleBasedOnDistance(double distanceMeters) {
    // tablo dışında gelirse clamp’le
    double d = MathUtil.clamp(distanceMeters, ShooterConstants.kHoodMinTableM, ShooterConstants.kHoodMaxTableM);

    Double target = hoodTable.get(d);
    if (target == null) {
      // fallback: güvenli orta açı
      target = ShooterConstants.kHoodSafeDeg;
    }
    setTargetAngleDeg(target);
  }

  public boolean isAtSetpoint() {
    return pid.atSetpoint();
  }

  public double getLastTargetDeg() {
    return lastTargetDeg;
  }

  public void stop() {
    hoodMotor.set(0);
  }
}