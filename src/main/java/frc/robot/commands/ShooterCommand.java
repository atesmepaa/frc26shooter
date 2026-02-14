package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.HoodSubsystem;
import frc.robot.subsystems.LedSubsystem;
import frc.robot.subsystems.LedSubsystem.LedMode;
import frc.robot.subsystems.ShooterSubsystem;

public class ShooterCommand extends Command {
  private final ShooterSubsystem shooter;
  private final HoodSubsystem hood;
  private final LedSubsystem leds;
  private final DoubleSupplier distanceMeters;

  private double targetRpm = 0;

  public ShooterCommand(
      ShooterSubsystem shooter,
      HoodSubsystem hood,
      LedSubsystem leds,
      DoubleSupplier distanceMeters) {

    this.shooter = shooter;
    this.hood = hood;
    this.leds = leds;
    this.distanceMeters = distanceMeters;

    // HOOD'u REQUIRE ETMİYORUZ -> default aim kesilmez
    addRequirements(shooter, leds);
  }

  @Override
  public void initialize() {
    leds.setMode(LedMode.SPINNING_UP);
  }

  @Override
  public void execute() {
    double d = distanceMeters.getAsDouble();

    // Hood hedefini default command zaten güncelliyor.
    // Burada hood.setAngleBasedOnDistance(d); YOK!

    Double rpm = ShooterConstants.kRpmTable.get(d);
    targetRpm = (rpm != null) ? rpm : ShooterConstants.kSafeRpm;

    shooter.runFlywheelRPM(targetRpm);

    boolean flyReady = shooter.isFlywheelReady(targetRpm);
    boolean hoodReady = hood.isAtSetpoint();
    boolean canFire = flyReady && hoodReady;

    if (!flyReady) leds.setMode(LedMode.SPINNING_UP);
    else if (!hoodReady) leds.setMode(LedMode.AIMING);
    else leds.setMode(LedMode.FIRING);

    if (canFire) {
      shooter.runFeeder(ShooterConstants.kFeederShootSpeed);
      shooter.runIntake(ShooterConstants.kIntakeShootSpeed);
    } else {
      shooter.runFeeder(0);
      shooter.runIntake(0);
    }
  }

  @Override
  public void end(boolean interrupted) {
    shooter.stopAll();
    leds.setMode(LedMode.IDLE);
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}