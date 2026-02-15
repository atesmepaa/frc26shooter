package frc.robot.commands;

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

  public ShooterCommand(
      ShooterSubsystem shooter,
      HoodSubsystem hood,
      LedSubsystem leds) {

    this.shooter = shooter;
    this.hood = hood;
    this.leds = leds;

    addRequirements(shooter, leds);
  }

  @Override
  public void initialize() {
    leds.setMode(LedMode.SPINNING_UP);
  }

  @Override
  public void execute() {
    double targetRpm = shooter.getTargetRpm();
    shooter.runFlywheelRPM(targetRpm);

    boolean flyReady = shooter.isFlywheelReady(targetRpm);
    boolean hoodReady = hood.isAtSetpoint();
    boolean canFire = flyReady && hoodReady;

    if (!flyReady) leds.setMode(LedMode.SPINNING_UP);
    else if (!hoodReady) leds.setMode(LedMode.AIMING);
    else leds.setMode(LedMode.FIRING);

    if (canFire) {
      shooter.runFeeder(ShooterConstants.kFeederShootSpeed);
    } else {
      shooter.runFeeder(0);
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
