package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.LedSubsystem;
import frc.robot.subsystems.LedSubsystem.LedMode;
import frc.robot.subsystems.ShooterSubsystem;

public class ReverseCommand extends Command {
  private final ShooterSubsystem shooter;
  private final LedSubsystem leds;

  public ReverseCommand(ShooterSubsystem shooter, LedSubsystem leds) {
    this.shooter = shooter;
    this.leds = leds;
    addRequirements(shooter, leds);
  }

  @Override
  public void initialize() {
    leds.setMode(LedMode.AIMING);
  }

  @Override
  public void execute() {
    shooter.reverseAll(ShooterConstants.kReverseSpeed);
  }

  @Override
  public void end(boolean interrupted) {
    shooter.reverseAll(0);
    leds.setMode(LedMode.IDLE);
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
