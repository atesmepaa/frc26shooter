package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.LedSubsystem;
import frc.robot.subsystems.LedSubsystem.LedMode;
import frc.robot.subsystems.ShooterSubsystem;

public class IntakeCommand extends Command {
  private final ShooterSubsystem shooter;
  private final LedSubsystem leds;

  public IntakeCommand(ShooterSubsystem shooter, LedSubsystem leds) {
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
    shooter.runFeeder(ShooterConstants.kFeederIntakeSpeed);
    shooter.runIntake(ShooterConstants.kIntakeCollectSpeed);
  }

  @Override
  public void end(boolean interrupted) {
    shooter.runIntake(0);
    leds.setMode(LedMode.IDLE);
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
