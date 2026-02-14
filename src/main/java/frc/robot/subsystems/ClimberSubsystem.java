package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClimbConstants;

public class ClimberSubsystem extends SubsystemBase {

  private final Solenoid climber =
      new Solenoid(ClimbConstants.kModuleType,
                   ClimbConstants.kSolenoidChannel);

  public ClimberSubsystem() {
    climber.set(false); // robot açılışında kapalı
  }

  /** pnömatik açıkken */
  public void deploy() {
    climber.set(true);
  }

  /** pnömatiği siktirledikten sonra */
  public void stow() {
    climber.set(false);
  }

  public boolean isDeployed() {
    return climber.get();
  }
}