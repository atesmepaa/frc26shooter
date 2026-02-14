package frc.robot;

import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.QuestNavSubsystem;
import frc.robot.subsystems.Swerve.DriveTrain;

import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.HoodSubsystem;
import frc.robot.subsystems.LedSubsystem;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.commands.IntakeCommand;
import frc.robot.commands.ReverseCommand;
import frc.robot.commands.ShooterCommand;

public class RobotContainer {

  private final SendableChooser<Command> autoChooser;

  // SWERVE
  private final DriveTrain drivetrain = new DriveTrain();
  private final QuestNavSubsystem questNav = new QuestNavSubsystem(drivetrain);

  // SUPERSTRUCTURE
  private final ShooterSubsystem shooter = new ShooterSubsystem();
  private final HoodSubsystem hood = new HoodSubsystem();
  private final LedSubsystem leds = new LedSubsystem();
  private final ClimberSubsystem climber = new ClimberSubsystem();

  // Xbox
  public static final CommandXboxController primary  = new CommandXboxController(OIConstants.primaryPort);   // driver

  public RobotContainer() {
    var distanceSupplier = (java.util.function.DoubleSupplier) drivetrain::getDistanceToShotTargetMeters;
    hood.setDistanceSupplier(distanceSupplier);
    shooter.setDistanceSupplier(distanceSupplier);

    configureBindings();

    autoChooser = AutoBuilder.buildAutoChooser();
    SmartDashboard.putData("Auto Chooser", autoChooser);

    // Default drive
    drivetrain.setDefaultCommand(
      new RunCommand(
        () -> {
          double ySpeed = -MathUtil.applyDeadband(primary.getLeftY(),  OIConstants.driveDeadband);
          double xSpeed = -MathUtil.applyDeadband(primary.getLeftX(),  OIConstants.driveDeadband);
          double rot    = -MathUtil.applyDeadband(primary.getRightX(), OIConstants.driveDeadband);
          drivetrain.drive(ySpeed, xSpeed, rot, true);
        },
        drivetrain
      )
    );
  }

  private void configureBindings() {

    // Driver: Start = gyro + questnav reset
    primary.start().onTrue(Commands.runOnce(() -> drivetrain.zeroHeading()));
    primary.start().onTrue(Commands.runOnce(() -> questNav.zeroPose()));

    // Driver: X hold = setX (defans lock)
    primary.x().whileTrue(new RunCommand(() -> drivetrain.setX(), drivetrain));

    // Driver: swerve “target / lock” modları (seninki aynen)
    primary.rightTrigger().whileTrue(
      new RunCommand(
        () -> drivetrain.driveAtTarget(
          -MathUtil.applyDeadband(primary.getLeftY(), 0.1),
          -MathUtil.applyDeadband(primary.getLeftX(), 0.1)
        ),
        drivetrain
      )
    );

    primary.rightBumper().whileTrue(
      new RunCommand(
        () -> drivetrain.lockFront(
          -MathUtil.applyDeadband(primary.getLeftY(), 0.1),
          -MathUtil.applyDeadband(primary.getLeftX(), 0.1)
        ),
        drivetrain
      )
    );

    primary.leftBumper().whileTrue(
      new RunCommand(
        () -> drivetrain.lockBack(
          -MathUtil.applyDeadband(primary.getLeftY(), 0.1),
          -MathUtil.applyDeadband(primary.getLeftX(), 0.1)
        ),
        drivetrain
      )
    );

    Command shootCmd = new ShooterCommand(shooter, hood, leds);
    Command intakeCmd = new IntakeCommand(shooter, leds);
    Command reverseCmd = new ReverseCommand(shooter, leds);
    new Trigger(() -> primary.getLeftTriggerAxis() > 0.2).whileTrue(shootCmd);
    primary.y().whileTrue(intakeCmd);
    primary.b().whileTrue(reverseCmd);

    // -----------------------------
    // OPERATOR: Climb (A basılıyken deploy, bırakınca stow)
    // -----------------------------
    primary.a().whileTrue(
      new RunCommand(climber::deploy, climber)
        .finallyDo(interrupted -> climber.stow())
    );

    // Opsiyonel: operator.start() ile “her şeyi stop”
    primary.start().onTrue(Commands.runOnce(() -> {
      shooter.stopAll();
      climber.stow();
      leds.setMode(frc.robot.subsystems.LedSubsystem.LedMode.IDLE);
    }));
  }

  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }
}
