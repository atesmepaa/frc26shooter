package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.motorcontrol.VictorSP;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstants;

public class ShooterSubsystem extends SubsystemBase {
  private final SparkMax flyNeo1;
  private final SparkMax flyNeo2;
  private final SparkMax feedNeo;

  private final VictorSP feedCim;
  private final VictorSP transferCim;
  private final VictorSP intakeCim;

  public ShooterSubsystem() {
    flyNeo1 = new SparkMax(ShooterConstants.kShooterNeo1ID, MotorType.kBrushless);
    flyNeo2 = new SparkMax(ShooterConstants.kShooterNeo2ID, MotorType.kBrushless);
    feedNeo = new SparkMax(ShooterConstants.kFeederNeoID, MotorType.kBrushless);

    feedCim = new VictorSP(ShooterConstants.kFeederCimPort);
    transferCim = new VictorSP(ShooterConstants.kTransferCimPort);
    intakeCim = new VictorSP(ShooterConstants.kIntakeCimPort);

    // --- Flywheel leader config ---
    SparkMaxConfig leaderCfg = new SparkMaxConfig();
    leaderCfg
      .idleMode(IdleMode.kCoast)
      .smartCurrentLimit(ShooterConstants.kFlywheelCurrentLimit)
      .closedLoopRampRate(ShooterConstants.kFlywheelRamp);

    leaderCfg.closedLoop
      .p(ShooterConstants.kFlywheelP)
      .i(ShooterConstants.kFlywheelI)
      .d(ShooterConstants.kFlywheelD)
      .outputRange(-1, 1);

    // --- Flywheel follower config ---
    SparkMaxConfig followerCfg = new SparkMaxConfig();
    followerCfg.follow(flyNeo1, false);

    // --- Feeder NEO config ---
    SparkMaxConfig feederCfg = new SparkMaxConfig();
    feederCfg
      .idleMode(IdleMode.kBrake)
      .smartCurrentLimit(ShooterConstants.kFeederCurrentLimit)
      .openLoopRampRate(0.15);

    flyNeo1.configure(leaderCfg, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    flyNeo2.configure(followerCfg, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    feedNeo.configure(feederCfg, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    // CIM invert gerekirse constants ile yönet
    feedCim.setInverted(ShooterConstants.kFeederCimInverted);
    transferCim.setInverted(ShooterConstants.kTransferInverted);
    intakeCim.setInverted(ShooterConstants.kIntakeInverted);
  }

  /** Flywheel hedef RPM (RPM varsayıyoruz) */
  public void runFlywheelRPM(double rpm) {
    rpm = MathUtil.clamp(rpm, 0, ShooterConstants.kMaxFlywheelRPM);
    flyNeo1.getClosedLoopController().setReference(rpm, ControlType.kVelocity);
  }

  public double getFlywheelRPM() {
    return flyNeo1.getEncoder().getVelocity();
  }

  public boolean isFlywheelReady(double targetRPM) {
    return Math.abs(getFlywheelRPM() - targetRPM) < ShooterConstants.kFlywheelReadyToleranceRPM;
  }

  /** Feeder (NEO + CIM) */
  public void runFeeder(double speed) {
    speed = MathUtil.clamp(speed, -1, 1);
    feedNeo.set(speed);
    feedCim.set(speed);
  }

  /** Intake + transfer */
  public void runIntake(double speed) {
    speed = MathUtil.clamp(speed, -1, 1);
    intakeCim.set(speed);
    transferCim.set(speed);
  }

  /** Unjam: hepsini ters çalıştır */
  public void reverseAll(double speed) {
    speed = Math.abs(speed);
    runFeeder(-speed);
    runIntake(-speed);
  }

  public void stopAll() {
    flyNeo1.set(0);
    feedNeo.set(0);
    feedCim.set(0);
    transferCim.set(0);
    intakeCim.set(0);
  }
}