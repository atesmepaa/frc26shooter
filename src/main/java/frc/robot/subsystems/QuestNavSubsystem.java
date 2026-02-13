package frc.robot.subsystems;

import java.util.OptionalInt;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.subsystems.Swerve.DriveTrain;
import gg.questnav.questnav.PoseFrame;
import gg.questnav.questnav.QuestNav;

public class QuestNavSubsystem extends SubsystemBase {

    // ---------------- QUEST ----------------
    private final QuestNav questNav = new QuestNav();

    // ---------------- SWERVE ----------------
    private final DriveTrain swerveDrive;

    // ---------------- POSE ----------------
    private Pose2d lastRawPose = new Pose2d();
    private Pose2d offset = new Pose2d();
    private boolean hasPose = false;

    // ---------------- BATTERY ----------------
    private int questBatteryPercent = -1;

    // ---------------- TRACKING ----------------
    private boolean questTracking = false;

    private boolean questConnected = false;

    // ---------------- TRANSFORM ----------------
    // Quest arkada ve arkaya bakıyor (yaw = PI)
    private static final Transform3d ROBOT_TO_QUEST =
        new Transform3d(
            new Translation3d(-0.30, 0.15, 0.40),
            new Rotation3d(0.0, 0.0, Math.PI)
        );

    // ---------------- STD DEVS ----------------
    private static final Matrix<N3, N1> QUEST_STD_DEVS =
        VecBuilder.fill(0.02, 0.02, 0.035);

    // ---------------- CONSTRUCTOR ----------------
    public QuestNavSubsystem(DriveTrain driveTrain) {
        this.swerveDrive = driveTrain;
    }

    // ---------------- PERIODIC ----------------
    @Override
    public void periodic() {

        // -------- BATTERY --------
        OptionalInt batteryOpt = questNav.getBatteryPercent();
        questBatteryPercent = batteryOpt.isPresent()
            ? batteryOpt.getAsInt()
            : -1;

        // -------- TRACKING FLAG RESET --------
        questTracking = false;

        

        // -------- POSE FRAMES --------
        PoseFrame[] frames = questNav.getAllUnreadPoseFrames();
        PoseFrame latestFrame = null;

        for (PoseFrame frame : frames) {
            if (frame.isTracking()) {
                latestFrame = frame;
            }
        }

        if (latestFrame != null) {
            Pose3d robotPose3d =
                latestFrame.questPose3d()
                    .transformBy(ROBOT_TO_QUEST.inverse());

            lastRawPose = robotPose3d.toPose2d();
            hasPose = true;
            questTracking = true;
            


            // OFFSET APPLY
            Pose2d correctedPose = lastRawPose.relativeTo(offset);

            // SEND TO POSE ESTIMATOR
            swerveDrive.addVisionMeasurement(
                correctedPose,
                latestFrame.dataTimestamp(),
                QUEST_STD_DEVS
            );
        }

        publishToDashboard();
    }

    // ---------------- DASHBOARD ----------------
    private void publishToDashboard() {
        SmartDashboard.putBoolean("Quest/Tracking", questTracking);
        SmartDashboard.putBoolean("Quest/Connected", questNav.isConnected());
        SmartDashboard.putNumber("Quest/Battery", questBatteryPercent);

        if (hasPose) {
            Pose2d pose = lastRawPose.relativeTo(offset);
            SmartDashboard.putNumber("Quest/X", pose.getX());
            SmartDashboard.putNumber("Quest/Y", pose.getY());
            SmartDashboard.putNumber(
                "Quest/RotDeg",
                pose.getRotation().getDegrees()
            );
        }
    }

    // ---------------- PUBLIC API ----------------

    /** OFFSET uygulanmış Quest pozu (navX getYaw muadili) */
    public Pose2d getPose2d() {
        if (!hasPose) {
            return new Pose2d();
        }
        return lastRawPose.relativeTo(offset);
    }

    /** navX.reset() MUADİLİ */
    public void zeroPose() {
        if (hasPose) {
            offset = lastRawPose;
        }
    }

    public boolean isTracking() {
        return questTracking;
    }

    public int getBatteryPercent() {
        return questBatteryPercent;
    }

    public boolean isConnected(){
        return questConnected;
    }
}
