package frc.robot.subsystems;

import java.util.function.Supplier;


import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrain;
import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.commands.paths.DriveTrainConfig;
import frc.robot.commands.paths.PathableDrivetrain;

/**
 * Class that extends the Phoenix SwerveDrivetrain class and implements
 * subsystem so it can be used in command-based projects easily.
 */
public class CommandSwerveDrivetrain extends SwerveDrivetrain implements Subsystem, PathableDrivetrain {
    private static final double kSimLoopPeriod = 0.005; // 5 ms
    private Notifier m_simNotifier = null;
    private double m_lastSimTime;

     private final SwerveRequest.ApplyChassisSpeeds driveRequest = new SwerveRequest.ApplyChassisSpeeds()
            .withDriveRequestType(DriveRequestType.Velocity); // I want field-centric


    /* Blue alliance sees forward as 0 degrees (toward red alliance wall) */
    private final Rotation2d BlueAlliancePerspectiveRotation = Rotation2d.fromDegrees(0);
    /* Red alliance sees forward as 180 degrees (toward blue alliance wall) */
    private final Rotation2d RedAlliancePerspectiveRotation = Rotation2d.fromDegrees(180);
    /* Keep track if we've ever applied the operator perspective before or not */
    private boolean hasAppliedOperatorPerspective = false;

    private DriveTrainConfig drivetrainConfig = new DriveTrainConfig();

    public CommandSwerveDrivetrain(SwerveDrivetrainConstants driveTrainConstants, double OdometryUpdateFrequency, SwerveModuleConstants... modules) {
        super(driveTrainConstants, OdometryUpdateFrequency, modules);
        if (Utils.isSimulation()) {
            startSimThread();
        }
        drivetrainConfig.maxAcceleration = 4;
        drivetrainConfig.maxVelocity = 4;
        drivetrainConfig.maxAnglularVelocity = 10;
        drivetrainConfig.maxAngularAcceleration = 5;
        drivetrainConfig.rotationCorrectionP = 3;
        drivetrainConfig.maxCentripetalAcceleration = 8;
    }
    public CommandSwerveDrivetrain(SwerveDrivetrainConstants driveTrainConstants, SwerveModuleConstants... modules) {
        super(driveTrainConstants, modules);
        if (Utils.isSimulation()) {
            startSimThread();
        }
        drivetrainConfig.maxAcceleration = 4;
        drivetrainConfig.maxVelocity = 4;
        drivetrainConfig.maxAnglularVelocity = 10;
        drivetrainConfig.maxAngularAcceleration = 5;
        drivetrainConfig.rotationCorrectionP = 3;
        drivetrainConfig.maxCentripetalAcceleration = 8;
    }

    public void applyRequest(SwerveRequest request) {
        this.setControl(request);
    }

    public void drive(double xVelo, double yVelo, double rotation){
        drive(new ChassisSpeeds(xVelo, yVelo, rotation));
    }
    
    @Override
    public void drive(ChassisSpeeds speeds){
        applyRequest(driveRequest.withSpeeds(speeds));
    }

    private void startSimThread() {
        m_lastSimTime = Utils.getCurrentTimeSeconds();

        /* Run simulation at a faster rate so PID gains behave more reasonably */
        m_simNotifier = new Notifier(() -> {
            final double currentTime = Utils.getCurrentTimeSeconds();
            double deltaTime = currentTime - m_lastSimTime;
            m_lastSimTime = currentTime;

            /* use the measured time delta, get battery voltage from WPILib */
            updateSimState(deltaTime, RobotController.getBatteryVoltage());
        });
        m_simNotifier.startPeriodic(kSimLoopPeriod);
    }

    @Override
    public void periodic() {
        /* Periodically try to apply the operator perspective */
        /* If we haven't applied the operator perspective before, then we should apply it regardless of DS state */
        /* This allows us to correct the perspective in case the robot code restarts mid-match */
        /* Otherwise, only check and apply the operator perspective if the DS is disabled */
        /* This ensures driving behavior doesn't change until an explicit disable event occurs during testing*/
        if (!hasAppliedOperatorPerspective || DriverStation.isDisabled()) {
            DriverStation.getAlliance().ifPresent((allianceColor) -> {
                this.setOperatorPerspectiveForward(
                        allianceColor == Alliance.Red ? RedAlliancePerspectiveRotation
                                : BlueAlliancePerspectiveRotation);
                hasAppliedOperatorPerspective = true;
            });
        }
    }
    @Override
    public double getGyroRadians() {
        return getRotation3d().getZ();
    }
    public double getGyroDegrees(){
        return Math.toDegrees(getGyroRadians());
    }
    @Override
    public Pose2d getPose() {
        return getState().Pose;
    }
    @Override
    public void setPose(Pose2d pose) {
        seedFieldRelative(pose);
    }
    @Override
    public ChassisSpeeds getSpeeds() {
        return getState().speeds;
    }
    @Override
    public DriveTrainConfig getConfig() {
        return drivetrainConfig;
    }
}
