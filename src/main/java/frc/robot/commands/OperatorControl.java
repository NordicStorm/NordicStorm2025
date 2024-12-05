package frc.robot.commands;

import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.generated.TunerConstants;

public class OperatorControl extends Command {

    private double MaxSpeed = TunerConstants.kSpeedAt12VoltsMps; // kSpeedAt12VoltsMps desired top speed
    private double MaxAngularRate = 0.25 * Math.PI; // 3/4 of a rotation per second max angular velocity (Typically 1.5)

    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.Velocity); // I want field-centric
                                                              // driving in open loop

    public OperatorControl() {

    }

    @Override
    public void execute() {
        if (RobotContainer.joystick.getAButton()) {
            RobotContainer.drivetrain.applyRequest(brake);
        } else if (RobotContainer.joystick.getBButton()) {
            RobotContainer.drivetrain.applyRequest(
                    point.withModuleDirection(new Rotation2d(
                            -RobotContainer.joystick.getLeftY(), -RobotContainer.joystick.getLeftX())));
        } else {
            RobotContainer.drivetrain.applyRequest(drive.withVelocityX(-RobotContainer.joystick.getLeftY() * MaxSpeed) // Drive
                                                                                                                       // forward
                    // with
                    // negative Y (forward)
                    .withVelocityY(-RobotContainer.joystick.getLeftX() * MaxSpeed) // Drive left with negative X (left)
                    .withRotationalRate(-RobotContainer.joystick.getRightX() * MaxAngularRate) // Drive counterclockwise
                                                                                               // with negative X
            // (left)
            );
        }

    }

}