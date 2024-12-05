package frc.robot.subsystems;

import java.io.IOException;
import java.util.ArrayList;
import java.util.List;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonTrackedTarget;
import org.photonvision.targeting.TargetCorner;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
//import frc.robot.Util;
import frc.robot.Utils.RollingAverage;

public class VisionSubsystem extends SubsystemBase {

    public static PhotonCamera photonCamera;
    public static PhotonCamera noteCamera;
    public static AprilTagFieldLayout layout;
    public boolean canSeeTarget = false;

    final double CAMERA_HEIGHT_METERS = 0.216;

    // Angle between horizontal and the camera.
    final double CAMERA_PITCH_RADIANS = Units.degreesToRadians(25);

    public static PhotonPoseEstimator poseEstimator;

    public static RollingAverage distanceAverage = new RollingAverage(1);

    List<PhotonTrackedTarget> notes = new ArrayList<PhotonTrackedTarget>();

    public PhotonTrackedTarget bestTarget = null;
    double camHeight = Units.inchesToMeters(27); //
    Transform3d transform3d = new Transform3d(new Translation3d(0.381, 0.1845, -0.2159),
            new Rotation3d(0, Math.toRadians(25), 0));

    public VisionSubsystem() {

        try {
            layout = AprilTagFieldLayout.loadFromResource(AprilTagFields.k2024Crescendo.m_resourceFile);
        } catch (IOException e) {
            // TODO Auto-generated catch block
            e.printStackTrace();
        }

        noteCamera = new PhotonCamera("NoteCam");
        photonCamera = new PhotonCamera("USB_2M_GS_camera");
        poseEstimator = new PhotonPoseEstimator(layout, PoseStrategy.LOWEST_AMBIGUITY, photonCamera,
                transform3d);

    }

    public static double visToRealAngle(double ang) {

        double x = ang;
        double result = -0.0014616533340579727*x*x + 1.2844392880100985*x + 3.056435799133628; // CURVE:real,04:09,04/03
        return result;
    }

     public static double visToRealYaw(double ang) {

        double x = ang;
        double result = 0.001375129001337856*x*x + 1.2974749941216455*x + -0.5037514979083114; // CURVE:realyaw,04:49,04/03
        return result;
    }

    public List<PhotonTrackedTarget> getTargets() {

        return notes;
    }

    @Override
    public void periodic() {
        notes = noteCamera.getLatestResult().targets;
        poseEstimator.setReferencePose(RobotContainer.drivetrain.getPose()); // sets reference pose to (0,0,
                                                                             // Rotation2d.fromDegrees(0))
        var estimated = poseEstimator.update();
        var result = photonCamera.getLatestResult();

        if (estimated.isPresent() && result.targets.size() >= 2) {

            // var newPose = estimated.get();
            // SmartDashboard.putNumber("Tags Visible", newPose.targetsUsed.size());

            // // RobotContainer.drivetrain.setPose(newPose.estimatedPose.toPose2d());

            // RobotContainer.drivetrain.addVisionMeasurment(
            // newPose.estimatedPose.toPose2d(),
            // ((System.currentTimeMillis()-result.getLatencyMillis())/1000.0));
            // In photonvision, need to have matching photonvision versions, also, need NT
            // connected as well.

        }

        SmartDashboard.putNumber("latency", result.getLatencyMillis());
        PhotonTrackedTarget bestTarget = null;
        //System.out.println("start");
        if(result.hasTargets()){

            for(PhotonTrackedTarget possible: result.targets){
                if(possible.getFiducialId() == 9 || possible.getFiducialId() == 10 || possible.getFiducialId() == 1 || possible.getFiducialId() == 2){ continue;}
                //System.out.println("NUMBER"+possible.getFiducialId()+" is "+possible.getYaw());
                if(bestTarget == null || Math.abs(possible.getYaw()) < Math.abs(bestTarget.getYaw())){
                    bestTarget = possible;
                    //System.out.println("newbest:"+bestTarget.getFiducialId());
                }
            }
        }
        if (bestTarget != null && Math.abs(bestTarget.getYaw()) < 7) {
                    SmartDashboard.putBoolean("Can it see a tag? ",true);

            // var bestTarget = result.getBestTarget();
            //System.out.println("used "+bestTarget.getFiducialId());
            SmartDashboard.putNumber("used", bestTarget.getFiducialId());
            var tagPose = layout.getTagPose(bestTarget.getFiducialId());

            double yaw = 180 + visToRealYaw(bestTarget.getYaw()) -
                    RobotContainer.drivetrain.getGyroDegrees();

            TargetCorner bottomCorner = bestTarget.getDetectedCorners().get(0);
            TargetCorner topCorner = bestTarget.getDetectedCorners().get(3);

            double height = bottomCorner.y - topCorner.y;
            double targetsY = topCorner.y + height / 2;
            double pitch = visToRealAngle(bestTarget.getPitch());
            double distance = PhotonUtils.calculateDistanceToTargetMeters(
                    CAMERA_HEIGHT_METERS,
                    tagPose.get().getZ(),
                    CAMERA_PITCH_RADIANS,
                    Units.degreesToRadians(pitch));

            /*Pose2d newPose = PhotonUtils.estimateFieldToRobot(CAMERA_HEIGHT_METERS, tagPose.get().getZ(), CAMERA_PITCH_RADIANS,
                    Units.degreesToRadians(bestTarget.getPitch()), Rotation2d.fromDegrees(bestTarget.getYaw()),
                    Rotation2d.fromDegrees(-RobotContainer.drivetrain.getGyroDegrees()), new Pose2d(tagPose.get().getX(), tagPose.get().getY(), new Rotation2d()),
                    new Transform2d(transform3d.getX(), transform3d.getY(), new Rotation2d()));*/

            distanceAverage.put(distance);
            SmartDashboard.putNumber("visDistance", distance);

            double y = Math.sin(Math.toRadians(yaw)) * distanceAverage.get();
            double x = Math.cos(Math.toRadians(yaw)) * distanceAverage.get();

            double robotX = tagPose.get().getX() + -x;
            double robotY = tagPose.get().getY() + y;

            robotX +=(transform3d.getX() *
            Math.cos(RobotContainer.drivetrain.getGyroRadians())) - (transform3d.getY() *
            Math.sin(RobotContainer.drivetrain.getGyroRadians()));
            robotY+= (transform3d.getX() *
            Math.sin(RobotContainer.drivetrain.getGyroRadians())) + (transform3d.getY() *
            Math.cos(RobotContainer.drivetrain.getGyroRadians()));
            
            RobotContainer.drivetrain.addVisionMeasurement(new Pose2d(robotX, robotY, new Rotation2d(RobotContainer.drivetrain.getGyroRadians())), System.currentTimeMillis() - result.getLatencyMillis());
            return;
        } else{
                                SmartDashboard.putBoolean("Can it see a tag? ",false);

        }
    }
}