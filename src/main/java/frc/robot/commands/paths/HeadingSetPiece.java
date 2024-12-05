package frc.robot.commands.paths;

public class HeadingSetPiece extends CommandPathPiece{
    MultiPartPath path;
    double offset = 0; // if it does not follow trajectory, this is just the absolute angle
    boolean followTrajectory = false;
    public HeadingSetPiece(MultiPartPath path, double offset, boolean followTrajectory){
        this.path = path;
        this.offset = offset;
        this.followTrajectory = followTrajectory;
    }
    @Override
    public void initialize() {
        path.headingFollowMovement = followTrajectory;
        if(followTrajectory){
            path.headingOffset = offset;
        }else{
            path.targetRotationDegrees = offset;
        }
        
    }

}
