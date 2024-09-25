package frc.robot;



import static frc.robot.Constants.Vision.*;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import java.util.Optional;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;

import frc.robot.subsystems.SwerveSubsystem;

public class Vision {
    private final PhotonCamera camera;
    private final PhotonCamera cameraL;
    private final PhotonCamera cameraF;
    private final PhotonPoseEstimator photonEstimator;
    private final PhotonPoseEstimator photonEstimatorL;
    private final PhotonPoseEstimator photonEstimatorF;
    private double lastEstTimestamp = 0;
    private final Field2d m_field = new Field2d();
    public static double xErr;
    public static double yErr;
    public static double rotationErr;
    public static double targetDistance=0;
    public static boolean shotReady;
    public static double shotReadyCycles = 0;
    public static double shootAngleOffset = 0; //for new off-axis shooter - populated from lookup table
    public static double desiredAngle = 0;
    public static double goalPosOffset = 0.32;


    private VisionLookupTable backShotLow_Lookup = new VisionLookupTable(Constants.shooterLookup.backShotLow);
    private VisionLookupTable backShotHigh_Lookup = new VisionLookupTable(Constants.shooterLookup.backShotHigh);
    private VisionLookupTable forwardShotLow_Lookup = new VisionLookupTable(Constants.shooterLookup.forwardShotLow);
    private VisionLookupTable forwardShotHigh_Lookup = new VisionLookupTable(Constants.shooterLookup.forwardShotHigh);
    private VisionLookupTable tallPodiumOnly = new VisionLookupTable(Constants.shooterLookup.tallPodiumBack);

    // Simulation
    //private PhotonCameraSim cameraSim;
    //private VisionSystemSim visionSim;

    
    public Vision() {
        camera = new PhotonCamera("Arducam_Camera_Right"); 
        cameraL = new PhotonCamera("Arducam_Left");
        cameraF = new PhotonCamera("Front");
        
        photonEstimator =
                new PhotonPoseEstimator(
                        kTagLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, camera, kRobotToCam);
        photonEstimatorL = 
                new PhotonPoseEstimator(
                        kTagLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, cameraL, kRobotToCamL);
        photonEstimatorF = 
                new PhotonPoseEstimator(
                        kTagLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, cameraF, kRobotToCamF);
                        
        photonEstimatorF.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);
        photonEstimatorL.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);
        photonEstimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);
        //m_field = photonEstimator.; //get m_field from odom  
        SmartDashboard.putData("FIELD", m_field);   
//        SmartDashboard.putString("photonPos",photonEstimator.update().toString());

    }

    public PhotonPipelineResult getLatestResult() {
        if (camera.getLatestResult().hasTargets()) {
            return camera.getLatestResult();
        } else if(cameraF.getLatestResult().hasTargets()) {
            return cameraF.getLatestResult();
        } else{
            return cameraL.getLatestResult();
        }
    }
    

    public static void goalPos(double beepboop) {
        goalPosOffset += beepboop;
        SmartDashboard.putNumber("Goal Offset", goalPosOffset);
    }

    public Pose2d getPhotonEstimatedPose2d(){
        if (photonEstimator.update().isPresent()) {
            return photonEstimator.getReferencePose().toPose2d();
        } else {
            return photonEstimatorL.getReferencePose().toPose2d();
        }
    }
    /**
     * The latest estimated robot pose on the field from vision data. This may be empty. This should
     * only be called once per loop.
     *
     * @return An {@link EstimatedRobotPose} with an estimated pose, estimate timestamp, and targets
     *     used for estimation.
     */
    public Optional<EstimatedRobotPose> getEstimatedGlobalPose() {

        var visionEst = photonEstimator.update();

        if (visionEst.isEmpty()) {
            visionEst = photonEstimatorL.update();
        }

        if (visionEst.isEmpty()) {
            visionEst = photonEstimatorF.update();
        }
        
        double latestTimestamp = camera.getLatestResult().getTimestampSeconds();
        boolean newResult = Math.abs(latestTimestamp - lastEstTimestamp) > 1e-5;
      /*  if (Robot.isSimulation()) {
            visionEst.ifPresentOrElse(
                    est ->
                            getSimDebugField()
                                    .getObject("VisionEstimation")
                                    .setPose(est.estimatedPose.toPose2d()),
                    () -> {
                        if (newResult) getSimDebugField().getObject("VisionEstimation").setPoses();
                    });
        }*/
        if (newResult) lastEstTimestamp = latestTimestamp;
        return visionEst;
    }

    /**
     * The standard deviations of the estimated pose from {@link #getEstimatedGlobalPose()}, for use
     * with {@link edu.wpi.first.math.estimator.SwerveDrivePoseEstimator SwerveDrivePoseEstimator}.
     * This should only be used when there are targets visible.
     *
     * @param estimatedPose The estimated pose to guess standard deviations for.
     */
    public Matrix<N3, N1> getEstimationStdDevs(Pose2d estimatedPose) {
        var estStdDevs = kSingleTagStdDevs;
        var targets = getLatestResult().getTargets();
        int numTags = 0;
        double avgDist = 0;
        for (var tgt : targets) {
            var tagPose = photonEstimator.getFieldTags().getTagPose(tgt.getFiducialId());
            if (tagPose.isEmpty()) continue;
            numTags++;
            avgDist +=
                    tagPose.get().toPose2d().getTranslation().getDistance(estimatedPose.getTranslation());
        }
        if (numTags == 0) return estStdDevs;
        avgDist /= numTags;
        // Decrease std devs if multiple targets are visible
        if (numTags > 1) estStdDevs = kMultiTagStdDevs;
        // Increase std devs based on (average) distance
        if (numTags == 1 && avgDist > 4)
            estStdDevs = VecBuilder.fill(Double.MAX_VALUE, Double.MAX_VALUE, Double.MAX_VALUE);
        else estStdDevs = estStdDevs.times(1 + (avgDist * avgDist / 30));

        return estStdDevs;
    }

    public void testLookupTables(double distance){
        SmartDashboard.putNumber("Base Angle Target", backShotLow_Lookup.getInterpolatedValue(distance,1)); // base angle
        System.out.println(backShotLow_Lookup.getInterpolatedValue(distance,1));
        SmartDashboard.putNumber("Endeffector Angle Target", backShotLow_Lookup.getInterpolatedValue(distance,2)); // base angle
        System.out.println(backShotLow_Lookup.getInterpolatedValue(distance,2));
        SmartDashboard.putNumber("Lower Speed Target", backShotLow_Lookup.getInterpolatedValue(distance,3)); // base angle
        System.out.println(backShotLow_Lookup.getInterpolatedValue(distance,3));
        SmartDashboard.putNumber("Upper Speed Target", backShotLow_Lookup.getInterpolatedValue(distance,4)); // base angle
        System.out.println(backShotLow_Lookup.getInterpolatedValue(distance,4));
        return;
    }

    public double[] getBackShotLowParameters (){
        double[] params = {
            backShotLow_Lookup.getInterpolatedValue(targetDistance,1), //base angle
            backShotLow_Lookup.getInterpolatedValue(targetDistance,2), //EE angle
            backShotLow_Lookup.getInterpolatedValue(targetDistance,3), //lower speed
            backShotLow_Lookup.getInterpolatedValue(targetDistance,4),  //Upper speed
            backShotLow_Lookup.getInterpolatedValue(targetDistance,5)  //yaw offset
        };
        shootAngleOffset = backShotLow_Lookup.getInterpolatedValue(targetDistance,5);
    return params;
    }
    
    public double[] getBackShotHighParameters (){
        double[] params = {
            backShotHigh_Lookup.getInterpolatedValue(targetDistance,1), //base angle
            backShotHigh_Lookup.getInterpolatedValue(targetDistance,2), //EE angle
            backShotHigh_Lookup.getInterpolatedValue(targetDistance,3), //lower speed
            backShotHigh_Lookup.getInterpolatedValue(targetDistance,4),  //Upper speed
            backShotHigh_Lookup.getInterpolatedValue(targetDistance,5)  //Upper speed
        };
        shootAngleOffset = backShotHigh_Lookup.getInterpolatedValue(targetDistance,5);
    return params;
    }

    public double[] getForwardShotLowParameters (){
        double[] params = {
            forwardShotLow_Lookup.getInterpolatedValue(targetDistance,1), //base angle
            forwardShotLow_Lookup.getInterpolatedValue(targetDistance,2), //EE angle
            forwardShotLow_Lookup.getInterpolatedValue(targetDistance,3), //lower speed
            forwardShotLow_Lookup.getInterpolatedValue(targetDistance,4),  //Upper speed
            forwardShotLow_Lookup.getInterpolatedValue(targetDistance,5)  //Upper speed
        };
        shootAngleOffset = forwardShotLow_Lookup.getInterpolatedValue(targetDistance,5);
    return params;
    }    

    public double[] getForwardShotHighParameters (){
        double[] params = {
            forwardShotHigh_Lookup.getInterpolatedValue(targetDistance,1), //base angle
            forwardShotHigh_Lookup.getInterpolatedValue(targetDistance,2), //EE angle
            forwardShotHigh_Lookup.getInterpolatedValue(targetDistance,3), //lower speed
            forwardShotHigh_Lookup.getInterpolatedValue(targetDistance,4),  //Upper speed
            forwardShotHigh_Lookup.getInterpolatedValue(targetDistance,5)  //Upper speed
        };
        shootAngleOffset = forwardShotHigh_Lookup.getInterpolatedValue(targetDistance,5);
    return params;
    }    
    
    public double[] getPodiumFixedDistance (){
        double[] params = {
            tallPodiumOnly.getInterpolatedValue(3,1), //base angle
            tallPodiumOnly.getInterpolatedValue(3,2), //EE angle
            tallPodiumOnly.getInterpolatedValue(3,3), //lower speed
            tallPodiumOnly.getInterpolatedValue(3,4),  //Upper speed
            tallPodiumOnly.getInterpolatedValue(3,5)  //Upper speed
        };
        shootAngleOffset = tallPodiumOnly.getInterpolatedValue(targetDistance,5);
    return params;
    }    
/*
    public double getBaseAngleTarget() {
        double angle = 0;
        return angle; //BOBBY ADD ARM TARGET FROM INTERPOLATOR
    }

    public double getEndeffectorAngleTarget() {
        double angle = 0;
        return angle; //BOBBY ADD ARM TARGET FROM INTERPOLATOR
    }

    public double getLowerShooterWheelSpeed() {
        double speed = 0;
        return speed; //BOBBY ADD WHEEL TARGET FROM INTERPOLATOR
    }

    public double getUpperShooterWheelSpeed() {
        double speed = 0;
        return speed; //BOBBY ADD WHEEL TARGET FROM INTERPOLATOR
    }
 */
/*
    public static void automaticAmpLineup(String REDorBLUE){
        double driveTargetX ; double driveXErr;
        double driveTargetY ; double driveYErr;
        double targetAngle  ; double targetAngleErr; 
        shootAngleOffset = 0;
        if (REDorBLUE.equals("BLUE")){//target is blue amp
            driveTargetX = 1.85; //meters
            driveTargetY = 7.76; //meters
            targetAngle = 90;  //opposite angle of target b/c front-scoring amp
        }
        else if (REDorBLUE.equals("RED")){//target is red amp
            driveTargetX = 14.7; //meters
            driveTargetY = 7.76; //meters
            targetAngle = 90; //opposite angle of target b/c front-scoring amp
        }
        else {//no target
            xErr = 0;
            yErr = 0;
            rotationErr = 0; //2-28 need to add this
            return;
        }
        
        driveXErr = SwerveSubsystem.getPoseSwerveOdometry().getX() - driveTargetX;
        driveYErr = SwerveSubsystem.getPoseSwerveOdometry().getY() - driveTargetY;
        targetAngleErr = SwerveSubsystem.getPoseFromPoseEstimator().getRotation().getDegrees() - targetAngle % 360;
        if (targetAngleErr > 180) targetAngleErr = targetAngleErr - 360;// 2-28-2024 - should prevent the flip around issue 
//        targetAngleErr =  SwerveSubsystem.getYaw().getDegrees() - targetAngle - 180;

        xErr = driveXErr;
        yErr = driveYErr;
        rotationErr = targetAngleErr;
        //SmartDashboard.putNumber("driveXErr", driveXErr);
        //SmartDashboard.putNumber("driveYErr", driveYErr);
        //SmartDashboard.putNumber("targetAngleErr", targetAngleErr);

        if (Math.abs(driveXErr) < 0.1 && Math.abs(driveXErr)<0.1 && targetAngleErr < 3){
            shotReadyCycles++;}
        else shotReadyCycles = 0;
        
        if (shotReadyCycles > 10) shotReady = true;
        else shotReady = false;
    }
 */
    public static void automaticShoot(String AUTOorTELE, String REDorBLUE){
        double aimTargetX ; double aimXErr; double driveTargetX ; double driveXErr; 
        double aimTargetY ; double aimYErr; double driveTargetY ; double driveYErr; 
        double targetAngle; double targetAngleErr; 

        if (REDorBLUE == null) {
            REDorBLUE = "BLUE";
        }
        
        if (REDorBLUE.equals("BLUE")){//target is blue speaker
            aimTargetX = -.038 + .25 ; //meters --- modded by .25 to aim at middle of speaker
            aimTargetY = 5.544; //meters
            driveTargetX = 1.5;
            if(SwerveSubsystem.getPoseSwerveOdometry().getX() > 12 && SwerveSubsystem.getPoseSwerveOdometry().getY() < 3){ //if at source
                aimTargetY += 4 + 2 + SmartDashboard.getNumber("far feed offset",0) ; //Y offset for near-amp feeder shot
            }else if(SwerveSubsystem.getPoseSwerveOdometry().getX() > 5){ //if midfield
                aimTargetY += 1 + SmartDashboard.getNumber("short feed offset",0) ; //Y offset for near-amp feeder shot
            }
            driveTargetY = 5.554;
            targetAngle = 0;// + shootAngleOffset;  //2-29-2024 added shoot angle offset to maps, updated when lookup is called for shot type
        }
        else if (REDorBLUE.equals("RED")){//target is red speaker
            aimTargetX = 16.566 - .25 + .1; //meters - third number is 
            aimTargetY = 5.544; //meters
            if(SwerveSubsystem.getPoseSwerveOdometry().getX() < 4 && SwerveSubsystem.getPoseSwerveOdometry().getY() < 3){ //if at source
                aimTargetY += 4 + 2 + SmartDashboard.getNumber("far feed offset",0) ; //Y offset for near-amp feeder shot
            }
            else if(SwerveSubsystem.getPoseSwerveOdometry().getX() < 11.5){ //if midfield
                aimTargetY += 1. + SmartDashboard.getNumber("short feed offset",0) ; //Y offset for near-amp feeder shot, was 2, changed for q24 to avoid robot jamm ing
            }
            driveTargetX = 15.0;
            driveTargetY = 5.554;
            targetAngle = 180;// + shootAngleOffset; //2-29-2024 added shoot angle offset to maps, updated when lookup is called for shot type
        }
        else {//no target 2-28-2024
            xErr = 0;
            yErr = 0;
            rotationErr = 0; //2-28 need to add this
            return;
        }
        
        aimXErr = aimTargetX - SwerveSubsystem.getPoseSwerveOdometry().getX();
        driveXErr = aimTargetY - SwerveSubsystem.getPoseSwerveOdometry().getX();
        
        aimYErr = aimTargetY - SwerveSubsystem.getPoseSwerveOdometry().getY() ;
        driveYErr = driveTargetY -SwerveSubsystem.getPoseSwerveOdometry().getY() ;

        if (REDorBLUE == "BLUE") desiredAngle = -((Math.atan(aimYErr / aimXErr)*180 / Math.PI) - targetAngle) + shootAngleOffset;
        if (REDorBLUE == "RED")  desiredAngle = -((Math.atan(aimYErr / aimXErr)*180 / Math.PI) - targetAngle) + shootAngleOffset;
//        targetAngleErr =  (desiredAngle + SwerveSubsystem.getPoseSwerveOdometry().getRotation().getDegrees()) % 360;
//        if (REDorBLUE == "BLUE" && targetDistance>6) desiredAngle = desiredAngle + 10;
//        if (REDorBLUE == "RED" && targetDistance>6) desiredAngle = desiredAngle - 8;//10
        targetAngleErr =  (desiredAngle + SwerveSubsystem.getPoseFromPoseEstimator().getRotation().getDegrees()) % 360;

        //changed reference to pose estimator for rotation instead of odometry
//        targetAngleErr = SwerveSubsystem.getPoseSwerveOdometry().getRotation().getDegrees() - (Math.atan(aimYErr / aimXErr)*180 / Math.PI) - targetAngle % 360;
        if (targetAngleErr > 180) targetAngleErr = targetAngleErr - 360;// 2-28-2024 - should prevent the flip around issue 
        else if (targetAngleErr < -180) targetAngleErr = targetAngleErr + 360;// 2-28-2024 - should prevent the flip around issue 

//        targetAngleErr =  SwerveSubsystem.getYaw().getDegrees() - targetAngle - 180;

//        xErr = driveXErr;
//        yErr = driveYErr;
        xErr = 0;
        yErr = 0;
        rotationErr = -targetAngleErr;
        //SmartDashboard.putNumber("aimXErr", aimXErr);
        //SmartDashboard.putNumber("aimYErr", aimYErr);
        //SmartDashboard.putNumber("desired target angle",desiredAngle);
        //SmartDashboard.putNumber("targetAngleErr", targetAngleErr);

        targetDistance = Math.pow(Math.pow(aimXErr,2)+ Math.pow(aimYErr,2), 0.5) + goalPosOffset;
        SmartDashboard.putNumber("targetDistance", targetDistance);

        if (targetDistance < 3.5 && targetAngleErr < .8) shotReadyCycles++;
        else if (targetDistance > 5.5) {
            shotReadyCycles++;}
        else if (targetDistance > 4) {
            shotReadyCycles = 0;}
        else if (targetAngleErr < .5) {
            shotReadyCycles++;}
        else shotReadyCycles = 0;
        
        if (shotReadyCycles > 6) shotReady = true;
        else shotReady = false;

    }


    public static void automaticPodiumShoot(String AUTOorTELE, String REDorBLUE){
        double targetAngle; double targetAngleErr; 

        if (REDorBLUE == null) {
            REDorBLUE = "BLUE";
        }
        
        if (REDorBLUE.equals("BLUE")){//target is blue speaker
            targetAngle = 0;//
        }
        else if (REDorBLUE.equals("RED")){//target is red speaker
            targetAngle = 180;//
        }
        else {//
            rotationErr = 0; //
            return;
        }

        if (REDorBLUE == "BLUE") desiredAngle =  targetAngle + shootAngleOffset - 30;
        if (REDorBLUE == "RED") desiredAngle = targetAngle + shootAngleOffset + 30;
//        targetAngleErr =  (desiredAngle + SwerveSubsystem.getPoseSwerveOdometry().getRotation().getDegrees()) % 360;
        targetAngleErr =  (desiredAngle + SwerveSubsystem.getPoseFromPoseEstimator().getRotation().getDegrees()) % 360;

        //changed reference to pose estimator for rotation instead of odometry
//        targetAngleErr = SwerveSubsystem.getPoseSwerveOdometry().getRotation().getDegrees() - (Math.atan(aimYErr / aimXErr)*180 / Math.PI) - targetAngle % 360;
        if (targetAngleErr > 180) targetAngleErr = targetAngleErr - 360;// 2-28-2024 - should prevent the flip around issue 
        else if (targetAngleErr < -180) targetAngleErr = targetAngleErr + 360;// 2-28-2024 - should prevent the flip around issue 

//        targetAngleErr =  SwerveSubsystem.getYaw().getDegrees() - targetAngle - 180;

//        xErr = driveXErr;
//        yErr = driveYErr;
        rotationErr = -targetAngleErr;
        //SmartDashboard.putNumber("aimXErr", aimXErr);
        //SmartDashboard.putNumber("aimYErr", aimYErr);
        //SmartDashboard.putNumber("desired target angle",desiredAngle);
        //SmartDashboard.putNumber("targetAngleErr", targetAngleErr);

        SmartDashboard.putNumber("targetDistance", targetDistance);

        if (targetAngleErr < .5) shotReady = true;
        else shotReady = false;


    }

     public static void climbAlign(String REDorBLUE, String LEFT_MID_RIGHT){
        double driveTargetX = 0 ; double driveXErr; 
        double driveTargetY = 0 ; double driveYErr; 
        double targetAngle = 0; double targetAngleErr;
        if (REDorBLUE.equals("BLUE")){//target is blue climb
            if(LEFT_MID_RIGHT.equals("LEFT")){
                driveTargetX = 4.64;
                driveTargetY = 3.71;
                targetAngle = 240;}
            else if (LEFT_MID_RIGHT.equals("MID")){
                driveTargetX = 5.3;
                driveTargetY = 4.1;
                targetAngle = 180;}
            else if (LEFT_MID_RIGHT.equals("RIGHT")){
                driveTargetX = 4.64;
                driveTargetY = 4.495;
                targetAngle = 120;}
        }
        else if (REDorBLUE.equals("RED")){//target is red speaker
            if(LEFT_MID_RIGHT.equals("LEFT")){
                driveTargetX = 4.64;
                driveTargetY = 4.495;
                targetAngle = 120;}
            else if (LEFT_MID_RIGHT.equals("MID")){
                driveTargetX = 5.3;
                driveTargetY = 4.1;
                targetAngle = 0;}
            else if (LEFT_MID_RIGHT.equals("RIGHT")){
                driveTargetX = 4.64;
                driveTargetY = 3.71;
                targetAngle = 240;}        }
        else {//no target 2-28-2024
            xErr = 0;
            yErr = 0;
            rotationErr = 0; //2-28 need to add this
            return;
        }
        
        driveXErr = SwerveSubsystem.getPoseSwerveOdometry().getX() - driveTargetX;
        driveYErr = SwerveSubsystem.getPoseSwerveOdometry().getY() - driveTargetY;
        targetAngleErr = SwerveSubsystem.getPoseFromPoseEstimator().getRotation().getDegrees() - targetAngle % 360;
        
        if (targetAngleErr > 180) targetAngleErr = targetAngleErr - 360;// 2-28-2024 - should prevent the flip around issue 
        else if (targetAngleErr < -180) targetAngleErr = targetAngleErr + 360;// 2-28-2024 - should prevent the flip around issue 
        
        xErr = driveXErr;
        yErr = driveYErr;
        rotationErr = targetAngleErr;
    }

}