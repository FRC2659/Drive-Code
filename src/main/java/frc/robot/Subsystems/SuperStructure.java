package frc.robot.subsystems;

import javax.lang.model.util.ElementScanner14;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.AutoModes;
import frc.robot.RobotContainer;
import frc.robot.Vision;
import frc.robot.commands.TeleopSwerve;

public class SuperStructure extends SubsystemBase{
    
    private static boolean scoring;
    public static boolean targetShot;
    private static double deployTime;
    private static boolean released = false;

    private static final Arm mArm = new Arm();
    private static final EndEffector mEndEffector = new EndEffector();
    private static final Vision vision = new Vision();
    private static final Climb climb = new Climb();
    private static final LEDs LED = new LEDs();
    public static double climbState = 0;
    private static boolean intakeDelay = false; //time delay intake retracts when true and in auto
    public static int lightMode = 0;//0-disabled, 1-idle, 2-intaking, 3-scoring, 4-auto, 5-grabbed, 6-released

    public SuperStructure() {
        SmartDashboard.putNumber("climbState", climbState);
    }

    public static void intakeDelayDeploy(double time, boolean periodic) {//time delay intake retracts when true and in auto
        /*if(!periodic){
            deployTime = time + Timer.getFPGATimestamp();
            intakeDelay = true;}
        if(Timer.getFPGATimestamp() > deployTime)intakeDelay = true;
        else intakeDelay = false;*/
        intake();
    }

    public static void intake() {
        mEndEffector.intake();
        mArm.intake();
        lightMode = 2;
    }

    public void source() {
        mEndEffector.intake();
        mArm.source();
        lightMode =  2;
    }
    
    public static void subShot() {
        mEndEffector.subShot();
        mArm.subShot();
        scoring = true;
        lightMode = 3;
    }
    
    public static void frontTallShot() {
        mEndEffector.subShot();
        mArm.frontTall();
        scoring = true;
        lightMode = 3;
    }

    public static void settleNote() {
        EndEffector.setWheelPercentOutput(-1);
    }

    public static void autoShot(double sped, double angle) {
        mArm.autoShot(angle);
        mEndEffector.autoShot(sped);
    }

    public static void targetShot(boolean autoFire, int zeroBackOneLow) {
        double[] shotParams ;
        if(zeroBackOneLow == 0) {shotParams = vision.getBackShotLowParameters();}
        else if(zeroBackOneLow == 1){shotParams = vision.getForwardShotLowParameters();}
        else if(zeroBackOneLow == 2){shotParams = vision.getBackShotHighParameters();}
        else if(zeroBackOneLow == 3){shotParams = vision.getForwardShotHighParameters();}
        else if(zeroBackOneLow == 4){shotParams = vision.getPodiumFixedDistance();}
        else{shotParams = vision.getBackShotLowParameters();}
        double baseArmAngle = shotParams[0];
        double EndeffectorAngle = shotParams[1];
        double lowerFlywheelSpeed  = shotParams[2];
        double upperFlywheelSpeed  = shotParams[3];
        double angleOffset  = shotParams[4];
        //SmartDashboard.putNumber("baseAngle", baseArmAngle);
        //SmartDashboard.putNumber("Endeffector", EndeffectorAngle);
        
        mArm.targetShot(baseArmAngle, EndeffectorAngle);
//        mEndEffector.targetShot(EndeffectorAngle);
        mEndEffector.setWheelSpeed(upperFlywheelSpeed,lowerFlywheelSpeed);

        //subShot();
        targetShot = true;
        if (autoFire && shotConditionsReady()) {
            releaseShot();
            lightMode = 6;
            released = true;
        } else if (!released){
            lightMode = 3;
        }
    }
    public static boolean shotConditionsReady(){
        SmartDashboard.putBoolean("Ready - Swerve Speeds",(SwerveSubsystem.getAvgSwerveSpeeds() < .05));
        SmartDashboard.putBoolean("Ready - Vision Angle", Vision.shotReady);
        SmartDashboard.putBoolean("Ready - Shooter RPM", EndEffector.shotReady);
        SmartDashboard.putBoolean("Ready - Arm angle", Arm.shotReady);
    
        if (SwerveSubsystem.getAvgSwerveSpeeds() < .1 && Vision.shotReady && EndEffector.shotReady && Arm.shotReady) return true;
        else return false;
    }

    public void homeClimb(){
        Climb.homeClimbers();
    }
    public static void climbStart(){
        climb.startClimb();//extends hooks
        climbState = 1;
    }

    public static void BringClimbersDown(){
        climb.climb();//retracts hooks
        mArm.setClimbState(1);
        climbState = 0;
    }

    public static void justBringClimbersDown(){
        climb.climb();//retracts hooks
        //mArm.setClimbState(1);
        climbState = 0;
    }
    
    public void climbReset(){
        climb.startClimb();//extends hooks
        mArm.setClimbState(1);
        climbState = 0;
    }

    public static void climbSequence(double index){
        climbState += index;
        
        if (climbState < -1) climbState = -1;
        if (climbState > 11) climbState = 11;
        SmartDashboard.putNumber("climbState", climbState);
        
        if (climbState == -1) climb.climb();
        if (climbState == 1) climbStart();
        if (climbState == 2) mArm.climb3();
        if (climbState == 3) mArm.setClimbState(1);
        if (climbState == 4) climb.setHooks();
        if (climbState == 5) climb.climb();
        if (climbState == 6) {
            mArm.setClimbState(2);
            EndEffector.setWheelSpeedClimb(3,30);//5,50
        }
        if (climbState == 7) {
            mArm.setClimbState(5);} //new trap angle ()
            
        if (climbState == 8){
            mEndEffector.release();
            //EndEffector.setWheelSpeedClimb(18,5);
        }
        if (climbState == 9){
            //EndEffector.setHolder(0);
            //EndEffector.setWheelSpeedClimb(18,5);
            EndEffector.setWheelSpeedClimb(-30,30);
        }
        if (climbState == 10){
            mArm.setClimbState(2);
        }
        if (climbState == 11){
            mArm.setClimbState(5);
        }
       /*COMMENTED EN ROUTE TO AZ 3-13-24 -- Steps 7+ are <50% trap and very difficult to operate
        if (climbState == 7) {mEndEffector.release();
            EndEffector.setWheelSpeedClimb(3,4.5);}        
        if (climbState == 8) {
            mArm.setClimbState(2);
            EndEffector.setHolder(0);
            EndEffector.setWheelPercentOutput(0);}
        if (climbState == 9) {
            mArm.setClimbState(4);
            EndEffector.setHolder(0);
            EndEffector.setWheelSpeedClimb(3,3);}*/
        
    }

    public static void climbAlign(String LEFT_MID_RIGHT){
        targetShot = true;
        Vision.climbAlign(RobotContainer.alliance, LEFT_MID_RIGHT);
    }

    public static void stopClimbAlign(){
        targetShot = false;
    }

    public static void releaseShot() {
        //if (scoring) { //commented to test shooting 2-28-2024 BVN
            mEndEffector.release();
            //targetShot = false;
        //}
    }

    public static void goHome() {

        lightMode = 1;
        mArm.goHome();
        mEndEffector.goHome();
        targetShot = false;
        Climb.stopHoming();
        climbState = -1;
        climbSequence(0);
        //justBringClimbersDown();
        released = false;
    }

    public void ampScore() {
        mArm.amp();
        mEndEffector.amp();
        scoring = true;
        lightMode = 3;
    }

    public void TEST(){
        mArm.TEST();
        mEndEffector.TEST();
        climb.TEST();
    }

    @Override
    public void periodic() {
 //       intakeDelayDeploy(0,true);

//        if (DriverStation.isAutonomous() && intakeDelay) intake();

        if (DriverStation.isAutonomousEnabled() && AutoModes.target) {
            targetShot(true,0);
        }
    }
}
