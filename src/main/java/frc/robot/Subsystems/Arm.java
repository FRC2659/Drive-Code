package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.FaultID;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkBase.SoftLimitDirection;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Arm extends SubsystemBase{

    private static final CANSparkMax mBase1 = new CANSparkMax(Constants.BASE_ARM_1, MotorType.kBrushless);
    private static final CANSparkMax mBase2 = new CANSparkMax(Constants.BASE_ARM_2, MotorType.kBrushless);
    private static final SparkPIDController mBasePID = mBase1.getPIDController();
    private static final SparkPIDController mBasePID2 = mBase2.getPIDController();
    private static final RelativeEncoder mBaseEncoder = mBase1.getEncoder();
    private static final RelativeEncoder mBaseEncoder2 = mBase2.getEncoder();

    private static final CANSparkMax mTop1 = new CANSparkMax(Constants.TOP_ARM_1, MotorType.kBrushless);
    private static final CANSparkMax mTop2 = new CANSparkMax(Constants.TOP_ARM_2, MotorType.kBrushless);
    private static final SparkPIDController mTopPID = mTop1.getPIDController();
    private static final SparkPIDController mTopPID2 = mTop2.getPIDController();
    private static final RelativeEncoder mTopEncoder = mTop1.getEncoder();
    private static final RelativeEncoder mTopEncoder2 = mTop2.getEncoder();

    //Motion Limits
    private static final float lowerLimBase = 0;
    private static final float upperLimBase = -22; //Set once bot is alive
    private static final float lowerLimTop = 0;
    private static final float upperLimTop = 22; //Set once bot is alive

    //Current Limits
    private static final int currentLimBase = 39; //Set once bot is alive
    private static final int currentLimTop = 39; //Set once bot is alive
    
    //PID Values
    private static final double kPB = 2; //was .6
    private static final double kIB = 0.005; //was 0.04
    private static final double kIZoneB = 0.4;//0.05, .02 
    private static final double kDB = 16.0;//7
    private static final double maxOutputBase = 0.4;
    private static final double minOutputBase = -0.9;

    private static final double kPT = 0.5;//was 0.5
    private static final double kIT = 0.005;
    private static final double kIZoneT = 0.2;
    private static final double kDT = 15.00;//was 10
    private static final double maxOutputTop = 0.9;
    private static final double minOutputTop = -0.9;

    private static ArmStates mStates= new ArmStates();
    private static ArmState mCurrentState;
    private static ArmState mTargetState;
    private static boolean shooting;
    private static boolean intaking;
    public static boolean shotReady;
    public static double shotReadyCycles = 0;

    public Arm() {
                
        mBase1.clearFaults();
        mBase2.clearFaults();
        mTop1.clearFaults();
        mTop2.clearFaults();

        mBase1.restoreFactoryDefaults();
        mBase2.restoreFactoryDefaults();
        mTop1.restoreFactoryDefaults();
        mTop2.restoreFactoryDefaults();
        
        mCurrentState = mStates.Home;
        mTargetState = mStates.Home;
        
        //Syncro Arm motors
        //mBase2.follow(mBase1, true);
        //mTop2.follow(mTop1, true);
        
        //Sets Motion and Current Limits

        //Base 1
        
        //mBase1.setSoftLimit(SoftLimitDirection.kReverse, upperLimBase);
        //mBase1.setSoftLimit(SoftLimitDirection.kForward, lowerLimBase);
        mBase1.setSmartCurrentLimit(currentLimBase);
        mBase1.setIdleMode(IdleMode.kBrake);
        

        mBasePID.setP(kPB);
        mBasePID.setI(kIB);
        mBasePID.setIZone(kIZoneB);
        mBasePID.setD(kDB);
        mBasePID.setIMaxAccum(.2, 0);
        
        mBase1.burnFlash();
        Timer.delay(.1);
        mBase1.burnFlash();
        Timer.delay(.1);

        //Base 2
        
        mBase2.setSmartCurrentLimit(currentLimBase);
        mBasePID2.setOutputRange(-maxOutputBase, -minOutputBase);
        //mBase2.setSoftLimit(SoftLimitDirection.kForward, -upperLimBase);
        //mBase2.setSoftLimit(SoftLimitDirection.kReverse, lowerLimBase);
        mBase2.setIdleMode(IdleMode.kBrake);

        mBasePID2.setOutputRange(-maxOutputBase, -minOutputBase);        
        mBasePID2.setP(kPB);
        mBasePID2.setI(kIB);
        mBasePID2.setIZone(kIZoneB);
        mBasePID2.setD(kDB);
        mBasePID2.setIMaxAccum(.2, 0);

        
        mBase2.burnFlash();
        Timer.delay(.1);
        mBase2.burnFlash();
        Timer.delay(.1);

        //Top 1

        mTop1.setInverted(false);
//        mTop1.setSoftLimit(SoftLimitDirection.kForward, upperLimTop);
//        mTop1.setSoftLimit(SoftLimitDirection.kReverse, lowerLimTop);
        mTop1.setSmartCurrentLimit(currentLimTop);
        mTop1.setIdleMode(IdleMode.kBrake);
        mTopPID.setOutputRange(minOutputTop,maxOutputTop);


        mTopPID.setP(kPT);
        mTopPID.setI(kIT);
        mTopPID.setIZone(kIZoneT);
        mTopPID.setD(kDT);
        
        mTop1.burnFlash();
        Timer.delay(.1);
        mTop1.burnFlash();
        Timer.delay(.1);

        //Top 2

        mTop2.setSmartCurrentLimit(currentLimTop);
        mTop2.setInverted(false);
        mTop2.setIdleMode(IdleMode.kBrake);
        //mTop2.setSoftLimit(SoftLimitDirection.kForward, upperLimTop);
        //mTop2.setSoftLimit(SoftLimitDirection.kReverse, lowerLimTop);
        mTopPID2.setOutputRange(minOutputTop,maxOutputTop);

        mTopPID2.setP(kPT);
        mTopPID2.setI(kIT);
        mTopPID2.setIZone(kIZoneT);
        mTopPID2.setD(kDT);

        mTop2.burnFlash();
        Timer.delay(.1);
        mTop2.burnFlash();
        Timer.delay(.1);
    }

    @Override
    public void periodic() {
        updateSD();
        /*
        if (mTargetState != mCurrentState) {
            if (shooting) {
                if (mTargetState == mStates.Home) {
                    mCurrentState = mTargetState;
                    shooting = false;
                } else if (mTargetState == mStates.Intaking) {
                    if (-mBaseEncoder.getPosition() < 1.0 || mTopEncoder.getPosition() < 1.5) {
                        mCurrentState = mTargetState;
                        shooting = false;
                        intaking = true;
                    } else {
                        mCurrentState = mStates.Home;
                    }
                }
            } else if (intaking) {
                if (mTargetState == mStates.Home) {
                    mCurrentState = mTargetState;
                    intaking = false;
                } else if (shooting) {
                    if (mTopEncoder.getPosition() < 1.5 || -mBaseEncoder.getPosition() < 1.0) {
                        mCurrentState = mTargetState;
                        shooting = true;
                        intaking = false;
                    } else {
                        mCurrentState = mStates.Home;
                    }
                }
            } else if (mCurrentState == mStates.Home) {
                mCurrentState = mTargetState;
            }
        }
        */
        mCurrentState = mTargetState;

        mBasePID.setReference(mCurrentState.base , ControlType.kPosition); //LEAVE COMMENTED FOR INITIAL SEQUENCE TESTING
        mBasePID2.setReference(-mCurrentState.base, ControlType.kPosition);
        mTopPID.setReference(mCurrentState.top, ControlType.kPosition); //LEAVE COMMENTED FOR INITIAL SEQUENCE TESTING
        mTopPID2.setReference(-mCurrentState.top, ControlType.kPosition);

        checkShotReady();
    }

    private static double getYaw() {
        return SwerveSubsystem.gyro.getAngle();
    }

    private static void checkShotReady(){
        if(Math.abs( -mBaseEncoder2.getPosition() - mCurrentState.base) < .15 && Math.abs(mTopEncoder.getPosition() - mCurrentState.top) < .1 ){
            shotReadyCycles++;}
        else shotReadyCycles = 0;
        
        if (shotReadyCycles > 10) shotReady = true;
        else shotReady = false;

    }

    public void source() {
        mTargetState = mStates.Source;
    }

    public void intake() {
        mTargetState = mStates.Intaking;
    }

    public void subShot() {
        mTargetState = mStates.SubShotBack;
        shooting = true;
    }
    
    public void frontTall() {
        mTargetState = mStates.FrontTallFixedShot;
        shooting = true;
    }

    public void targetShot(double baseAngle, double EEAngle) {
        mTargetState = new ArmState(baseAngle, EEAngle);
        
        shooting = true;
    }

    public void goHome() {
        mTargetState = mStates.Home;
    }

    public void climb3() {
        mTargetState = mStates.climb3;
    }

    public void amp() {
        mTargetState = mStates.AmpScore;
        shooting = true;
    }
    public void setClimbState(double state){
        if (state == 1) mTargetState = mStates.climb1;
        if (state == 2) mTargetState = mStates.climb2;
        if (state == 3) mTargetState = mStates.climb3;
        if (state == 4) mTargetState = mStates.climb4;
        if (state == 5) mTargetState = mStates.climbVertTrap;
    }

    public void autoShot(double angle) {
        mTargetState = new ArmState(angle, 1);
    }

    public void TEST(){
        mStates.TEST(SmartDashboard.getNumber("TEST_Base_Angle", 0),SmartDashboard.getNumber("TEST_Endeffector_Angle",0.5));
        mTargetState = mStates.TEST;
    }
    
    private static void updateSD() {
        SmartDashboard.putNumber("Base Arm1 Current Position", mBaseEncoder.getPosition());
        SmartDashboard.putNumber("Base Arm2 Current Position", mBaseEncoder2.getPosition());
        SmartDashboard.putNumber("Top Arm1 Current Position", mTopEncoder.getPosition());
        SmartDashboard.putNumber("Top Arm2 Current Position", mTopEncoder2.getPosition());
        SmartDashboard.putNumber("Base Arm1 Output", mBase1.getAppliedOutput());
        SmartDashboard.putNumber("Base Arm2 Output", mBase2.getAppliedOutput());
        SmartDashboard.putNumber("Top Arm1 Output", mTop1.getAppliedOutput());
        SmartDashboard.putNumber("Top Arm2 Output", mTop2.getAppliedOutput());

        //SmartDashboard.putNumber("Base Arm Current State", mCurrentState.base);
        //SmartDashboard.putNumber("Top Arm Current State", mCurrentState.top);
    }
}
