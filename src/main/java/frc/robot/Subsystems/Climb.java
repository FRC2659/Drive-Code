package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.SoftLimitDirection;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Climb extends SubsystemBase{

    private static final CANSparkMax mLeftClimb = new CANSparkMax(Constants.CLIMB_LEFT, MotorType.kBrushless);
    private static final CANSparkMax mRightClimb = new CANSparkMax(Constants.CLIMB_RIGHT, MotorType.kBrushless);
    private static final SparkPIDController mLeftClimbPID = mLeftClimb.getPIDController();
    private static final RelativeEncoder mLeftClimbEncoder = mLeftClimb.getEncoder();

    private static final SparkPIDController mRightClimbPID = mRightClimb.getPIDController();
    private static final RelativeEncoder mRightClimbEncoder = mRightClimb.getEncoder();
   
    private static double leftClimbTarget = 0;
    private static double leftClimbExtend = 85;
    private static double leftClimbLevelOffset = 0;

    private static double rightClimbTarget = 0;
    private static double rightClimbExtend = 85;
    private static double rightClimbLevelOffset = 0;

    private static boolean homing = false;

    //Motion Limits
/*    private static final float lowerLimBase = 0;
    private static final float upperLimBase = -20; //Set once bot is alive
    private static final float lowerLimTop = 0;
    private static final float upperLimTop = 20; //Set once bot is alive
*/
    //Current Limits
    private static final int currentLim = 60; //Climbing Current Limit
    private static final int hookSetCurrent = 15; //Climbing Current Limit
    
    
    //PID Values
    private static final double kP = 0.6;
    private static final double kI = 0.05;
    private static final double kIDrain = 0.05;
    private static final double kD = 2.0;
    private static final double maxOutput = 0.8;
    private static final double minOutput = -0.8;


    private static boolean shooting;
    private static boolean intaking;

    public Climb() {
        mLeftClimb.restoreFactoryDefaults();
        mRightClimb.restoreFactoryDefaults();
                
        //Sets Motion and Current Limits
        //mLeftClimb.setSoftLimit(SoftLimitDirection.kForward, upperLimBase);
        //mRightClimb.setSoftLimit(SoftLimitDirection.kForward, upperLimBase);
        //mLeftClimb.setSoftLimit(SoftLimitDirection.kReverse, lowerLimBase);
        //mRightClimb.setSoftLimit(SoftLimitDirection.kReverse, lowerLimBase);

        mLeftClimb.setSmartCurrentLimit(currentLim);
        mRightClimb.setSmartCurrentLimit(currentLim);

        mLeftClimbPID.setOutputRange(minOutput, maxOutput);
        mRightClimbPID.setOutputRange(minOutput, maxOutput);

        //Sets PID Vals
        mLeftClimbPID.setP(kP);
        mLeftClimbPID.setI(kI);
        mLeftClimbPID.setIZone(kIDrain);
        mLeftClimbPID.setD(kD);
        mLeftClimb.setInverted(true);

        mRightClimbPID.setP(kP);
        mRightClimbPID.setI(kI);
        mRightClimbPID.setIZone(kIDrain);
        mRightClimbPID.setD(kD);
        mRightClimb.setInverted(false);
        mLeftClimb.burnFlash();
        Timer.delay(.20);

        mRightClimb.burnFlash();
        Timer.delay(.20);
    }

    @Override
    public void periodic() {

        updateSD();
        if (!homing){
        mLeftClimbPID.setReference(leftClimbTarget, ControlType.kPosition); 
        mRightClimbPID.setReference(rightClimbTarget, ControlType.kPosition); 
        }
        else{
            homeClimbers();
        }
    }

    public static void homeClimbers(){
        homing = true;
        mLeftClimbPID.setReference(-0.3, ControlType.kDutyCycle);
        mRightClimbPID.setReference(-0.3, ControlType.kDutyCycle);
        leftClimbTarget=0;
        rightClimbTarget=0;
        if (mLeftClimb.getOutputCurrent() > 35 ) mLeftClimbEncoder.setPosition(-.01);
        if (mRightClimb.getOutputCurrent() > 35 ) mRightClimbEncoder.setPosition(-.01);

        if (mLeftClimb.getOutputCurrent() > 35 && mRightClimb.getOutputCurrent() > 35){
            homing = false;
        } 
    }

    public static void stopHoming(){
        homing = false;
//        mLeftClimbEncoder.setPosition(0);
//        mRightClimbEncoder.setPosition(0);
    }

    public void startClimb(){
        homing = false;
        leftClimbTarget = leftClimbExtend;
        rightClimbTarget = rightClimbExtend;
       }
       

    public void setHooks(){
        mLeftClimb.setSmartCurrentLimit(hookSetCurrent);
        mRightClimb.setSmartCurrentLimit(hookSetCurrent);
        leftClimbTarget = 0;
        rightClimbTarget = 0;
        leftClimbLevelOffset = mLeftClimbEncoder.getPosition();
        rightClimbLevelOffset = mRightClimbEncoder.getPosition();
    }

    public void climb(){
        mLeftClimb.setSmartCurrentLimit(currentLim);
        mRightClimb.setSmartCurrentLimit(currentLim);
        if(leftClimbLevelOffset > rightClimbLevelOffset){
            rightClimbTarget = 0;
            leftClimbTarget = 0;
        //leftClimbTarget = leftClimbLevelOffset - rightClimbLevelOffset+5;
        }
        else if(leftClimbLevelOffset < rightClimbLevelOffset){
            //leftClimbTarget = .2;
            leftClimbTarget = 0;
            rightClimbTarget = 0;
            //rightClimbTarget = rightClimbLevelOffset - leftClimbLevelOffset;
        }
        else{
            leftClimbTarget = 0;
            rightClimbTarget = 0;
        }

    }

    public void TEST(){

        rightClimbTarget = SmartDashboard.getNumber("Right Climb TEST", 0);
        leftClimbTarget = SmartDashboard.getNumber("Left Climb TEST", 0);
        SmartDashboard.putNumber("Right Climb TEST",rightClimbTarget);
        SmartDashboard.putNumber("Left Climb TEST", leftClimbTarget);
       }
    
    private static void updateSD() {
        SmartDashboard.putNumber("Right Climb Current Position", mRightClimbEncoder.getPosition());
        SmartDashboard.putNumber("Left Climb Current Position", mRightClimbEncoder.getPosition());
    }
}
