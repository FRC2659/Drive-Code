package frc.robot.subsystems;

import com.ctre.phoenix6.configs.*;
//import com.ctre.phoenix6.configs.TalonFXConfiguration;
//import com.ctre.phoenix6.controls.DutyCycleOut;
//import com.ctre.phoenix6.configs.ClosedLoopRampsConfigs;
//import com.ctre.phoenix6.configs.OpenLoopRampsConfigs;

import com.ctre.phoenix6.controls.*;
//import com.ctre.phoenix6.controls.Follower;
//import com.ctre.phoenix6.controls.VelocityDutyCycle;
//import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;
import edu.wpi.first.units.Velocity;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.AutoModeSelector;
import frc.robot.AutoModes;
import frc.robot.Constants;
import frc.robot.RobotContainer;

public class EndEffector extends SubsystemBase{
    private static final TalonFX mMotor1 = new TalonFX(Constants.END_EFFECTOR_1);
    private static final TalonFX mMotor2 = new TalonFX(Constants.END_EFFECTOR_2);
    
    private static final TalonFX mHolder = new TalonFX(Constants.END_EFFECTOR_3);
    //private static final CANSparkMax mHolder = new CANSparkMax(Constants.END_EFFECTOR_3, MotorType.kBrushless);
    private static final DigitalInput mBeam = new DigitalInput(0);

    private static final double mSubShotSpeed = 80.0;
    private static final double mIntakeSpeed = -40.0;
    private static final double mAmpSpeed = 30.0;
    private static final double mHoldingPower = 0.05;
    private static final double deadband = 1.0;
    private static double mTargetSpeed;
    private static final double trapSpeed = 10;
    
    private static final double mSubShotPercentOutput = .6; //temporary
    private static final double mIntakePercentOutput = -0.75; //temporary
    private static final double mAmpPercentOutput = 0.4; //temporary

    private static boolean intaking;
    private static boolean shooting;
    public static boolean shotReady;

    private static double timeI = 0.0;
    private static double timeS = 0.0;
    private static boolean started = false;
    
    private final VelocityVoltage m_FlywheelVoltageVelocity = new VelocityVoltage(0, 0, false, 0, 0, false, false, false);

    public EndEffector() {
        mMotor1.getConfigurator().apply(new TalonFXConfiguration());
        mMotor2.getConfigurator().apply(new TalonFXConfiguration());
        mHolder.getConfigurator().apply(new TalonFXConfiguration());
        //mHolder.restoreFactoryDefaults();

        //mMotor2.setControl(new Follower(mMotor1.getDeviceID(), true));
        TalonFXConfiguration mConfig = new TalonFXConfiguration();
        mConfig.Slot0.kP = 0.7;//was 0.4
        mConfig.Slot0.kI = 0.1; //
        mConfig.Slot0.kV = 0.12; //was .06
        mConfig.Slot0.kD = 0.00001; //was .0001
        //mConfig.Slot0.
        mConfig.CurrentLimits.SupplyCurrentLimit = 39.0;//was 20

        mConfig.OpenLoopRamps.VoltageOpenLoopRampPeriod = .08;        
        mConfig.OpenLoopRamps.DutyCycleOpenLoopRampPeriod = .08;
        mConfig.ClosedLoopRamps.VoltageClosedLoopRampPeriod = .08;
        mConfig.ClosedLoopRamps.DutyCycleClosedLoopRampPeriod = .08;

        mMotor1.getConfigurator().apply(mConfig);
        mMotor1.setInverted(true);

        mMotor2.getConfigurator().apply(mConfig);
        mMotor2.setInverted(true);
    //    mMotor1.config.OpenLoopRampsConfigs()
    //    mMotor1.ClosedLoopRampsConfigs(0.05);
        mHolder.setInverted(false);
    }

    public void intake() {
        setWheelSpeed(mIntakeSpeed, mIntakeSpeed);
        //setWheelPercentOutput(mIntakePercentOutput);
        setHolder(0.1);
        intaking = true;
    }

    public void setWheelSpeed(double upperSpeed, double lowerSpeed) {
        //mTargetSpeed = rollers;
        mMotor1.setControl(new VelocityVoltage(upperSpeed));
        mMotor2.setControl(new VelocityVoltage(lowerSpeed));
//        mMotor1.setControl(new VelocityVoltage(0)); //temp 3-3-28
//        mMotor2.setControl(new VelocityVoltage(0)); //temp 3-3-28

        if( Math.abs(mMotor1.getRotorVelocity().getValueAsDouble() - upperSpeed ) < 1 && 
            Math.abs(mMotor2.getRotorVelocity().getValueAsDouble() - lowerSpeed ) < 1 ){
                shotReady = true;
        }
        else shotReady = false;
    }

    
    public static void setWheelSpeedClimb(double upperSpeed, double lowerSpeed) {
        //mTargetSpeed = rollers;
        mMotor1.setControl(new VelocityVoltage(upperSpeed));
        mMotor2.setControl(new VelocityVoltage(lowerSpeed));
//        mMotor1.setControl(new VelocityVoltage(0)); //temp 3-3-28
//        mMotor2.setControl(new VelocityVoltage(0)); //temp 3-3-28

    }


    public static void setWheelPercentOutput(double percentOut) {
        mMotor1.setControl(new DutyCycleOut(percentOut));
        mMotor2.setControl(new DutyCycleOut(percentOut));
    }

    public static void setHolder(double percentOut) {
        mHolder.setControl(new DutyCycleOut(percentOut));
    }

    public void release() {
        double actual = mMotor1.getVelocity().getValue();
        setHolder(-0.7);
        SuperStructure.lightMode = 6;
    }

    public void subShot() {
        shooting = true;
        setWheelSpeed(mSubShotSpeed, mSubShotSpeed);
      //  setWheelPercentOutput(mSubShotPercentOutput);
    }

    public void amp() {
        shooting = true;
//        setWheelSpeed(mAmpSpeed);
        setWheelPercentOutput(mAmpPercentOutput);
    }

    public void trap() {
        shooting = true;
        setWheelSpeed(.75* trapSpeed, trapSpeed);
        
    }

    public void autoShot(double sped) {
        setWheelSpeed(sped, sped);
    }

    //public void targetShot(double speed) {
    //    shooting = true;
    //    setWheelPercentOutput(speed);
    //}

    public void goHome() {
        setWheelPercentOutput(0.0);
        setHolder(mHoldingPower);
        shooting = false; //BVN 2-23-2024 - added this line
        intaking = false;
        if ((mHolder.getSupplyCurrent().getValueAsDouble() > 0.25 && mHolder.getRotorVelocity().getValueAsDouble() > 0) || (mHolder.getRotorVelocity().getValueAsDouble() == 0) && mHolder.getSupplyCurrent().getValueAsDouble() > 0.15) {
            SuperStructure.lightMode = 5;
        }
    }

    public void TEST(){
        //SmartDashboard.getNumber("TEST_LowerFlywheelSpeed",0);
        //SmartDashboard.getNumber("TEST_UpperFlywheelSpeed",0);
        if(SmartDashboard.getNumber("TEST_ShootIntake",0) == -1){ 
            shooting = false;
            intake();}
        else if(SmartDashboard.getNumber("TEST_ShootIntake",0) == 0) {
            intaking = false;
            setHolder(.05);
            setWheelSpeed((SmartDashboard.getNumber("TEST_LowerFlywheelSpeed",0)), (SmartDashboard.getNumber("TEST_UpperFlywheelSpeed",0)));
        }
        else if(SmartDashboard.getNumber("TEST_ShootIntake",0) == 1) release();        
        //setWheelSpeed(SmartDashboard.getNumber("TEST_UpperFlywheelSpeed",0),SmartDashboard.getNumber("TEST_LowerFlywheelSpeed",0));

        
    }

    @Override
    public void periodic() {
        if (intaking) {
            if (!started) {
                started = true;
                timeS = Timer.getFPGATimestamp();
                //SmartDashboard.putNumber("timeS", timeS);
            }

            timeI = Timer.getFPGATimestamp() - timeS;
            //SmartDashboard.putNumber("timeI", timeI);
            if ((/*!mBeam.get() ||*/ mHolder.getSupplyCurrent().getValueAsDouble()>3)&& !DriverStation.isAutonomous()) {
                setWheelPercentOutput(0.0);
                setHolder(mHoldingPower);
                SuperStructure.goHome();
                intaking = false;
                SuperStructure.lightMode = 5;
            } else if ((/*!mBeam.get()  ||*/ mHolder.getSupplyCurrent().getValueAsDouble() > 3) && DriverStation.isAutonomous()) {
                if (AutoModeSelector.returnAutoMode() == 3 || AutoModeSelector.returnAutoMode() == 2) {
                    setHolder(mHoldingPower);
                    intaking = false;
                    SuperStructure.subShot();
                } else if (AutoModeSelector.returnAutoMode() == 7 || AutoModeSelector.returnAutoMode() == 6) {
                    setHolder(mHoldingPower);
                    intaking = false;
                    //SuperStructure.autoShot(AutoModes.sped, AutoModes.angle);
                    SuperStructure.subShot();
                } else {
                    setHolder(mHoldingPower);
                    intaking = false;
                    SuperStructure.subShot();
                }
            }

            if (Math.abs(mMotor1.getVelocity().getValueAsDouble()) < 1) {
                //setWheelPercentOutput(-0.75);
                SmartDashboard.putBoolean("Unjam", true);
            }
        } else {
            started = false;
        }

        updateSD();
    }

    private static void updateSD() {
        SmartDashboard.putNumber("Flywheel 1 Speed", mMotor1.getRotorVelocity().getValue());
        SmartDashboard.putNumber("Flywheel 2 Speed", mMotor2.getRotorVelocity().getValue());
        //SmartDashboard.putNumber("Flywheel Target", mTargetSpeed/2);
        SmartDashboard.putBoolean("Has Gamepiece", !mBeam.get());
        SmartDashboard.putNumber("holderCurrent", mHolder.getSupplyCurrent().getValueAsDouble());
        SmartDashboard.putNumber("Motor Current", mMotor1.getStatorCurrent().getValue());
        SmartDashboard.putBoolean("started", started);
    }
}
