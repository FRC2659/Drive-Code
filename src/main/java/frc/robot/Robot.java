// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.net.PortForwarder;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.AutoModeSelector.AutonMode;
import frc.robot.subsystems.EndEffector;
import frc.robot.subsystems.SuperStructure;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.Vision;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  public static CTREConfigs ctreConfigs;

  private Command m_autonomousCommand;

  private static final SuperStructure mSuperStructure = new SuperStructure();
  private RobotContainer m_robotContainer;
      double baseAngleTEST ;
    double EEAngleTEST ;
    double FW1TEST ;
    double FW2TEST ;
    private static boolean climbAlign = false;
    private static boolean operatorPOV = false;
    private static boolean targetShotBoolean = false;
    private static boolean targetShotBoolean2 = false;

//  private Vision vision;

  private final XboxController m_driveController = new XboxController(0);
    private final XboxController m_operatorController = new XboxController(1);
    //public static PowerDistribution m_pdh = new PowerDistribution(1, ModuleType.kRev);
    //public static SwerveSubsystem m_Swerve;

  @Override
  public void robotInit() {
    //SmartDashboard.putNumber("debugRobotInit", Timer.getFPGATimestamp());

    ctreConfigs = new CTREConfigs();
    // Instantiate our RobotContainer.  This will perform all our button bindings, and put our
    // autonomous chooser on the dashboard.
    m_robotContainer = new RobotContainer();

    PortForwarder.add(5800, "photonvision.local", 5800);

  }

  /**
   * This function is called every robot packet, no matter the mode. Use this for items like
   * diagnostics that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {

    RobotContainer.alliance = RobotContainer.mAlliance.getSelected().toString();
    SmartDashboard.putString("Alliance Color", RobotContainer.alliance);
    //SmartDashboard.putNumber("debugRobotPeriodic", Timer.getFPGATimestamp());
    CommandScheduler.getInstance().run();

    if (m_operatorController.getLeftY() > 0.8) {
      Vision.goalPos(0.005);
    } else if (m_operatorController.getLeftY() < -0.8) {
      Vision.goalPos(-0.005);
    }
  }

  /** This function is called once each time the robot enters Disabled mode. */
  @Override
  public void disabledInit() {

  }

  @Override
  public void disabledPeriodic() {
   /*
    if (AutoModeSelector.returnAutoMode() == 2) {
      SwerveSubsystem.gyroOffset(-28.6);//blue middle
    } else if (AutoModeSelector.returnAutoMode() == 3) {
      SwerveSubsystem.gyroOffset(180-28.6);//red middle
    } else if (AutoModeSelector.returnAutoMode() == 4){
      SwerveSubsystem.gyroOffset(180-60-28.6);
    } else if (AutoModeSelector.returnAutoMode() == 5){
      SwerveSubsystem.gyroOffset(60-28.6);
    }
*/
    SuperStructure.lightMode = 0;
  }

  /** This autonomous runs the autonomous command selected by your {@link RobotContainer} class. */
  @Override
  public void autonomousInit() {

    mSuperStructure.homeClimb();
    
    if (AutoModeSelector.returnAutoMode() == 2) {
      SwerveSubsystem.gyroOffset(0);//blue middle
    } else if (AutoModeSelector.returnAutoMode() == 3) {
      SwerveSubsystem.gyroOffset(180);//red middle
    } else if (AutoModeSelector.returnAutoMode() == 4){
      SwerveSubsystem.gyroOffset(60);
    } else if (AutoModeSelector.returnAutoMode() == 5){
      SwerveSubsystem.gyroOffset(180-60);
    } else if (AutoModeSelector.returnAutoMode() == 6){
      SwerveSubsystem.gyroOffset(-60);
    } else if (AutoModeSelector.returnAutoMode() == 7){ // red source 
      SwerveSubsystem.gyroOffset(180 + 60);
    }
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

//    RobotContainer.s_Swerve.resetOdometry(new Pose2d()); //die lol

    // schedule the autonomous command (example)
    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }

  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {
  SuperStructure.lightMode = 4;
   //m_intakeSolenoid.set(m_armSub.getSolAuto());

  }

   //SmartDashboard.putBoolean("autonomousSolenoid", ArmSubsystem.autonomousSolenoid);
  @Override
  public void teleopInit() {
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }

    SuperStructure.goHome();

  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
      if (m_driveController.getXButton()) {
        SwerveSubsystem.resetModulesToAbsolute();
      }

      if (m_driveController.getRightBumper()) mSuperStructure.targetShot(true, 0); //low back
      if (m_driveController.getRightBumperReleased()) mSuperStructure.targetShot(false, 0);//low back
      if (m_driveController.getLeftBumper()) mSuperStructure.targetShot(true, 1);//low forward
      if (m_driveController.getLeftBumperReleased()) mSuperStructure.targetShot(false, 1); //low forward
      if (m_driveController.getYButton()) mSuperStructure.targetShot(true, 4);//low forward
      if (m_driveController.getYButton()) mSuperStructure.targetShot(false, 4); //low forward
     
      if (m_driveController.getLeftTriggerAxis() > 0.5) {
        targetShotBoolean = true;
        mSuperStructure.targetShot(true, 3);} //High Back
      if (m_driveController.getLeftTriggerAxis() < 0.5 && targetShotBoolean) {
        targetShotBoolean = false;
        mSuperStructure.targetShot(false, 3);} //High Back Pause Autofire

      if (m_driveController.getRightTriggerAxis() > 0.5) {
        targetShotBoolean2 = true;
        mSuperStructure.targetShot(true, 2);} //High Back Pause Autofire
      if (m_driveController.getRightTriggerAxis() < 0.5 && targetShotBoolean2) {
        targetShotBoolean2 = false;
        mSuperStructure.targetShot(false, 2);} //High Back Pause Autofire

      if (m_driveController.getAButton()) mSuperStructure.goHome();

      if(m_driveController.getPOV() == 270) {
        
        SuperStructure.climbAlign("LEFT");
        climbAlign = true;}
      else if(m_driveController.getPOV() == 0) {
        SuperStructure.climbAlign("MID");
        climbAlign = true;}
      else if(m_driveController.getPOV() == 90) {
        SuperStructure.climbAlign("RIGHT");
        climbAlign = true;}
      else if(climbAlign){
        mSuperStructure.stopClimbAlign();
        climbAlign = false;}

      if (m_operatorController.getXButton()) {
        SuperStructure.goHome();
      } else if (m_operatorController.getLeftBumper()) {
        mSuperStructure.source();
      } else if (m_operatorController.getRightBumper()) {
        mSuperStructure.intake();
      } else if (m_operatorController.getYButton()) {
        SuperStructure.subShot();
      } else if (m_operatorController.getAButton()) {
        mSuperStructure.ampScore();
      } else if (m_operatorController.getRightTriggerAxis() > 0.5) {
        mSuperStructure.releaseShot();
      } else if (m_operatorController.getLeftTriggerAxis() > 0.5) {
        mSuperStructure.targetShot(true,0);
      }


      if (m_operatorController.getBButton()) {
        SuperStructure.settleNote();
      } else if (m_operatorController.getBButtonReleased() && SuperStructure.climbState >= 2) {
        EndEffector.setWheelSpeedClimb(3,30);
      } else if (m_operatorController.getBButtonReleased()&& SuperStructure.climbState < 2) {
        SuperStructure.goHome();
      }


      if(!operatorPOV && m_operatorController.getPOV() == 90) {
        mSuperStructure.climbSequence(1);
        operatorPOV = true;}
      else if(!operatorPOV && m_operatorController.getPOV() == 180) {
        mSuperStructure.climbReset();
        operatorPOV = true;}
      else if(!operatorPOV && m_operatorController.getPOV() == 270) {
        mSuperStructure.climbSequence(-1);
        operatorPOV = true;}
      else if(!operatorPOV && m_operatorController.getPOV() == 0) { //THIS ONE IS NOT CLIMB******************
        mSuperStructure.frontTallShot();                            //THIS ONE IS NOT CLIMB******************
        operatorPOV = true;}                                        //THIS ONE IS NOT CLIMB******************
      else if(m_operatorController.getPOV()==-1){
        operatorPOV = false;}
      
      
      if (m_operatorController.getStartButtonPressed()){
      mSuperStructure.homeClimb();        
      }
   
  }

  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
    mSuperStructure.homeClimb();


    SmartDashboard.putNumber("TEST_Base_Angle",baseAngleTEST);
    SmartDashboard.putNumber("TEST_Endeffector_Angle",EEAngleTEST);
    SmartDashboard.putNumber("TEST_LowerFlywheelSpeed",FW1TEST);
    SmartDashboard.putNumber("TEST_UpperFlywheelSpeed",FW2TEST);
    SmartDashboard.putNumber("TEST_ShootIntake",0);
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {
  mSuperStructure.TEST();
  
  }
}