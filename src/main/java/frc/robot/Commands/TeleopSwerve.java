package frc.robot.commands;

import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.Vision;
import frc.robot.subsystems.SuperStructure;
import frc.robot.subsystems.SwerveSubsystem;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;


public class TeleopSwerve extends Command {    
    private SwerveSubsystem s_Swerve;    
    private DoubleSupplier translationSup;
    private DoubleSupplier strafeSup;
    private DoubleSupplier rotationSup;
    private static PIDController mSpin;
    private static PIDController mX;
    private static PIDController mY;

    public TeleopSwerve(SwerveSubsystem s_Swerve, DoubleSupplier translationSup, DoubleSupplier strafeSup, DoubleSupplier rotationSup, BooleanSupplier robotCentricSup) {
        this.s_Swerve = s_Swerve;
        addRequirements(s_Swerve);

        this.translationSup = translationSup;
        this.strafeSup = strafeSup;
        this.rotationSup = rotationSup;

        //mSpin = new PIDController(.035, 0.005, 0.015 );
        mSpin = new PIDController(.06, 0.005, 0.005 ); //kd from .005 to .01 after Q4
//        mSpin = new PIDController(.08, 0.001, 0.01); // AZ pid numbers
        mSpin.setIZone(.5);
        mSpin.setSetpoint(0.0);
        mSpin.setTolerance(.2); //was .5
        
        mX = new PIDController(.1, 0, 0.01);
        mX.setSetpoint(0.0);
        mX.setTolerance(1.25);

        mY = new PIDController(.1, 0, 0.01);
        mY.setSetpoint(0.0);
        mY.setTolerance(1.25);
    }

    @Override
    public void execute() {
        /* Get Values, Deadband*/
        double translationVal = MathUtil.applyDeadband(translationSup.getAsDouble(), Constants.stickDeadband);
        double strafeVal = MathUtil.applyDeadband(strafeSup.getAsDouble(), Constants.stickDeadband);
        double rotationVal = MathUtil.applyDeadband(rotationSup.getAsDouble(), Constants.stickDeadband);
        double rotatAdjust = 0.0;
        double xAdjust = 0.0;
        double yAdjust = 0.0;
        
        if (SuperStructure.targetShot) {
            Vision.automaticShoot("TELE", RobotContainer.alliance);
            //rotatAdjust = Vision.rotationErr;
            rotatAdjust = mSpin.calculate(Vision.rotationErr);
            xAdjust = mX.calculate(Vision.xErr);
            yAdjust = -mY.calculate(Vision.yErr);

            SmartDashboard.putNumber("rotatAdjust",rotatAdjust);
            if (mSpin.atSetpoint()) {
                rotatAdjust = 0;
            } 
            if (mX.atSetpoint()) {
                xAdjust = 0;
            } 
            if (mY.atSetpoint()) {
                yAdjust = 0;
            } 
        }
        else{
            rotatAdjust = 0;
            xAdjust = 0;
            yAdjust = 0;
        }

        if (Math.abs(rotatAdjust) > .75) {
            rotatAdjust = .75*(rotatAdjust/Math.abs(rotatAdjust));
        }
        if (Math.abs(xAdjust) > .75) {
            xAdjust = .75*(xAdjust/Math.abs(xAdjust));
        }
        if (Math.abs(yAdjust) > .75) {
            yAdjust = .75*(yAdjust/Math.abs(yAdjust));
        }
        //if (Math.abs(rotatAdjust) < 0.02) rotatAdjust = 0;
        /* Drive */
        
        if (RobotContainer.alliance == "RED") {
            translationVal = -translationVal;
            strafeVal = -strafeVal;
        }

        s_Swerve.drive(
            new Translation2d(translationVal + xAdjust, strafeVal + yAdjust).times(Constants.Swerve.maxSpeed), 
            (rotationVal * Constants.Swerve.maxAngularVelocity) + (rotatAdjust * Constants.Swerve.maxAngularVelocity * .35), 
            true, 
            true
        );

//        SmartDashboard.putNumber("teleopSwerve1", Timer.getFPGATimestamp());
    }
}
