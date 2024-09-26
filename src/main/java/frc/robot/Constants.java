package frc.robot;

import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.revrobotics.CANSparkBase.IdleMode;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import frc.lib.util.COTSFalconSwerveConstants;
import frc.lib.util.SwerveModuleConstants;
import edu.wpi.first.math.Matrix;
import java.util.Arrays;

/** Add your docs here. */
public class Constants {

    //Conversion Factors
    public static final double talonConversionUnits = ((1/((20.0 / 46.0) * (30.0 / 18.0) * (15.0 / 60.0))));

    public static final double kTurningP = 0.1;
    public static final double kTurningI = 0.0;
    public static final double kTurningD = 0.0;
    public static final double kTurningFF = 0.0;

    // Driving PID constants
    public static final double kDrivingP = 0.05;
    public static final double kDrivingI = 0.4;
    public static final double kDrivingD = 0.4;

    // Driving Encoder Conversion Factors
    public static final double kTurningEncoderPositionFactor = 0.1;  // Example value
    public static final double kTurningEncoderVelocityFactor = 0.1;  // Example value
    public static final double kDrivingEncoderPositionFactor = 0.1;  // Example value
    public static final double kDrivingEncoderVelocityFactor = 0.1;  // Example value
    public static final double stickDeadband = 0.1;

    public static final class Swerve {
        public static final int pigeonID = 13;
        public static final boolean invertGyro = false; // Always ensure Gyro is CCW+ CW-

        public static final COTSFalconSwerveConstants chosenModule =  //TODO: This must be tuned to specific robot
            COTSFalconSwerveConstants.SKETTYNOODUL(1/((20.0 / 46.0) * (30.0 / 18.0) * (15.0 / 60.0)));

        /* Drivetrain Constants */
        public static final double trackWidth = Units.inchesToMeters(24.75); //TODO: This must be tuned to specific robot
        public static final double wheelBase = Units.inchesToMeters(24.75); //TODO: This must be tuned to specific robot
        public static final double wheelCircumference = chosenModule.wheelCircumference;

        /* Swerve Kinematics 
         * No need to ever change this unless you are not doing a traditional rectangular/square 4 module swerve */
         public static final SwerveDriveKinematics swerveKinematics = new SwerveDriveKinematics(
            new Translation2d(wheelBase / 2.0, trackWidth / 2.0),
            new Translation2d(wheelBase / 2.0, -trackWidth / 2.0),
            new Translation2d(-wheelBase / 2.0, trackWidth / 2.0),
            new Translation2d(-wheelBase / 2.0, -trackWidth / 2.0));

        /* Module Gear Ratios */
        public static final double driveGearRatio = chosenModule.driveGearRatio;
        public static final double angleGearRatio = chosenModule.angleGearRatio;

        /* Motor Inverts */
        public static final boolean angleMotorInvert = true;
        public static final boolean driveMotorInvert = chosenModule.driveMotorInvert;

        /* Angle Encoder Invert */
        public static final SensorDirectionValue canCoderInvert = SensorDirectionValue.Clockwise_Positive;

        /* Swerve Current Limiting */
        public static final int angleContinuousCurrentLimit = 25;
        public static final int anglePeakCurrentLimit = 40;
        public static final double anglePeakCurrentDuration = 0.1;
        public static final boolean angleEnableCurrentLimit = true;

        public static final int driveContinuousCurrentLimit = 35;
        public static final int drivePeakCurrentLimit = 60;
        public static final double drivePeakCurrentDuration = 0.1;
        public static final boolean driveEnableCurrentLimit = true;

        /* These values are used by the drive falcon to ramp in open loop and closed loop driving.
         * We found a small open loop ramp (0.25) helps with tread wear, tipping, etc */
        public static final double openLoopRamp = 0.25;
        public static final double closedLoopRamp = 0.0;

        /* Angle Motor PID Values */
        public static final double angleKP = chosenModule.angleKP;
        public static final double angleKI = chosenModule.angleKI;
        public static final double angleKD = chosenModule.angleKD;
        public static final double angleKF = chosenModule.angleKF;

        /* Drive Motor Characterization Values 
         * Divide SYSID values by 12 to convert from volts to percent output for CTRE */
        public static final double driveKS = (0.32 / 12); //TODO: This must be tuned to specific robot
        public static final double driveKV = (1.51 / 12);
        public static final double driveKA = (0.27 / 12);

        /* Swerve Profiling Values */
        /** Meters per Second */
        public static final double maxSpeed = 4.5; //TODO: This must be tuned to specific robot
        /** Radians per Second */
        public static final double maxAngularVelocity = 10.0; //TODO: This must be tuned to specific robot

        /* Neutral Modes */
        public static final IdleMode angleNeutralMode = IdleMode.kCoast;
        public static final NeutralModeValue driveNeutralMode = NeutralModeValue.Brake;

        /* Module Specific Constants */
        /* Front Left Module - Module 0 */
        public static final class Mod0 { //TODO: This must be tuned to specific robot
            public static final int driveMotorID = 1;
            public static final int angleMotorID = 2;
            public static final int canCoderID = 3;
            public static final Rotation2d angleOffset = Rotation2d.fromDegrees(66.973);
            public static final SwerveModuleConstants constants = 
                new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
        }//66.797
        
        /* Front Right Module - Module 1 */
        public static final class Mod1 { //TODO: This must be tuned to specific robot
            public static final int driveMotorID = 4;
            public static final int angleMotorID = 5;
            public static final int canCoderID = 6;
            public static final Rotation2d angleOffset = Rotation2d.fromDegrees(256.289);
            public static final SwerveModuleConstants constants = 
                new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
        }//16.699
        
        /* Back Left Module - Module 2 */
        public static final class Mod2 { //TODO: This must be tuned to specific robot
            public static final int driveMotorID = 7;
            public static final int angleMotorID = 8;
            public static final int canCoderID = 9;
            public static final Rotation2d angleOffset = Rotation2d.fromDegrees(196.523);
            public static final SwerveModuleConstants constants = 
                new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
        }//78.838

        /* Back Right Module - Module 3 */
        public static final class Mod3 { //TODO: This must be tuned to specific robot
            public static final int driveMotorID = 10;
            public static final int angleMotorID = 11;
            public static final int canCoderID = 12;
            public static final Rotation2d angleOffset = Rotation2d.fromDegrees(211.992);
            public static final SwerveModuleConstants constants = 
                new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
        }

        public static Object resetOdometry(Pose2d initialPose) {
            return null;
        }
    }


    public static class Vision {
        //public static final String kCameraName = "photonvision";
        // Cam mounted facing forward, half a meter forward of center, half a meter up from center.
        public static final Transform3d kRobotToCam =
            new Transform3d(new Translation3d(-0.311, -0.111, .4064), new Rotation3d(0, 20*Math.PI / 180, -180 * Math.PI / 180));

        public static final Transform3d kRobotToCamL =
            new Transform3d(new Translation3d(-0.311, 0.111, .4064), new Rotation3d(0, -20*Math.PI / 180, 180 * Math.PI / 180));
        
        public static final Transform3d kRobotToCamF = //NEED TO SET THIS AFTER CAMERA CAL
            new Transform3d(new Translation3d(0.2347-.215, 0.166, .1935), new Rotation3d(0, -25*Math.PI / 180, 0 * Math.PI / 180));
        // The layout of the AprilTags on the field
        public static final AprilTagFieldLayout kTagLayout =
                AprilTagFields.kDefaultField.loadAprilTagLayoutField();


        // The standard deviations of our vision estimated poses, which affect correction rate
        // (Fake values. Experiment and determine estimation noise on an actual robot.)
        public static final Matrix<N3, N1> kSingleTagStdDevs = VecBuilder.fill(2, 2, 4);
        public static final Matrix<N3, N1> kMultiTagStdDevs = VecBuilder.fill(0.5, 0.5, 1);
    }

    public static final int BASE_ARM_1 = 17;
    public static final int BASE_ARM_2 = 18;
    public static final int TOP_ARM_1 = 19;
    public static final int TOP_ARM_2 = 20;

    public static final int END_EFFECTOR_1 = 21;
    public static final int END_EFFECTOR_2 = 22;
    public static final int END_EFFECTOR_3 = 23;

    public static final int CLIMB_LEFT = 24;
    public static final int CLIMB_RIGHT = 25;

    public static final class AutoConstants { //TODO: The below constants are used in the example auto, and must be tuned to specific robot
        public static final double kMaxSpeedMetersPerSecond = 5.5; //5.5
        public static final double kMaxAccelerationMetersPerSecondSquared = 3;//4 for source side 
        public static final double kMaxAngularSpeedRadiansPerSecond = Math.PI;
        public static final double kMaxAngularSpeedRadiansPerSecondSquared = Math.PI;
    
        public static final double kPController = 2.5; 
        public static final double kDController = 3.0; 
        public static final double kPThetaController = 6; //was 6
        public static final double kDThetaController = .2;
    
        /* Constraint for the motion profilied robot angle controller */
        public static final TrapezoidProfile.Constraints kThetaControllerConstraints =
            new TrapezoidProfile.Constraints(
                kMaxAngularSpeedRadiansPerSecond, kMaxAngularSpeedRadiansPerSecondSquared);
    }

    public static final class shooterLookup {
//        public double[][] OTB_Low = new double[6][5];

        public static final double[][] backShotLow = {//dist (m), baseAngle (motor rotations), endeffectorAngle (motor rotations), lowerFlywheelSpeed(rpm), upperFlywheelSpeed (rpm)
            {0,      0   ,  3.8 + 1.3+.1,     48.0,       48.0,        0,  29.0},
            {.8,     0.0 ,  0.5 + 1.3+.1,     60.0,       60.0,        -2,  30.5},
            {1,      -.75,     1.3+.2+.1,   63+3+2,     63+3+2,        -2,   28.5},
            {1.5,   -2.6 + 1,     1.0+.2+.1+.1,  65.0+3+3+2,  65.0+3+3+2,    -2,  18}, //pmatch10 adjusted base -3 to -2.5 + more angle compensation
            {2 ,    -3.8 + 1,     1.0+.1+.1,     73.0+3+3,    73.0+3+3,      -2,  14}, //
            {2.5,    -5.0+ 1-.5,  1.0+.1,     75.0+3+3+1,  75.0+3+3+1,    -2,  13},
            {3,     -5.75+ 1-.5,  1.0+.1,     78.0+3+3+2,  78.0+3+3+2,    -2,  12},//podium shot
            {3.5,    -6.5+ 1-.5+.25,  1.0+.1-.2,     78.0+3+3+2,  78.0+3+3+2,    -2,  12},
            {4,      -6.9+ 1,     1.0+.1,     80.0+3+3,    80.0+3+3,      -2,  12},//good
            {5,      -7.0+ 1,     1.0+.1,     80.0+3+3,    80.0+3+3,      -2,  12},//good
            {5.1,    -3.0,     3.5,     78.0-3,        78.0-3,          -2, 12},
            {12,       -3,     2.75,     85.0-3,        85.0-3,          -2,  12},
            {12.1,       -3,     2.75,     60.0-3,        60.0-3,          -2,  12},
            {16,       -3,     2.75,     60.0-3,        60.0-3,          -2,  12}//cross-court feed -- SPEED WAS 110
          };
          /* 
           * 
          */

        public static final double[][] backShotHigh = {
          /*  {0, -13.9, 6,  50.0,    50.0,  20},
            {1, -13.9, 5.5,  50.0,    50.0,  20},
            {2, -13.9, 4.2,  75.0,    75.0,  17},
            {3, -13.9, 3.3,  90.0,    90.0,  15},
            {4, -13.9, 3,  100.0,  100.0,  12}, //intentionally infireable 
            {5, -13.9, 4,  110.0,  110.0,  12}*/
            {1,   -17.5,  7.5+.1,  70.0,    70.0, 0},
            {2,   -17.5,  6.7+.1,  70.0,    70.0, 0},
            {2.5, -17.5,  6.35+.1,  76.0,    76.0,  0},
            {3,   -17.5,  5.8+.1,  80.0,    80.0,  0},
            {4,   -17.5,  5.6+.1,  83.0,    83.0,  0},
            {4.1,   -17.5,  6.8,  60.0,    60.0,  0}  


          };
        public static final double[][] forwardShotLow = {
            {0,   0.0 ,  5.5 + 1.3-.1,     55.0,    55.0,  180+2},//, -25},
            {1,   0.0 ,  6.4 + 1.3-.2-.6,     68.0,    68.0,  180+2},//, -23},
            {2,   0.0 ,  7.5 + 1.3-.2,     82.0,    82.0,  180+2},// -15},
            {3,   0.0 ,  8.4 + 1.3-.1+.2,     87.0,    87.0,  180+2},// -13},
            {3.5,   0.0, 8.65 + 1.3+.1,     87.0,    87.0,  180+2},// -13},
            {4,   0.0 ,  8.8 + 1.3+.1,     88.0,    88.0,  180+2},// -11.5},
            {5,   0.0 ,  6.2 + 1.3+.1,    65.0,   65.0,  180+2},//, -11} //intentionally unshootable -- WAS 110 speed  
            {5.1,   0.0 , 9.0,          60,   60,  180},//adjusted from 8.6 to 8.8 for Q4, to 8.9 after Q4, to 9, 60 before
            {12,   0.0 ,  8.1,          74,   74,  180}, //adjusted from 78 speed to 76 for Q4, 8.0 to 8.1 after Q4, 74,74 for q24x
//            {12.1,   0.0 ,  7.5 ,    75,   75,  180}//, -11} Midfield feed w/ 8m offset-lands mid back wall
            {12.1,   0.0 ,  8 ,    75,   75,  180}//, -11} Midfield feed            
            //prechamps:
            //{5,   0.0 ,  6.2 + 1.3,    85.0,   85.0,  180},//, -11} //intentionally unshootable -- WAS 110 speed  
            //{5.1,   0.0 ,  6.2 + 1.3 -.5,    78.0-3+2,   78.0-3+2,  180},//, -11} //intentionally unshootable -- WAS 110 speed  
            //{12,   0.0 ,  6.2 + 1.3 -.5,    85.0-3-1+2,   85.0-3-1+2,  180},
            
        };
        public static final double[][] forwardShotHigh = {
            {  0,     -20,   18.8+.1,    60,  60,  180},// -10}, 
            {  1,     -20,   18.8+.1-.3,    75,  75,  185},// -10}, 
            {  2,     -20,   19.0+.1-.2,    75,  75,  185},// -8}, 
            {  3,     -20,   19.45+.1-.1,   78,  78,  185},// -8}, 
            {3.5,     -20,   19.55+.1-.1 ,  82,  82,  185},// -8}, 
            {4.0,     -20,   19.45+.1-.1 ,  82,  82,  185},//, -8} //unshootable - shouldn't reach 110 rev/sec
            {4.1,     -20,   18.8+.1-.3,    60,  60,  185} 
          };
        public static final double[][] tallPodiumBack = {

            {0, -13.9, 3.3 + 1.3,  90.0,    90.0, 0,  15},
            {20, -13.9, 3.3 + 1.3,  90.0,    90.0, 0,  15}
          };
          


    }
}

