package frc.robot;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class AutoModeSelector {

    enum AutonMode {
        JUST_SCORE, BLUE_4_PIECE, RED_4_PIECE, RED_AMP, BLUE_AMP , BLUE_SOURCE , RED_SOURCE, RED_MID_5, BLUE_MID_5
    }

    private AutonMode mCachedAutoMode = null;

 //    private SendableChooser<DesiredMode> mModeChooser;
    private static SendableChooser<AutonMode> mAutoMode;
 //   private AutonMode autoModeReturn = null;
    private static String autoChoiceReturn;

    public AutoModeSelector() {
        mAutoMode = new SendableChooser<>();
        mAutoMode.setDefaultOption("JUST SCORE", AutonMode.JUST_SCORE); //ID: 1
        mAutoMode.addOption("CENTER BLUE", AutonMode.BLUE_4_PIECE); //ID: 2
        mAutoMode.addOption("CENTER RED", AutonMode.RED_4_PIECE); //ID: 3
        mAutoMode.addOption("AMP SIDE BLUE", AutonMode.BLUE_AMP); //ID: 4
        mAutoMode.addOption("AMP SIDE RED", AutonMode.RED_AMP); //ID: 5
        mAutoMode.addOption("SOURCE SIDE BLUE", AutonMode.BLUE_SOURCE); //ID: 6
        mAutoMode.addOption("SOURCE SIDE RED", AutonMode.RED_SOURCE); //ID: 7
        mAutoMode.addOption("BLUE 5", AutonMode.BLUE_MID_5); //ID: 8
        mAutoMode.addOption("RED 5", AutonMode.RED_MID_5); //ID: 9
        SmartDashboard.putData("Auto Mode", mAutoMode);
    }

    public void updateModeCreator() {
        AutonMode desiredMode = mAutoMode.getSelected();
         if (mCachedAutoMode != desiredMode ) {
            System.out.println("Auto selection changed, updating creator: desiredMode->" + desiredMode.name());
          //  autoModeReturn = desiredMode;
        }
        mCachedAutoMode = desiredMode;
    }

    public static int returnAutoMode(){
       autoChoiceReturn = mAutoMode.getSelected().toString();
        SmartDashboard.putString(autoChoiceReturn, "autoChoiceReturn");
       switch (autoChoiceReturn) {
            case "JUST_SCORE": 
            return 1;

            case "BLUE_4_PIECE":
            return 2;

            case "RED_4_PIECE":
            return 3;

            case "BLUE_AMP":
            return 4;

            case "RED_AMP":
            return 5;

            case "BLUE_SOURCE":
            return 6;

            case "RED_SOURCE":
            return 7;

            case "RED_MID_5":
            return 9;

            case "BLUE_MID_5":
            return 8;
       }

       return 0;
    }

    public void outputToSmartDashboard() {
       // SmartDashboard.putString("AutoModeSelected", mCachedDesiredMode.name());
        SmartDashboard.putString("Auto Mode Selected", mCachedAutoMode.name());
    }

    public void reset() {
        mCachedAutoMode = null;
    }
}