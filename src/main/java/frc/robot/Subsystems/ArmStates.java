package frc.robot.subsystems;

public class ArmStates {
    public ArmState Home;
    public ArmState Intaking;
    public ArmState AmpScore;
    public ArmState SubShotBack;
    public ArmState SubShotFront;
    public ArmState Source;
    public ArmState climb1;
    public ArmState climb2;

    public ArmState climb3;
    public ArmState climb4;
    public ArmState climbVertTrap;
    public ArmState TEST;
    public ArmState FrontTallFixedShot;

    public ArmStates() {
        Home = new ArmState(0.0, 0.5);
        Intaking = new ArmState(0.0, 17.5); //0,17.35-> 0,17.5 after Q4
//coming into champs: Intaking = new ArmState(0.0, 17.35); //0, 16.3  
        AmpScore = new ArmState(-18.0, 24); //16.3,22.6 -- end of day: -18.5,23.0 
        SubShotBack = new ArmState(-0.0, 1.3 - .25); //Set once robot is alive
        SubShotFront = new ArmState(0.0, 6.0); //Set once robot is alive
        FrontTallFixedShot = new ArmState(-17.5, 17.8); //Set once robot is alive
        Source = new ArmState(-13.9, 4.9);
        climb3 = new ArmState(-11,14.3); //-11, 21
        climb1 = new ArmState(-22, 22.3);// Lift state
        climb2 = new ArmState(-13.75, 18.3);// lean into trap
        climb4 = new ArmState(-14., 19.8);// lower endeffector slightly from climb2 (to jimmy it into trap)
        climbVertTrap = new ArmState(-8.1, 13.1);// -8.2->8.1, 13 -> 13.1 after Q4
//        climbVertTrap = new ArmState(-8.7 + .5, 13);// vertical Trap Release (6147 style) AZ -> -8.7, 13.3
        TEST = new ArmState(-1,1);//updates from test function
    }
    public void TEST(double baseAngle, double endeffectorAngle){
        TEST.base = baseAngle;
        TEST.top = endeffectorAngle;
    }
}
