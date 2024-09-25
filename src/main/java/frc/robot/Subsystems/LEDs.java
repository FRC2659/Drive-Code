package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.DigitalOutput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class LEDs extends SubsystemBase{
    final AddressableLED m_LED = new AddressableLED(9);

    final AddressableLEDBuffer m_ledBuffer = new AddressableLEDBuffer(171);

public LEDs(){
    m_LED.setLength(m_ledBuffer.getLength());

    // Set the data
    m_LED.setData(m_ledBuffer);
    m_LED.start();
        
}



@Override
public void periodic() {
    for (var i = 0; i < m_ledBuffer.getLength(); i++) {
        // Sets the specified LED to the RGB values for red
        //m_ledBuffer.setRGB(i, 255, 255  , 255);
        if (SuperStructure.lightMode == 0) {
           m_ledBuffer.setRGB(i, 255, 0 , 0); 
        } else if (SuperStructure.lightMode == 1){
            m_ledBuffer.setRGB(i, 247,247,247);
        } else if (SuperStructure.lightMode == 2){
            m_ledBuffer.setRGB(i, 255,0,213);
        } else if (SuperStructure.lightMode == 3){
            m_ledBuffer.setRGB(i, 255,100,10);
        } else if (SuperStructure.lightMode == 4){
            m_ledBuffer.setRGB(i, 141,3,183);
        } else if (SuperStructure.lightMode == 5) {
            m_ledBuffer.setRGB(i, 255,30,0);
        } else if (SuperStructure.lightMode == 6) {
            m_ledBuffer.setRGB(i, 100,255,12);
        }
     }
     //System.out.println("LEDs BEING SET");
     m_LED.setData(m_ledBuffer);
}


}
