package team1403.robot.commands;

import com.ctre.phoenix.CANifier.LEDChannel;

import edu.wpi.first.wpilibj2.command.Command;
import team1403.robot.subsystems.Blackbox;
import team1403.robot.subsystems.LEDSubsystem;
import team1403.robot.subsystems.LEDSubsystem.LEDConfig;

public class LightCommand extends Command{
    
    //Variable for local copy of LED subsystem
    private LEDSubsystem m_LED;

    /***
     * Runs the lights based of robot state
     * 
     * @param LED LED subsystem
     */
    public LightCommand(LEDSubsystem LED) {
        m_LED = LED;

        addRequirements(m_LED);
    }

    @Override
    public void initialize() {}

    @Override
    public void execute() {
        switch(Blackbox.robotState){
            case loading:
<<<<<<< Updated upstream
                m_LED.setLEDcolor(LEDSubsystem.Color.Green);
            break;
            case driving:
                m_LED.setLEDcolor(LEDSubsystem.Color.Red);
=======
                m_LED.setLEDcolor(LEDConfig.Style.Solid, LEDConfig.Color.Yellow);
            break;
            case driving:
                m_LED.setLEDcolor(LEDConfig.Style.Solid, LEDConfig.Color.Green);
>>>>>>> Stashed changes
            break;
            case aligning:
                m_LED.setLEDcolor(LEDConfig.Style.Solid, LEDConfig.Color.Blue);
            break;
            case placing:
<<<<<<< Updated upstream
                m_LED.setLEDcolor(LEDSubsystem.Color.Grey);
            break;
            case exiting:
                m_LED.setLEDcolor(LEDSubsystem.Color.Pink);
=======
                m_LED.setLEDcolor(LEDConfig.Style.Solid, LEDConfig.Color.Pink);
            break;
            case exiting:
                m_LED.setLEDcolor(LEDConfig.Style.Solid, LEDConfig.Color.Red);
>>>>>>> Stashed changes
            break;
            case ManualElevator:
                m_LED.setLEDcolor(LEDConfig.Style.Solid, LEDConfig.Color.White);
            break;
        }

    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
