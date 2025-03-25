package team1403.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import team1403.robot.subsystems.Blackbox;
import team1403.robot.subsystems.LEDSubsystem;

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
                m_LED.setLEDcolor(LEDSubsystem.Color.Yellow);
            break;
            case driving:
                m_LED.setLEDcolor(LEDSubsystem.Color.Green);
            break;
            case aligning:
                m_LED.setLEDcolor(LEDSubsystem.Color.Blue);
            break;
            case placing:
                m_LED.setLEDcolor(LEDSubsystem.Color.Pink);
            break;
            case exiting:
                m_LED.setLEDcolor(LEDSubsystem.Color.Red);
            break;
            case ManualElevator:
                m_LED.setLEDcolor(LEDSubsystem.Color.White);
            break;
        }

    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
