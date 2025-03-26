package team1403.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import team1403.robot.subsystems.Blackbox;
import team1403.robot.subsystems.ElevatorSubsystem;
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
                m_LED.setLEDcolor(LEDConfig.Color.Yellow);
            break;
            case driving:
                m_LED.setLEDcolor(LEDConfig.Color.Green);
            break;
            case aligning:
                m_LED.setLEDcolor(LEDConfig.Color.Blue);
            break;
            case placing:
                m_LED.setLEDcolor(LEDConfig.Color.Pink);
            break;
            case exiting:
                m_LED.setLEDcolor(LEDConfig.Color.Red);
            break;
            case ManualElevator:
                m_LED.setLEDcolor(LEDConfig.Color.White);
            break;
        }
        if(!ElevatorSubsystem.isAtSetpoint()) {
            if(ElevatorSubsystem.isGoingUp){
                m_LED.setLEDcolor(LEDConfig.Style.Upwards, LEDConfig.Color.Green);
            }
            else if(!ElevatorSubsystem.isGoingUp) {
                m_LED.setLEDcolor(LEDConfig.Style.Downwards, LEDConfig.Color.Grey);
            }
        }   
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
