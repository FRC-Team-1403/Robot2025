package team1403.robot.commands;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import team1403.robot.subsystems.Blackbox;
import team1403.robot.subsystems.ElevatorSubsystem;
import team1403.robot.subsystems.LEDSubsystem;
import team1403.robot.subsystems.LEDSubsystem.LEDConfig;

public class LightCommand extends Command{
    
    //Variable for local copy of LED subsystem
    private LEDSubsystem m_LED;
    private ElevatorSubsystem m_elevator;

    /***
     * Runs the lights based of robot state
     * 
     * @param LED LED subsystem
     */
    public LightCommand(LEDSubsystem LED, ElevatorSubsystem elevator) {
        m_LED = LED;
        m_elevator = elevator;

        addRequirements(m_LED);
    }

    @Override
    public void initialize() {}

    @Override
    //Runs every 20 milliseconds
    public void execute() {
    if(true){
        if(!m_elevator.isAtSetpoint()) {
            if(m_elevator.isGoingUp){
                m_LED.setLEDcolor(LEDConfig.Style.Upwards, LEDConfig.Color.Green);
            }
            else if(!m_elevator.isGoingUp) {
                m_LED.setLEDcolor(LEDConfig.Style.Downwards, LEDConfig.Color.Grey);
            }
        }

        else if(DriverStation.isDisabled()) {
            m_LED.setLEDcolor(LEDConfig.Style.Strobe, LEDConfig.Color.White);
        }

        else if(DriverStation.isEStopped()) {
            m_LED.setLEDcolor(LEDConfig.Style.Strobe, LEDConfig.Color.Red);
        }
        
        else {
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
        }
    }
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
