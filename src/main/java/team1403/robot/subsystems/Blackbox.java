package team1403.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import team1403.robot.commands.CoralDepositCommand;

public class Blackbox {
    private static boolean trigger;
    private static int coralLevel;
    
    public static void setTrigger(boolean t){
        trigger = t;
    }

    public static boolean getTrigger(){
        return trigger;
    }
    
    public static void setCoralLevel(int coral){
        coralLevel = coral;
    }

    public static int getCoralLevel(){
        return coralLevel;
    }

    public static Command place(){
        return new CoralDepositCommand();
    }


}
