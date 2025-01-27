package team1403.robot;

public class MotionProfiler {
    private double setPoint;
    private double currentPos;
    private double currMotorOutput;
    private double desiredMotorOutput;
    private boolean isRampDone;
    private boolean isGoingUp;
    private boolean isGoingDown;


    public MotionProfiler(double m_setPoint, double m_currentPos, double m_currMotorOutput) {
        setPoint = m_setPoint;
        currentPos = m_currentPos;
        currMotorOutput = m_currMotorOutput;
        isRampDone = false;

        checkDirection();
    }

    public void moveToSetPoint() {
        // set desired motor output equal to the difference between current position and setpoint * a gain constant
        double posError = setPoint - currentPos;
        if(isGoingUp) {
            posError *= Constants.Elevator.Command.movementUpGain;
        } 
        else if(isGoingDown){
            posError *= Constants.Elevator.Command.movementDownGain;
        }
        
        desiredMotorOutput = posError;

        // checks conditions that don't require any output by the motor 
        if(!isGoingUp && !isGoingDown || (isGoingUp && desiredMotorOutput < 0) || (isGoingDown && desiredMotorOutput > 0)) {
            desiredMotorOutput = 0;
        }

        // clamp desired motor output to a maximum value
        desiredMotorOutput = Math.abs(desiredMotorOutput);
        if(desiredMotorOutput > Constants.Elevator.Command.maxSpeed) {
            desiredMotorOutput = Constants.Elevator.Command.maxSpeed;
        }

        // run ramp function with parameters depending on whether elevator needs to go up or down
        if(isGoingUp) {
            currMotorOutput = ramp(Constants.Elevator.Command.elevatorUpRampUpTime, Constants.Elevator.Command.elevatorUpRampDownTime, currMotorOutput, desiredMotorOutput);
        }
        else if(isGoingDown) {
            currMotorOutput = ramp(Constants.Elevator.Command.elevatorDownRampUpTime, Constants.Elevator.Command.elevatorDownRampDownTime, Math.abs(currMotorOutput), desiredMotorOutput);
        }

        // once ramp function is done and the elevator is moving up or down, set velocity to a minimum value
        if((isGoingUp || isGoingDown) && isRampDone && currMotorOutput < Constants.Elevator.Command.minSpeed) {
            currMotorOutput = Constants.Elevator.Command.minSpeed;
        }

        // invert output if elevator is moving down
        if(isGoingDown) {
            currMotorOutput *= -1;
        }

        // if elevator is within the window of the setpoint, stop the motor from running and set booleans to false
        if(currentPos > setPoint - Constants.Elevator.Command.setPointMargin && currentPos < setPoint + Constants.Elevator.Command.setPointMargin) {
            currMotorOutput = 0;
            isGoingUp = false;
            isGoingDown = false;

        }
        // else set isRampDone to false and continue the following steps above 
        else {
            isRampDone = false;
        }

        // simulate position of elevator 
        currentPos += (currMotorOutput / 100) * Constants.Elevator.Command.simPositionFactor;
        if(currentPos > 150) {
            currentPos = 150;
        }
        else if (currentPos < 0) {
            currentPos = 0;
        }
    }

    //check whether component is moving up or down
    public void checkDirection() {
        if(setPoint > currentPos - Constants.Elevator.Command.setPointMargin) {
            isGoingUp = true;
            isGoingDown = false;
        } 
        else if(setPoint < currentPos + Constants.Elevator.Command.setPointMargin) {
            isGoingUp = false;
            isGoingDown = true;
        } 
        else {
            isGoingUp = false;
            isGoingDown = false;
        } 
    }  

    // ramp function gradually brings up the output of the motor to the desired motor output
    public double ramp(double rampUpTime, double rampDownTime, double currentOutput, double desiredOutput) {
        System.out.println(currentOutput);
        // if desired output is greater than current output, run upwards ramp function 
        if(desiredOutput > currentOutput) {
        // increment current output by 100/rampUpTime/cycle rate and if that output is less than desired 
        if((currentOutput + (100/(rampUpTime/0.02)) < desiredOutput)) {
            currentOutput += (100/(rampUpTime/0.02));
        }
        // set current output to desired output
        else {
            currentOutput = desiredOutput;
        }
        }
        // if desired output is less than current output, run downwards ramp function 
        else if(desiredOutput < currentOutput) {
        if((currentOutput - (100/(rampDownTime/0.02)) > desiredOutput)) 
        {
            // decrement current output by 100/rampDownTime/cycle rate and if that output is less than desired 
            currentOutput -= (100/(rampDownTime/0.02));

        }
        else {
            currentOutput = desiredOutput;
        }
        }
        // if current motor output reaches the desired output set isRampDone to true
        if(desiredOutput == currentOutput) {
        isRampDone = true;
        }
        // returns our current output
        return currentOutput;
    }

    public void logValues() {

    }
}
