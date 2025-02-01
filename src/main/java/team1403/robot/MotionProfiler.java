package team1403.robot;

import dev.doglog.DogLog;

import team1403.robot.subsystems.Elevator;

public class MotionProfiler {
    private double currentPos;
    private double currMotorOutput;
    private double desiredMotorOutput;
    private boolean isRampDone;
    private boolean isGoingUp;
    private boolean isGoingDown;
    private boolean directionFlag;
    private double posError;
    private Elevator m_elevator = new Elevator();
    private double setpoint;


    public MotionProfiler(double m_currentPos) {
        //setPoint = m_setPoint;
        currentPos = m_currentPos;
        currMotorOutput = 0;
        isRampDone = false;
        directionFlag = true;
    }

    public void moveToSetPoint(double setPoint) {
        setpoint = setPoint;
        if(directionFlag && Math.abs(setPoint - currentPos) > Constants.Elevator.Command.setPointMargin) {
            checkDirection(setPoint);
        }
        desiredMotorOutput = getDesiredOutput(setPoint);

        // run ramp function with parameters depending on whether elevator needs to go up or down
        if(isGoingUp) {
            currMotorOutput = ramp(Constants.Elevator.Command.elevatorUpRampUpTime, Constants.Elevator.Command.elevatorUpRampDownTime, currMotorOutput, desiredMotorOutput);
        }
        else if(isGoingDown) {
            currMotorOutput = ramp(Constants.Elevator.Command.elevatorDownRampUpTime, Constants.Elevator.Command.elevatorDownRampDownTime, Math.abs(currMotorOutput), desiredMotorOutput);
        }

        adjustCurrentOutput();
        checkIfReachedSetPoint(setPoint);
        currMotorOutput += m_elevator.calculation(currentPos, setpoint); 
        simulatePos();
        logValues();
    }

    //check whether component is moving up or down
    private void checkDirection(double setPoint) {
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

        directionFlag = false;
    }
    
    private double getDesiredOutput(double setPoint) {
        // set desired motor output equal to the difference between current position and setpoint * a gain constant
        posError = setPoint - currentPos;
        if(isGoingUp) {
            posError *= Constants.Elevator.Command.movementUpGain;
        } 
        else if(isGoingDown){
            posError *= Constants.Elevator.Command.movementDownGain;
        }

        double desiredOutput = posError;

        // checks conditions that don't require any output by the motor 
        if(!isGoingUp && !isGoingDown || (isGoingUp && desiredOutput < 0) || (isGoingDown && desiredOutput > 0)) {
            desiredOutput = 0;
        }

        // clamp desired motor output to a maximum value
        desiredOutput = Math.abs(desiredOutput);
        if(desiredOutput > Constants.Elevator.Command.maxSpeed) {
            desiredOutput = Constants.Elevator.Command.maxSpeed;
        }

        
        return desiredOutput;
    }

    // ramp function gradually brings up the output of the motor to the desired motor output
    private double ramp(double rampUpTime, double rampDownTime, double currentOutput, double desiredOutput) {
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

    private void adjustCurrentOutput() {
        // once ramp function is done and the elevator is moving up or down, set velocity to a minimum value
        if((isGoingUp || isGoingDown) && isRampDone && currMotorOutput < Constants.Elevator.Command.minSpeed) {
            currMotorOutput = Constants.Elevator.Command.minSpeed + m_elevator.calculation(currentPos, setpoint);
        }

        // invert output if elevator is moving down
        if(isGoingDown) {
            currMotorOutput *= -1;
        }
    }

    private void checkIfReachedSetPoint(double setPoint) {
        // if elevator is within the window of the setpoint, stop the motor from running and set booleans to false
        if(currentPos > setPoint - Constants.Elevator.Command.setPointMargin && currentPos < setPoint + Constants.Elevator.Command.setPointMargin) {
            currMotorOutput = 0;
            isGoingUp = false;
            isGoingDown = false;
            directionFlag = true;

        }
        // else set isRampDone to false and continue the following steps above 
        else {
            isRampDone = false;
        }
    }

    private void simulatePos() {
        // simulate position of elevator 
        currentPos += ((currMotorOutput / 100) * Constants.Elevator.Command.simPositionFactor);
        currentPos -= 0.001;
        if(currentPos > 150) {
            currentPos = 150;
        }
        else if (currentPos < 0) {
            currentPos = 0;
        }
    }



    private void logValues() {
        DogLog.log("desired motor output velocity", desiredMotorOutput);
        DogLog.log("current motor output", currMotorOutput);
        DogLog.log("is ramp done", isRampDone);
        DogLog.log("current position", currentPos);
        DogLog.log("position error", posError);
        DogLog.log("motor output error", desiredMotorOutput - currMotorOutput);
        DogLog.log("is going up", isGoingUp);
        DogLog.log("is going down", isGoingDown);
        DogLog.log("checking direction", directionFlag);
        DogLog.log("Feedforward", m_elevator.calculation(currentPos, setpoint));
    }
}
