package org.usfirst.frc.team3319.robot.custom;

public enum ArmSetpoint {
    //TODO I don't know at this time if beginning and the bottom hatch are the same
    BeginningConfiguration(0),
    BottomHatch(1),
    BottomPort(2),
    MiddleHatch(3),
    MiddlePort(4),
    HighHatch(5),
    HighPort(6);

    public final int value;

    ArmSetpoint(int value) {
        this.value = value;
    }

    public static ArmSetpoint getNext(ArmSetpoint point){
        switch(point){
            case BeginningConfiguration: return BottomHatch;
            case BottomHatch: return BottomPort;
            case BottomPort: return MiddleHatch;
            case MiddleHatch: return MiddlePort;
            case MiddlePort: return HighHatch;
            case HighHatch: return HighPort;
            default: return point;
        }
    }

    public static ArmSetpoint getPrevious(ArmSetpoint point){
        switch(point){
            case BottomHatch: return BeginningConfiguration;
            case BottomPort: return BottomHatch;
            case MiddleHatch: return BottomPort;
            case MiddlePort: return MiddleHatch;
            case HighHatch: return MiddlePort;
            case HighPort: return HighHatch;
            default: return point;
        }
    }
}