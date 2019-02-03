package org.usfirst.frc.team3319.robot.custom;

public enum ArmSetpoint {
    BottomHatch(0),
    BottomPort(1),
    MiddleHatch(2),
    MiddlePort(3),
    HighHatch(4),
    HighPort(5);

    public final int value;

    ArmSetpoint(int value) {
        this.value = value;
    }

    public static ArmSetpoint getNext(ArmSetpoint point){
        switch(point){
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
            case BottomPort: return BottomHatch;
            case MiddleHatch: return BottomPort;
            case MiddlePort: return MiddleHatch;
            case HighHatch: return MiddlePort;
            case HighPort: return HighHatch;
            default: return point;
        }
    }
}