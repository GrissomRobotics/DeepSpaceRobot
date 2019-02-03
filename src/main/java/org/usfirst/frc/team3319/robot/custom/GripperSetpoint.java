package org.usfirst.frc.team3319.robot.custom;

public enum GripperSetpoint {
    FullyFolded(0),
    Expel(1),
    Intake(2);

    public final int value;

    GripperSetpoint(int value) {
        this.value = value;
    }

    public static GripperSetpoint getNext(GripperSetpoint point){
        switch(point){
            case FullyFolded: return Expel;
            case Expel: return Intake;
            default: return point;
        }
    }

    public static GripperSetpoint getPrevious(GripperSetpoint point){
        switch(point){
            case Expel: return FullyFolded;
            case Intake: return Expel;
            default: return point;
        }
    }
}