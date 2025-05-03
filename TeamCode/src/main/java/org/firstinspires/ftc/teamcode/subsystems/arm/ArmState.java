package org.firstinspires.ftc.teamcode.subsystems.arm;


public class ArmState {
    private static State currentState;

    public static void initialize(){
        if (currentState == null) {currentState = State.IN_ROBOT;}
    }

    public static State get() {
        return currentState;
    }

    public static void set(State stateValue) {currentState = stateValue;}

    public static boolean isCurrentState(State state) {
        return currentState == state;
    }

    public static boolean isCurrentState(State ...states) {
        for (State state : states) {
            if (currentState == state) {return true;}
        }
        return false;
    }

    public static void resetAll() {
        currentState = State.IN_ROBOT;
    }

    public enum State {
        IN_ROBOT,
        SAMPLE_INTAKE,
        SAMPLE_SCORE,
        SPECIMEN_INTAKE,
        SPECIMEN_SCORE_FRONT,
        SPECIMEN_SCORE_BACK,
        HANG_SECOND_STAGE,
        HANG_THIRD_STAGE
    }
}