package org.firstinspires.ftc.teamcode.helpers.persistence;

import com.pedropathing.localization.Pose;

import java.util.logging.Logger;

public final class PoseSaver {
    static volatile Pose pedroPose = new Pose(0, 0, 0);
    static boolean poseSaved = false;

    public static void setPedroPose(Pose pose) {
        Logger.getLogger("PoseSaver").fine("Saved pedro pose");
        pedroPose = pose;
        poseSaved = true;
    }

    public static Pose getPedroPose() {
        return pedroPose;
    }

    public static boolean isPoseSaved() {
        return poseSaved;
    }

    public static void resetSavedPose() {
        poseSaved = false;
        pedroPose = new Pose(0, 0, 0);
    }
}
