package org.firstinspires.ftc.teamcode.auto.sample;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.localization.Pose;

@Config
public class PointsSample {
    public static Pose START_POSE = new Pose(10, 111.5, 0);

    public static Pose BUCKET_HIGH_SCORE_POSE = new Pose(24, 118.5, rad(-45));

    public static Pose BUCKET_HIGH_SCORE_POSE_SUB = new Pose(25, 117.5, rad(-45));

    public static Pose FIRST_MARK_GRAB = new Pose(29, 118.4, 0);

    public static Pose SECOND_MARK_GRAB = new Pose(FIRST_MARK_GRAB.getX(), FIRST_MARK_GRAB.getY() + 9.75, 0);

    public static Pose THIRD_MARK_GRAB = new Pose(32.5, 130, rad(30));

    public static Pose SUB_GRAB_0 = new Pose(53.2, 111.5435, Math.toRadians(-45));

    public static Pose SUB_GRAB_CONTROL_1 = new Pose(57.3124, 107.4);

    public static Pose SUB_GRAB_CONTROL_2 = new Pose(61.775, 104.6);

    public static Pose SUB_GRAB = new Pose(62, 95.5, Math.toRadians(-90));

    public static Pose[] GRAB_POSES = {FIRST_MARK_GRAB, SECOND_MARK_GRAB, THIRD_MARK_GRAB};




//    OLD POINTS:
//    public static Pose BUCKET_HIGH_SCORE_POSE = new Pose(30, 116, rad(-45));
//    public static Pose BUCKET_HIGH_SCORE_TRUE_POSE = new Pose(20, 118, rad(-45));
//    public static Pose FIRST_MARK_READY_POSE = new Pose(26, 118.7, 0);
//    public static Pose FIRST_MARK_GRAB_POSE = new Pose(32.3, 118.7, 0);
//
//    public static Pose SECOND_MARK_READY_POSE = new Pose(26, 129, 0);
//    public static Pose SECOND_MARK_GRAB_POSE = new Pose(32.3, 129, 0);
//
//    public static Pose THIRD_MARK_READY_POSE = new Pose(35, 120, rad(60));
//    public static Pose THIRD_MARK_GRAB_POSE = new Pose(40, 125.7, rad(60));
//
//    public static Pose SUB_PRE_BEZIER_POSE = new Pose(47,120, rad(-70));
//    public static Pose SUB_PRE_PREGRAB_POSE = new Pose(60,117, rad(-90));
//    public static Pose SUB_GRAB_POSE = new Pose(60,97, rad(-90));

    public static double rad(double deg) {
        return Math.toRadians(deg);
    }
}
