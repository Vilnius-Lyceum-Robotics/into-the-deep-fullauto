package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.auto.specimen.PointsSpecimen.PICK_UP_SPECIMENS_FROM_HUMAN_PLAYER;
import static org.firstinspires.ftc.teamcode.auto.specimen.PointsSpecimen.START_POSE;
import static org.firstinspires.ftc.teamcode.auto.specimen.PointsSpecimen.pickupAngle;

import com.arcrobotics.ftclib.command.Command;
import com.outoftheboxrobotics.photoncore.Photon;
import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.auto.specimen.AutonomousPeriodActionSpecimen;
import org.firstinspires.ftc.teamcode.helpers.opmode.VLRAutoTestOpMode;
import org.firstinspires.ftc.teamcode.subsystems.limelight.LimelightYoloReader;


@Autonomous(name = "VLR_SpecimenAuto", group = "!TELEOP")
@Photon
public class VLRSpecimenAuto extends VLRAutoTestOpMode {
    @Override
    public Pose StartPose() {return START_POSE;}

    @Override
    public Command autoCommand(Follower f, LimelightYoloReader reader){
        return new AutonomousPeriodActionSpecimen(f, reader);
    }

    @Override
    public boolean SpecimenOnly(){
        return true;
    }
}
