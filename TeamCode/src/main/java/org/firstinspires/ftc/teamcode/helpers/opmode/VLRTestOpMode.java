package org.firstinspires.ftc.teamcode.helpers.opmode;

import org.firstinspires.ftc.teamcode.helpers.utils.GlobalConfig;

public abstract class VLRTestOpMode extends VLRLinearOpMode{

    @Override
    public void run(){
        GlobalConfig.DEBUG_MODE = true;
        //CommandScheduler.getInstance().schedule(new InstantCommand(()-> VLRSubsystem.getArm().setOperationMode(MainArmConfiguration.OPERATION_MODE.NORMAL)));

        Init();
        while (opModeInInit()) {InitLoop();}

        waitForStart();

        Start();
        while (opModeIsActive()) {Loop();}

        Stop();
    }

    public abstract void Loop();
    public void Init() {}
    public void Start() {}
    public void Stop(){}
    public void InitLoop(){}
}
