package org.firstinspires.ftc.teamcode;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.hardware.dfrobot.HuskyLens;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name="Red Left", preselectTeleOp = "1 Manual Control")
public class RedLeftAuto extends LinearOpMode {
    UniversalControlClass control = new UniversalControlClass(true, false,this);
    private HuskyLens huskyLens;
    private final int READ_PERIOD = 1;
    int leftRightSpikeBound = 150;
    int autoPosition = 0;
    public double pixelDeliverFirstPos = 14.5;

    Pose2d startPose;
    MecanumDrive drive;
    public void runOpMode(){
    startPose = new Pose2d(-38.5,-62.5,Math.toRadians(90));
    drive = new MecanumDrive(hardwareMap, startPose);
        control.Init(hardwareMap);
        control.HuskyLensInit();
        control.AutoStartPos();

        telemetry.update();

        control.WebcamInit(hardwareMap);
        telemetry.update();

        while(!isStarted()){
            control.DetectTeamArtRed();
            telemetry.update();
        }
        waitForStart();

        // lock the pixels
        control.GrabPixels();

        // drive to the middle floor delivery position
        Actions.runBlocking(
            drive.actionBuilder(startPose)
                    .splineToLinearHeading(new Pose2d(-38.5, -12.5, Math.toRadians(90)), Math.toRadians(90))
                .build());

        // Drop the LEFT pixel (put PURPLE on LEFT, YELLOW on RIGHT) on the line
        control.DropOnLine();
        // put the arm back in a safe to travel position
        control.SafeStow();
        control.SpecialSleep(10000);

        // drive to the backboard area
        Actions.runBlocking(
            drive.actionBuilder(new Pose2d(-38.5, -12.5, Math.toRadians(90)))
                    .splineToLinearHeading(new Pose2d(-38,-9, Math.toRadians(90)), Math.toRadians(90))
        //            .setTangent(0)
                    .splineToLinearHeading(new Pose2d(-30,-9, Math.toRadians(180)), Math.toRadians(0))
            //.setTangent(0)
                    .splineToLinearHeading(new Pose2d(30,-9, Math.toRadians(180)), Math.toRadians(0))
                    //.splineToLinearHeading(new Pose2d(52.2,-28, Math.toRadians(180)), Math.toRadians(0))
            .build());

        // Raise Arm to delivery position
        control.ReadyToLiftSlides();
        sleep(150);

        //move arm, slides, drive, and release pixels to board
        DeliverPixelToBoardRoutine();

        // Return Arm to Ready position
        control.ResetArm();
        sleep(400);

        // Move the robot to the parking position
        Actions.runBlocking(
                //drive.actionBuilder(drive.  new Pose2d(50+control.rangeError, 36+control.yawError, Math.toRadians(180)))
                drive.actionBuilder(new Pose2d(52.2, -28, Math.toRadians(180)))
                        .splineToLinearHeading(new Pose2d(50,-15, Math.toRadians(90)), Math.toRadians(90))
                        .build());

        telemetry.update();
    }
    public void DeliverPixelToBoardRoutine(){
        Pose2d deliverPos;
        control.DeliverPixelToBoardPos();
        sleep(750);
        if(control.autoPosition == 1){
            deliverPos = new Pose2d(52.2, -34, Math.toRadians(180));
        }
        else if(control.autoPosition == 2) {
            deliverPos = new Pose2d(52.2, -28, Math.toRadians(180));
        }
        else{
            deliverPos = new Pose2d(52.2, -22, Math.toRadians(180));
        }
        Actions.runBlocking(
                drive.actionBuilder(new Pose2d(30,-9, Math.toRadians(180)))
                        .setTangent(0)
                        .splineToLinearHeading(deliverPos, Math.toRadians(0))
                        .build());
        control.ReleaseRight();
        control.ReleaseLeft();
        sleep(750);
    }
}