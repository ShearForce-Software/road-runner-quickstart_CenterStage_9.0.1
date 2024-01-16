package org.firstinspires.ftc.teamcode;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name="Red Board", preselectTeleOp = "1 Manual Control")
public class RedBoardAuto extends LinearOpMode {
    UniversalControlClass control = new UniversalControlClass(true, false,this);
    MecanumDrive drive;
    Pose2d startPose;
    Pose2d deliverToFloorPose;
    Pose2d deliverToBoardPose;

    public void runOpMode(){
        startPose = new Pose2d(12,-62.5,Math.toRadians(90));
        drive = new MecanumDrive(hardwareMap, startPose);
        control.Init(hardwareMap);
        control.HuskyLensInit();
        control.AutoStartPos();

        telemetry.update();

        while(!isStarted()){
            control.DetectTeamArtRed();
            telemetry.update();
        }
        waitForStart();

        // lock the pixels
        control.GrabPixels();

        RedBoardDelivery();
        control.SlidesToAuto();
        sleep(400);
        control.DeliverPixelToBoardPos();
        control.ReleaseLeft();
        sleep(400);
        control.ResetArmAuto();
        sleep(400);

        RedRightTeamArtPixelDelivery();
        control.DropOnLine();
        sleep(400);
        control.SafeStow();
        sleep(400);

        Actions.runBlocking(
                drive.actionBuilder(deliverToFloorPose)
                        .splineToLinearHeading(new Pose2d(24,-35,Math.toRadians(180)), Math.toRadians(180))
                        .setTangent(0)
                        .splineToLinearHeading(new Pose2d(48,-60,Math.toRadians(90)), Math.toRadians(0))
                        .build());
        //)
    }
    public void RedBoardDelivery() {
        // Look for potential errors
        if (control.autoPosition == 1) {
            deliverToBoardPose = new Pose2d(50,-22, Math.toRadians(180));
            Actions.runBlocking(
                    //drive.actionBuilder(drive.  new Pose2d(50+control.rangeError, 36+control.yawError, Math.toRadians(180)))
                    drive.actionBuilder(startPose)
                            .splineToLinearHeading(new Pose2d(38, -22, Math.toRadians(180)), Math.toRadians(90))
                            .splineToLinearHeading(deliverToBoardPose, Math.toRadians(180))
                            .build());
        } else if (control.autoPosition == 2) {
            deliverToBoardPose = new Pose2d(50,-28, Math.toRadians(180));
            Actions.runBlocking(
                    //drive.actionBuilder(drive.  new Pose2d(50+control.rangeError, 36+control.yawError, Math.toRadians(180)))
                    drive.actionBuilder(startPose)
                            .splineToLinearHeading(new Pose2d(38, -28, Math.toRadians(180)), Math.toRadians(90))
                            .splineToLinearHeading(deliverToBoardPose, Math.toRadians(180))
                            .build());
        } else if (control.autoPosition == 3) {
            deliverToBoardPose = new Pose2d(50,-34, Math.toRadians(180));
            Actions.runBlocking(
                    //drive.actionBuilder(drive.  new Pose2d(50+control.rangeError, 36+control.yawError, Math.toRadians(180)))
                    drive.actionBuilder(startPose)
                            .splineToLinearHeading(new Pose2d(38, -34, Math.toRadians(180)), Math.toRadians(90))
                            .splineToLinearHeading(deliverToBoardPose, Math.toRadians(180))
                            .build());
        } else {
            deliverToBoardPose = new Pose2d(50,-28, Math.toRadians(180));
            Actions.runBlocking(
                    //drive.actionBuilder(drive.  new Pose2d(50+control.rangeError, 36+control.yawError, Math.toRadians(180)))
                    drive.actionBuilder(startPose)
                            .splineToLinearHeading(new Pose2d(38, 34, Math.toRadians(180)), Math.toRadians(90))
                            .splineToLinearHeading(deliverToBoardPose, Math.toRadians(180))
                            .build());
        }
    }
    public void RedRightTeamArtPixelDelivery() {
        if (control.autoPosition == 3) {
            deliverToFloorPose = new Pose2d(10.5, -35, Math.toRadians(180));
            Actions.runBlocking(
                    //drive.actionBuilder(drive.  new Pose2d(50+control.rangeError, 36+control.yawError, Math.toRadians(180)))
                    drive.actionBuilder(deliverToBoardPose)
                            .splineToLinearHeading (deliverToFloorPose, Math.toRadians(180))
                            .build());
        } else if (control.autoPosition == 2) {
            deliverToFloorPose = new Pose2d(12, -36, Math.toRadians(270));
            Actions.runBlocking(
                 //drive.actionBuilder(drive.  new Pose2d(50+control.rangeError, 36+control.yawError, Math.toRadians(180)))
                       //Actions.runBlocking(
                            //drive.actionBuilder(drive.  new Pose2d(50+control.rangeError, 36+control.yawError, Math.toRadians(180)))
                            drive.actionBuilder(deliverToBoardPose)
                                    .splineToLinearHeading(new Pose2d(12, -35, Math.toRadians(180)), Math.toRadians(180))
                                    .splineToLinearHeading(deliverToFloorPose, Math.toRadians(180))
                                    .build());
        } else if (control.autoPosition == 1) {
            deliverToFloorPose = new Pose2d(-12, -35, Math.toRadians(180));
            Actions.runBlocking(
                    //drive.actionBuilder(drive.  new Pose2d(50+control.rangeError, 36+control.yawError, Math.toRadians(180)))
                    drive.actionBuilder(deliverToBoardPose)
                            .splineToLinearHeading(deliverToFloorPose, Math.toRadians(180))
                            .build());
        } else {
            Actions.runBlocking(
                    //drive.actionBuilder(drive.  new Pose2d(50+control.rangeError, 36+control.yawError, Math.toRadians(180)))
                    drive.actionBuilder(startPose)
                            .splineToLinearHeading(new Pose2d(14.5, -12.5, Math.toRadians(90)), Math.toRadians(90))
                            .build());
        }
    }
}