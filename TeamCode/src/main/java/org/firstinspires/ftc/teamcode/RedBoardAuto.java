package org.firstinspires.ftc.teamcode;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name="Red Board", preselectTeleOp = "1 Manual Control")
public class RedBoardAuto extends LinearOpMode {
    UniversalControlClass control = new UniversalControlClass(true, false,this);
    MecanumDrive drive;
    Pose2d startPose;
    Action BoardTraj2;
    Pose2d deliverToFloorPose;
    Pose2d deliverToBoardPose;

    public void runOpMode(){
        startPose = new Pose2d(12,-62.5,Math.toRadians(90));
        drive = new MecanumDrive(hardwareMap, startPose);
        control.Init(hardwareMap);
        control.HuskyLensInit();
        control.AutoStartPos();

        drive.localizer.update();
        telemetry.update();

        while(!isStarted()){
            control.DetectTeamArtRedBoard();
            telemetry.update();
        }
        waitForStart();

        // lock the pixels
        control.GrabPixels();

        RedBoardDelivery();
        control.TagCorrection();
        drive.updatePoseEstimate();
        Actions.runBlocking(
                drive.actionBuilder(drive.pose)
                        .strafeToLinearHeading(new Vector2d(drive.pose.position.x + .5, drive.pose.position.y + control.distanceCorrectionLR_HL), Math.toRadians(180))
                        .build());
        control.SlidesToAuto();
        sleep(150);
        control.DeliverPixelToBoardPos();
        control.StopNearBoardAuto(false);
        sleep(200);

        drive.updatePoseEstimate();
        Actions.runBlocking(
                drive.actionBuilder(drive.pose)
                        .lineToX(47)
                        .build());
        drive.updatePoseEstimate();
        control.ResetArmAuto();
        //sleep(400);

        RedRightTeamArtPixelDelivery();
        //sleep (5000);
       // control.SlidesToAuto();
        control.DropOnLine();
        control.ResetArmAuto();
        sleep(400);

        if (control.autoPosition == 2){
            Actions.runBlocking(
                    drive.actionBuilder(deliverToFloorPose)
                            .splineToLinearHeading(new Pose2d(12, -42, Math.toRadians(90)), Math.toRadians(270))
                            .build());
        }
        drive.updatePoseEstimate();
        //PARK
        Actions.runBlocking(
                drive.actionBuilder(drive.pose)
                        .setTangent(Math.toRadians(270))
                        .splineToLinearHeading(new Pose2d(60,-61.5, Math.toRadians(90)), Math.toRadians(0))
                        .build());

        telemetry.update();

    }

    public void RedBoardDelivery() {
        //***POSITION 1***
        if (control.autoPosition == 1) {
            deliverToBoardPose = new Pose2d(47.5,-25,Math.toRadians(180));
            Actions.runBlocking(
                    drive.actionBuilder(startPose)
                            .splineToLinearHeading(deliverToBoardPose, Math.toRadians(0))
                            .build());
            drive.updatePoseEstimate();
        }
        //***POSITION 3***
        else if (control.autoPosition == 3) {
            deliverToBoardPose = new Pose2d(47.5,-35,Math.toRadians(180));
            Actions.runBlocking(
                    drive.actionBuilder(startPose)
                            .splineToLinearHeading(deliverToBoardPose, Math.toRadians(0))
                            .build());
            drive.updatePoseEstimate();
        }
        //***POSITION 2***
        else {
            deliverToBoardPose = new Pose2d(47.5,-30,Math.toRadians(180));
            Actions.runBlocking(
                    drive.actionBuilder(startPose)
                            .splineToLinearHeading(deliverToBoardPose, Math.toRadians(0))
                            .build());
            drive.updatePoseEstimate();
        }
    }
    public void RedRightTeamArtPixelDelivery() {
        if (control.autoPosition == 1) {
            deliverToFloorPose = new Pose2d(12, -30, Math.toRadians(0));
            Actions.runBlocking(
                    drive.actionBuilder(deliverToBoardPose)
                            .splineToLinearHeading(new Pose2d(27,-33, Math.toRadians(0)), Math.toRadians(180))
                            //.setTangent(Math.toRadians(180))
                            .lineToX(0)
                            .lineToX(12)
                            .strafeTo(new Vector2d(12, -30))
                            //.splineToLinearHeading(new Pose2d(0,-33, Math.toRadians(0)), Math.toRadians(0))
                            //.splineToLinearHeading(deliverToFloorPose, Math.toRadians(0))
                            .build());
            drive.updatePoseEstimate();
        }
        else if (control.autoPosition == 3) {
            deliverToFloorPose = new Pose2d(12, -36, Math.toRadians(180));
            Actions.runBlocking(
                    drive.actionBuilder(deliverToBoardPose)
                            .setTangent(Math.toRadians(180))
                            .splineToLinearHeading (deliverToFloorPose, Math.toRadians(180))
                            .build());
            drive.updatePoseEstimate();
        }
        else {
            deliverToFloorPose = new Pose2d(12, -36, Math.toRadians(270));
            Actions.runBlocking(
                    drive.actionBuilder(deliverToBoardPose)
                            .splineToLinearHeading(new Pose2d(12, -30, Math.toRadians(-90)), Math.toRadians(180))
                            .splineToLinearHeading(deliverToFloorPose, Math.toRadians(270))
                            .build());
            drive.updatePoseEstimate();
        }
    }
}