package org.firstinspires.ftc.teamcode;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name="Blue Board", preselectTeleOp="1 Manual Control")
public class BlueBoardAuto extends LinearOpMode {
    UniversalControlClass control = new UniversalControlClass(true, false, this);
    MecanumDrive drive;
    Pose2d startPose;
    Pose2d deliverToFloorPose;
    Pose2d deliverToBoardPose;
    public void runOpMode() {
        startPose = new Pose2d(12, 62.5, Math.toRadians(270));
        drive = new MecanumDrive(hardwareMap, startPose);
        control.Init(hardwareMap);
        control.AutoStartPos();

        drive.localizer.update();
        telemetry.update();

        while (!isStarted()) {
            control.DetectTeamArtBlue();
            telemetry.update();
        }
        waitForStart();

        // lock the pixels
        control.GrabPixels();

        BlueBoardDelivery();
        control.SlidesToAuto();
        sleep(150);
        control.DeliverPixelToBoardPos();
        control.StopNearBoardAuto(false);
        sleep(200);

        drive.updatePoseEstimate();
        Actions.runBlocking(
                drive.actionBuilder(new Pose2d(drive.pose.position.x, drive.pose.position.y, Math.toRadians(180)))
                        .lineToX(47)
                        .build());
        control.ResetArmAuto();
        sleep(400);

        BlueLeftTeamArtPixelDelivery();
        control.DropOnLine();
        control.ResetArm();
        sleep(400);

        // Move the robot to the parking position
        Actions.runBlocking(
                drive.actionBuilder(deliverToFloorPose)
                        .setTangent(Math.toRadians(90))
                        .splineToLinearHeading(new Pose2d(60,60,Math.toRadians(270)), Math.toRadians(0))
                        .build());

        telemetry.update();
    }
    public void BlueBoardDelivery() {
        //***POSITION 1***
        if (control.autoPosition == 1) {
            deliverToBoardPose = new Pose2d(50,38,Math.toRadians(180));
            Actions.runBlocking(
                    drive.actionBuilder(startPose)
                            .splineToLinearHeading(deliverToBoardPose, Math.toRadians(0))
                            .build());
        }
        //***POSITION 3***
        else if (control.autoPosition == 3) {
            deliverToBoardPose = new Pose2d(46,26,Math.toRadians(180));
            Actions.runBlocking(
                    drive.actionBuilder(startPose)
                            .splineToLinearHeading(deliverToBoardPose, Math.toRadians(0))
                            .build());
        }
        //***POSITION 2***
        else {
            deliverToBoardPose = new Pose2d(46,33,Math.toRadians(180));
            Actions.runBlocking(
                    drive.actionBuilder(startPose)
                            .splineToLinearHeading(deliverToBoardPose, Math.toRadians(0))
                            .build());
        }
    }

    public void BlueLeftTeamArtPixelDelivery() {
        if (control.autoPosition == 1) {
            deliverToFloorPose = new Pose2d(12, 32, Math.toRadians(180));
            Actions.runBlocking(
                    drive.actionBuilder(deliverToBoardPose)
                            .splineToLinearHeading(deliverToFloorPose, Math.toRadians(180))
                            .build());
        }
        else if (control.autoPosition == 3) {
            deliverToFloorPose = new Pose2d(12, 33, Math.toRadians(0));
            Actions.runBlocking(
                    drive.actionBuilder(deliverToBoardPose)
                            .splineToLinearHeading(new Pose2d(27, 33, Math.toRadians(0)), Math.toRadians(0))
                            .setTangent(Math.toRadians(180))
                            .splineToLinearHeading(new Pose2d(0,33,Math.toRadians(0)), Math.toRadians(0))
                            .splineToLinearHeading (deliverToFloorPose, Math.toRadians(180))
                            .build());
        }
        else {
            deliverToFloorPose = new Pose2d(12, 36, Math.toRadians(90));
            Actions.runBlocking(
                    drive.actionBuilder(deliverToBoardPose)
                            .splineToLinearHeading(new Pose2d(12, 30, Math.toRadians(90)), Math.toRadians(180))
                            .splineToLinearHeading(deliverToFloorPose, Math.toRadians(90))
                            .build());
        }
    }
}
