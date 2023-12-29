package org.firstinspires.ftc.teamcode;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

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

        control.GrabPixels();
        BlueBoardDelivery();
        control.SlidesToAuto();
        sleep(400);

        control.DeliverPixelToBoardPos();

        drive.updatePoseEstimate();
        telemetry.addData("x", drive.pose.position.x);
        telemetry.addData("y", drive.pose.position.y);
        telemetry.addData("heading (deg)", Math.toDegrees(drive.pose.heading.toDouble()));
        telemetry.update();

        control.StopNearBoardAuto(false);
        control.ReleaseLeft();
        sleep(400);
        Actions.runBlocking(
                drive.actionBuilder(deliverToBoardPose)
                        .lineToX(44)
                        .build());
        control.SafeStow();
        control.SlidesDown();
        sleep(400);

        drive.updatePoseEstimate();
        telemetry.addData("x", drive.pose.position.x);
        telemetry.addData("y", drive.pose.position.y);
        telemetry.addData("heading (deg)", Math.toDegrees(drive.pose.heading.toDouble()));
        telemetry.update();


        BlueLeftTeamArtPixelDelivery();
        control.DropOnLine();
        sleep(400);
        control.ResetArm();
        sleep(400);

        drive.updatePoseEstimate();
        telemetry.addData("x", drive.pose.position.x);
        telemetry.addData("y", drive.pose.position.y);
        telemetry.addData("heading (deg)", Math.toDegrees(drive.pose.heading.toDouble()));
        telemetry.update();

        // Move the robot to the parking position
        Actions.runBlocking(
                drive.actionBuilder(deliverToFloorPose)
                        .splineToLinearHeading(new Pose2d(24,35,Math.toRadians(180)), Math.toRadians(180))
                        .setTangent(0)
                        .splineToLinearHeading(new Pose2d(48,60,Math.toRadians(270)), Math.toRadians(0))
                        .build());

        drive.updatePoseEstimate();
        telemetry.addData("x", drive.pose.position.x);
        telemetry.addData("y", drive.pose.position.y);
        telemetry.addData("heading (deg)", Math.toDegrees(drive.pose.heading.toDouble()));
        telemetry.update();

        telemetry.update();
    }
    public void BlueBoardDelivery() {
        // Look for potential errors
        if (control.autoPosition == 1) {
            deliverToBoardPose = new Pose2d(46,38,Math.toRadians(180));
            Actions.runBlocking(
                    //drive.actionBuilder(drive.  new Pose2d(50+control.rangeError, 36+control.yawError, Math.toRadians(180)))
                    drive.actionBuilder(startPose)
                            .splineToLinearHeading(new Pose2d(38, 40, Math.toRadians(180)), Math.toRadians(270))
                            .splineToLinearHeading(deliverToBoardPose, Math.toRadians(180))
                            .build());
        } else if (control.autoPosition == 3) {
            deliverToBoardPose = new Pose2d(46,26,Math.toRadians(180));
            Actions.runBlocking(
                    //drive.actionBuilder(drive.  new Pose2d(50+control.rangeError, 36+control.yawError, Math.toRadians(180)))
                    drive.actionBuilder(startPose)
                            .splineToLinearHeading(new Pose2d(38, 28, Math.toRadians(180)), Math.toRadians(270))
                            .splineToLinearHeading(deliverToBoardPose, Math.toRadians(180))
                            .build());
        } else {
            deliverToBoardPose = new Pose2d(46,33,Math.toRadians(180));
            Actions.runBlocking(
                    //drive.actionBuilder(drive.  new Pose2d(50+control.rangeError, 36+control.yawError, Math.toRadians(180)))
                    drive.actionBuilder(startPose)
                            .splineToLinearHeading(new Pose2d(38, 34, Math.toRadians(180)), Math.toRadians(270))
                            .splineToLinearHeading(deliverToBoardPose, Math.toRadians(180))
                            .build());
        }
    }

    public void BlueLeftTeamArtPixelDelivery() {
        if (control.autoPosition == 1) {
            deliverToFloorPose = new Pose2d(10.5, 35, Math.toRadians(180));
            Actions.runBlocking(
                    //drive.actionBuilder(drive.  new Pose2d(50+control.rangeError, 36+control.yawError, Math.toRadians(180)))
                    drive.actionBuilder(deliverToBoardPose)
                            .splineToLinearHeading (deliverToFloorPose, Math.toRadians(180))
                            .build());
        } else if (control.autoPosition == 2) {
            deliverToFloorPose = new Pose2d(12, 35, Math.toRadians(90));
            Actions.runBlocking(
                    //drive.actionBuilder(drive.  new Pose2d(50+control.rangeError, 36+control.yawError, Math.toRadians(180)))
                    drive.actionBuilder(deliverToBoardPose)
                            .splineToLinearHeading(new Pose2d(16, 30, Math.toRadians(180)), Math.toRadians(180))
                            .splineToLinearHeading(deliverToFloorPose, Math.toRadians(180))
                            .build());
        } else if (control.autoPosition == 3) {
            deliverToFloorPose = new Pose2d(-12, 35, Math.toRadians(180));
            Actions.runBlocking(
                    //drive.actionBuilder(drive.  newa Pose2d(50+control.rangeError, 36+control.yawError, Math.toRadians(180)))
                    drive.actionBuilder(deliverToBoardPose)
                            .splineToLinearHeading(deliverToFloorPose, Math.toRadians(180))
                            .build());
        } else {
            deliverToFloorPose = new Pose2d(10.5, 35, Math.toRadians(180));
            Actions.runBlocking(
                    //drive.actionBuilder(drive.  new Pose2d(50+control.rangeError, 36+control.yawError, Math.toRadians(180)))
                    drive.actionBuilder(deliverToBoardPose)
                            .splineToLinearHeading(deliverToFloorPose, Math.toRadians(180))
                            .build());
        }
    }
}
