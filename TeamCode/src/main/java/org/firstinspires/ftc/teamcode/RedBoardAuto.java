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

        RedRightTeamArtPixelDelivery();
        control.DropOnLine();
        control.ResetArmAuto();
        sleep(400);

        //PARK
        Actions.runBlocking(
                drive.actionBuilder(deliverToFloorPose)
                        .setTangent(Math.toRadians(270))
                        .splineToLinearHeading(new Pose2d(60,-60, Math.toRadians(90)), Math.toRadians(0))
                        .build());
    }
    public void RedBoardDelivery() {
        // Look for potential errors
        //***POSITION 1***
        if (control.autoPosition == 1) {
            deliverToBoardPose = new Pose2d(50,-28,Math.toRadians(180));
            Actions.runBlocking(
                    //drive.actionBuilder(drive.  new Pose2d(50+control.rangeError, 36+control.yawError, Math.toRadians(180)))
                    drive.actionBuilder(new Pose2d(30,-9, Math.toRadians(180)))
                            .splineToLinearHeading(deliverToBoardPose, Math.toRadians(0))
                            .build());
        }
        //***POSITION 3***
        else if (control.autoPosition == 3) {
            deliverToBoardPose = new Pose2d(50,-38.5,Math.toRadians(180));
            Actions.runBlocking(
                    //drive.actionBuilder(drive.  new Pose2d(50+control.rangeError, 36+control.yawError, Math.toRadians(180)))
                    drive.actionBuilder(new Pose2d(30, -9, Math.toRadians(180)))
                            .splineToLinearHeading(deliverToBoardPose, Math.toRadians(0))
                            .build());
        }
        //***POSITION 2***
        else {
            deliverToBoardPose = new Pose2d(50,-33,Math.toRadians(180));
            Actions.runBlocking(
                    //drive.actionBuilder(drive.  new Pose2d(50+control.rangeError, 36+control.yawError, Math.toRadians(180)))
                    drive.actionBuilder(new Pose2d(30, -9, Math.toRadians(180)))
                            .splineToLinearHeading(deliverToBoardPose, Math.toRadians(0))
                            .build());
        }
    }
    public void RedRightTeamArtPixelDelivery() {
        if (control.autoPosition == 1) {
            deliverToFloorPose = new Pose2d(12, -33, Math.toRadians(0));
            Actions.runBlocking(
                    drive.actionBuilder(deliverToBoardPose)
                            .splineToLinearHeading(new Pose2d(27,-33, Math.toRadians(0)), Math.toRadians(0))
                            .setTangent(Math.toRadians(180))
                            .splineToLinearHeading(new Pose2d(0,-33, Math.toRadians(0)), Math.toRadians(0))
                            .splineToLinearHeading(deliverToFloorPose, Math.toRadians(0))
                            .build());
        } else if (control.autoPosition == 3) {
            deliverToFloorPose = new Pose2d(12, -36, Math.toRadians(180));
            Actions.runBlocking(
                    drive.actionBuilder(deliverToBoardPose)
                            .splineToLinearHeading (deliverToFloorPose, Math.toRadians(180))
                            .build());
        } else {
            deliverToFloorPose = new Pose2d(12, -36, Math.toRadians(270));
            Actions.runBlocking(
                    drive.actionBuilder(deliverToBoardPose)
                            .splineToLinearHeading(new Pose2d(12, -30, Math.toRadians(-90)), Math.toRadians(180))
                            .splineToLinearHeading(deliverToFloorPose, Math.toRadians(270))
                            .build());
        }
    }
}