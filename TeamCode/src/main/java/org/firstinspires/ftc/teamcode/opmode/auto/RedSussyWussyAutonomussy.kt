package org.firstinspires.ftc.teamcode.opmode.auto

import com.acmerobotics.dashboard.FtcDashboard
import com.amarcolini.joos.command.Command
import com.amarcolini.joos.command.CommandOpMode
import com.amarcolini.joos.command.SequentialCommand
import com.amarcolini.joos.command.WaitCommand
import com.amarcolini.joos.dashboard.JoosConfig
import com.amarcolini.joos.geometry.Pose2d
import com.amarcolini.joos.util.deg
import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import org.firstinspires.ftc.teamcode.CSRobot
import org.firstinspires.ftc.teamcode.Intake
import org.firstinspires.ftc.teamcode.tile
import org.firstinspires.ftc.teamcode.vision.PropPipeline
import org.openftc.easyopencv.OpenCvCamera
import org.openftc.easyopencv.OpenCvCameraRotation

@Autonomous
@JoosConfig
class RedSussyWussyAutonomussy : CommandOpMode() {
    private val robot by robot<CSRobot>()
    private val pipeline = PropPipeline(false)

    companion object {
        var startPose = Pose2d(16.0, -3 * tile + 9.0, (90).deg)
        var leftPlopPose = Pose2d(10.0, -35.0, (-180).deg)
        var leftPlacePose = Pose2d(50.0, -30.0, 0.deg)
        var centerPlopPose = Pose2d(16.0, -37.0, (90).deg)
        var centerPlacePose = Pose2d(50.0, -36.0, 0.deg)
        var rightPlopPose = Pose2d(26.0, -46.0, (90).deg)
        var rightPlacePose = Pose2d(50.0, -45.0, 0.deg)
        var parkPose = Pose2d(52.0, -64.0, 0.deg)

        var exitTangent = (-120).deg
        var exitPose = Pose2d(18.0, -60.0, 0.deg)
        var stackPose = Pose2d(-61.0, -36.0, 0.deg)
        var crossPose = Pose2d(-36.0, -60.0, 0.deg)
        var stackPlacePose = Pose2d(47.0, -45.0, 0.deg)

        var stackHigh = 0.5
    }

    override fun preInit() {
        robot.outtake.init()
        robot.pixelPlopper.prime()
        robot.webcam.openCameraDeviceAsync(object : OpenCvCamera.AsyncCameraOpenListener {
            override fun onOpened() {
                robot.webcam.startStreaming(
                    640,
                    480,
                    OpenCvCameraRotation.UPRIGHT
                )
                robot.webcam.setPipeline(pipeline)
                FtcDashboard.getInstance().startCameraStream(robot.webcam, 0.0)

                schedule(true) {
                    telem.addData("prop location", pipeline.lastKnownLocation)
                }
            }

            override fun onError(errorCode: Int) {
                telem.addLine("Camera failed to open!!").setRetained(true)
                telem.update()
                requestOpModeStop()
            }

        })
    }

    override fun preStart() {
        cancelAll()
        robot.outtake.ready()
        robot.drive.poseEstimate = startPose

        val (plopPose, placePose) = when (pipeline.lastKnownLocation) {
            PropPipeline.PropLocation.Left -> leftPlopPose to leftPlacePose
            PropPipeline.PropLocation.Center -> centerPlopPose to centerPlacePose
            PropPipeline.PropLocation.Right -> rightPlopPose to rightPlacePose
        }
        val purplePlopCommand = robot.drive.pathBuilder(startPose)
            .lineToSplineHeading(plopPose)
            .then(robot.pixelPlopper.plop())
            .build()
        val yellowPlaceCommand = robot.drive.pathBuilder(plopPose)
            .back(3.0)
            .lineToSplineHeading(placePose)
            .wait(0.1)
            .forward(3.0).withTimeout(1.0)
            .then(robot.outtake.extend())
            .wait(2.0)
            .then {
                robot.outtake.releaseRight()
                robot.outtake.releaseLeft()
            }
            .wait(1.0)
            .then(robot.outtake.reset())
            .back(3.0)
            .build()

        val cycleCommand = robot.drive.pathBuilder(placePose)
            .setTangent(exitTangent)
            .splineToSplineHeading(exitPose, exitPose.heading)
            .splineToConstantHeading(crossPose.vec(), crossPose.heading)
            .splineToSplineHeading(stackPose, stackPose.heading)
            .and(
                WaitCommand(3.0) then robot.intake.waitForServoPosition(RedCycleAuto.stackHigh)
                    .then {
                        robot.intake.servoState = Intake.ServoState.STACK
                        robot.intake.motorState = Intake.MotorState.ACTIVE
                    }
            )
            .then(Command.emptyCommand().waitUntil {
                robot.intake.numPixels == 2
            }.withTimeout(3.0))
            .splineToSplineHeading(Pose2d(crossPose.vec(), exitPose.heading), crossPose.heading)
            .splineToConstantHeading(exitPose.vec(), -crossPose.heading)
            .splineToSplineHeading(stackPlacePose, stackPlacePose.heading)
            .wait(0.1)
            .then(robot.verticalExtension.goToPosition(200.0) then robot.outtake.extend())
            .wait(1.0)
            .then {
                robot.outtake.releaseRight()
                robot.outtake.releaseLeft()
            }
            .wait(1.0)
            .then(robot.outtake.reset() then robot.verticalExtension.goToPosition(0.0))
            .build()

        val parkCommand = robot.drive.pathBuilder(leftPlacePose - Pose2d(3.0))
            .lineToSplineHeading(parkPose)
            .build()

        SequentialCommand(
            true,
            purplePlopCommand,
            yellowPlaceCommand,
            cycleCommand,
            parkCommand
        ).thenStopOpMode().schedule()
    }
}