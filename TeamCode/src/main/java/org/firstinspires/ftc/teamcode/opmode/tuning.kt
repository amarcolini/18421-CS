package org.firstinspires.ftc.teamcode.opmode

import com.amarcolini.joos.command.Command
import com.amarcolini.joos.command.Command.Companion.emptyCommand
import com.amarcolini.joos.command.CommandOpMode
import com.amarcolini.joos.command.CommandScheduler
import com.amarcolini.joos.command.InstantCommand
import com.amarcolini.joos.command.SequentialCommand
import com.amarcolini.joos.command.TimeCommand
import com.amarcolini.joos.command.WaitCommand
import com.amarcolini.joos.dashboard.JoosConfig
import com.amarcolini.joos.drive.DriveSignal
import com.amarcolini.joos.extensions.*
import com.amarcolini.joos.gamepad.GamepadEx
import com.amarcolini.joos.geometry.Angle
import com.amarcolini.joos.geometry.Pose2d
import com.amarcolini.joos.geometry.Vector2d
import com.amarcolini.joos.hardware.Motor
import com.amarcolini.joos.util.*
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.hardware.IMU
import org.firstinspires.ftc.teamcode.CSRobot
import org.firstinspires.ftc.teamcode.Drivetrain


@TeleOp(group = "Tuning")
class ForwardPushTest : CommandOpMode() {
    private val robot by robot<CSRobot>()

    //    private fun getDistance() = robot.drive.getWheelPositions().average()
    private fun getTicks() = robot.drive.parallelEncoders.map { it.position }.average()

    override fun preInit() {
        val initTicks = getTicks()
        val distancePerTick = robot.drive.parallelEncoders[0].distancePerTick

        robot.drive.motors.zeroPowerBehavior = Motor.ZeroPowerBehavior.FLOAT
        robot.drive.motors.setPower(0.0)

        schedule(true) {
            val ticks = getTicks() - initTicks
            telem.addData("Encoder Ticks", ticks)
            telem.addData("Estimated Distance", ticks * distancePerTick)
        }
    }
}

@TeleOp(group = "Tuning")
class LateralPushTest : CommandOpMode() {
    private val robot by robot<CSRobot>()

    private fun getTicks() = robot.drive.motors.getPositions().let {
        0.25 * (it[0] + it[1] + it[2] + it[3])
    }

    override fun preInit() {
        val initTicks = getTicks()
        val distancePerTick = robot.drive.encoders[0].distancePerTick

        robot.drive.motors.zeroPowerBehavior = Motor.ZeroPowerBehavior.FLOAT
        robot.drive.motors.setPower(0.0)

        schedule(true) {
            val ticks = getTicks() - initTicks
            telem.addData("Encoder Ticks", ticks)
            telem.addData("Estimated Distance", ticks * distancePerTick)
        }
    }
}

@TeleOp(group = "Tuning")
class LocalizationTest : CommandOpMode() {
    private val robot by robot<CSRobot>()

    override fun preInit() {
        robot.drive.localizer.poseEstimate = Pose2d()

        schedule(true) {
            val leftStick = gamepad.p1.getLeftStick()
            val rightStick = gamepad.p1.getRightStick()
            robot.drive.setDrivePower(
                Pose2d(
                    -leftStick.y,
                    -leftStick.x,
                    -rightStick.x.rad
                )
            )

            telem.drawRobot(robot.drive.poseEstimate, "blue")
        }
    }
}

@TeleOp(group = "Tuning")
@JoosConfig
class AngularRampLogger : CommandOpMode() {
    private val robot by robot<CSRobot>()

    private val headingSensor by getHardware<IMU>("imu").map {
        getAngleSensor(
            RevHubOrientationOnRobot.LogoFacingDirection.RIGHT,
            RevHubOrientationOnRobot.UsbFacingDirection.UP
        )
    }

    companion object {
        var totalTime = 5.0
    }

    override fun preInit() {
        val angVels = ArrayList<Double>()
        val leftVels = ArrayList<Double>()
        val rightVels = ArrayList<Double>()
        val perpVels = ArrayList<Double>()

        schedule(
            SequentialCommand(
                true,
                TimeCommand { t, _ ->
                    robot.drive.setDrivePower(Pose2d(heading = (t / totalTime).rad))
                    angVels += headingSensor.getAngularVelocity()?.radians!!
                    leftVels += robot.drive.parallelEncoders[0].distanceVelocity
                    rightVels += robot.drive.parallelEncoders[1].distanceVelocity
                    perpVels += robot.drive.perpEncoder.distanceVelocity
                    t > totalTime
                },
                Command.of {
                    telem.addData("heading", angVels).setRetained(true)
                    telem.addData("left", leftVels).setRetained(true)
                    telem.addData("right", rightVels).setRetained(true)
                    telem.addData("perp", perpVels).setRetained(true)
                    telem.update()

                    data class WheelData(
                        val name: String,
                        val data: List<Double>,
                        val phi: Angle,
                    )
                    listOf(
                        WheelData("left", leftVels, Drivetrain.leftPose.heading),
                        WheelData("right", rightVels, Drivetrain.rightPose.heading),
                        WheelData("perp", perpVels, Drivetrain.perpPose.heading)
                    ).forEach { (name, data, phi) ->
                        val solution = doLinearRegressionNoIntercept(angVels, data)
                        val pos = Vector2d(solution * sin(phi), -solution * cos(phi))
                        telem.addData("$name solution", solution).setRetained(true)
                        telem.addData("$name pos", pos).setRetained(true)
                    }
                }
            ).onEnd {
                robot.drive.setDrivePower(Pose2d())
                telem.addLine("finished!").setRetained(true)
            }
        )
    }

    override fun postStop() {
        CommandScheduler.reset()
    }
}

@TeleOp(group = "Tuning")
@JoosConfig
class ManualFeedforwardTuner : CommandOpMode() {
    private val robot by robot<CSRobot>()

    companion object {
        var distance = 24.0
    }

    override fun preInit() {
        val forwardTrajectory = robot.drive.trajectoryBuilder()
            .forward(ManualFeedbackTuner.distance)
            .build()
        val backwardTrajectory =
            robot.drive.trajectoryBuilder(forwardTrajectory.end())
                .back(distance)
                .build()

        var targetVelocity = 0.0

        val tuningCommand = SequentialCommand(
            true,
            TimeCommand { t, _ ->
                val vel = forwardTrajectory.velocity(t)
                targetVelocity = vel.x
                robot.drive.setDriveSignal(DriveSignal(vel, forwardTrajectory.acceleration(t)))
                t >= forwardTrajectory.duration()
            }.onEnd { robot.drive.setDriveSignal(DriveSignal()) },
            WaitCommand(0.5),
            TimeCommand { t, _ ->
                val vel = backwardTrajectory.velocity(t)
                targetVelocity = vel.x
                robot.drive.setDriveSignal(DriveSignal(vel, backwardTrajectory.acceleration(t)))
                t >= backwardTrajectory.duration()
            }.onEnd { robot.drive.setDriveSignal(DriveSignal()) },
            WaitCommand(0.5)
        ).repeatForever().requires(robot.drive)

        val resettingCommand = Command.of {
            val leftStick = gamepad.p1.getLeftStick()
            val rightStick = gamepad.p1.getRightStick()
            robot.drive.setDrivePower(
                Pose2d(
                    -leftStick.y,
                    -leftStick.x,
                    -rightStick.x.rad
                )
            )
        }.runForever().requires(robot.drive)

        val loggingCommand = InstantCommand {
            val actualVelocity = (robot.drive.poseVelocity ?: return@InstantCommand).x
            telem.addData("targetVelocity", targetVelocity)
            telem.addData("actualVelocity", actualVelocity)
        }.repeatForever()

        schedule(tuningCommand, loggingCommand)
        map(gamepad.p1 { (y or triangle)::justActivated }, resettingCommand)
        map(gamepad.p1 { (b or circle)::justActivated }, tuningCommand)
    }
}

@TeleOp(group = "Tuning")
@JoosConfig
class TurnTest : CommandOpMode() {
    private val robot by robot<CSRobot>()

    companion object {
        var angle = 90.deg
    }

    override fun preInit() {
        val trajectory = robot.drive.trajectoryBuilder()
            .turn(angle)
            .build()

        TimeCommand { t, _ ->
            robot.drive.setDriveSignal(
                DriveSignal(
                    trajectory.velocity(t),
                    trajectory.acceleration(t)
                )
            )
            t >= trajectory.duration()
        }.onEnd { robot.drive.setDriveSignal(DriveSignal()) }
            .thenStopOpMode()
            .schedule()
    }
}

@TeleOp
@JoosConfig
class ManualFeedbackTuner : CommandOpMode() {
    private val robot by robot<CSRobot>()

    companion object {
        var distance = 24.0
    }

    override fun preInit() {
        robot.drive.poseEstimate = Pose2d()
        val forwardTrajectory = robot.drive.trajectoryBuilder(Pose2d())
            .lineToSplineHeading(Pose2d(distance, 0.0, 180.deg))
            .build()
        val backwardTrajectory =
            robot.drive.trajectoryBuilder(forwardTrajectory.end())
                .lineToSplineHeading(Pose2d())
                .build()

        SequentialCommand(
            false,
            emptyCommand(),
            robot.drive.followTrajectory(forwardTrajectory),
            WaitCommand(0.5),
            robot.drive.followTrajectory(backwardTrajectory),
            WaitCommand(0.5)
        ).repeatForever().schedule()
    }
}

@TeleOp(group = "Tuning")
class MotorDirectionDebugger : CommandOpMode() {
    private val robot by robot<CSRobot>()

    override fun preInit() {
        val buttonMap: List<GamepadEx.() -> Boolean> =
            listOf(
                { (x or square).isActive },
                { (a or cross).isActive },
                { (b or circle).isActive },
                { (y or triangle).isActive }
            )

        schedule(true) {
            robot.drive.motors.zip(buttonMap).forEach { (motor, condition) ->
                motor.power = if (condition(gamepad.p1)) 1.0 else 0.0
            }
        }
    }
}