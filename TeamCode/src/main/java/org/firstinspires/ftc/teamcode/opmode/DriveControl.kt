package org.firstinspires.ftc.teamcode.opmode

import android.graphics.Color
import com.amarcolini.joos.command.Command
import com.amarcolini.joos.command.CommandOpMode
import com.amarcolini.joos.control.PIDCoefficients
import com.amarcolini.joos.control.PIDController
import com.amarcolini.joos.dashboard.JoosConfig
import com.amarcolini.joos.gamepad.Button
import com.amarcolini.joos.geometry.Pose2d
import com.amarcolini.joos.util.deg
import com.amarcolini.joos.util.rad
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.robotcore.external.Telemetry
import org.firstinspires.ftc.teamcode.CSRobot
import org.firstinspires.ftc.teamcode.Intake
import kotlin.math.PI
import kotlin.math.abs
import kotlin.math.pow

@JoosConfig
@TeleOp(name = "DriveControl", group = "Official")
class DriveControl : CommandOpMode() {
    private val robot by robot<CSRobot>()
    override fun preInit() {
        robot.pixelPlopper.prime()
        robot.outtake.reset().schedule()
//        robot.verticalExtension.initializeEncoders().schedule()
        telem.setDisplayFormat(Telemetry.DisplayFormat.HTML)
    }

    companion object {
        val headingCoeffs = PIDCoefficients(6.0, 0.0, 0.2)
    }

    private var leftPixelColor: Intake.PixelColor? = null
    private var rightPixelColor: Intake.PixelColor? = null
    private var canTransfer = true
    private var hasIntaked = false
    private val headingController = PIDController(headingCoeffs)
    private var headingLock = false

    init {
        headingController.setInputBounds(-PI, PI)
        headingController.setOutputBounds(-0.5, 0.5)
    }

    override fun preStart() {
        headingController.reset()
        schedule(true) {
            val leftStick = gamepad.p1.getLeftStick()
            val rightStick = gamepad.p1.getRightStick()

            val isSlow = robot.outtake.isExtended && robot.frontDistanceSensor.getDistance() < 24.0
            val drivePose = Pose2d(
                -leftStick.y.pow(3),
                -leftStick.x.pow(3),
                -rightStick.x.pow(3).rad
            )
            val initialPower = if (!isSlow) drivePose
            else drivePose.run {
                Pose2d(
                    x * 0.2,
                    y * 0.5,
                    heading * 0.3
                )
            }
            robot.drive.setDrivePower(
                if (headingLock) initialPower.copy(
                    heading = headingController.update(robot.drive.poseEstimate.heading.radians).rad
                ) else initialPower
            )

            val leftTrigger = gamepad.p1.left_trigger.value
            val rightTrigger = gamepad.p1.right_trigger.value
            var power = (rightTrigger - leftTrigger).pow(3).toDouble() * 1.0 + 0.1
            canTransfer = !robot.verticalExtension.bottomSensor.state
            if (canTransfer && power < 0) power = 0.0
            robot.verticalExtension.motors.setPower(
                power
            )

            val leftColor = when (leftPixelColor) {
                Intake.PixelColor.WHITE -> Color.WHITE
                Intake.PixelColor.GREEN -> Color.GREEN
                Intake.PixelColor.PURPLE -> Color.rgb(255, 0, 255)
                Intake.PixelColor.YELLOW -> Color.YELLOW
                null -> Color.BLACK
            }
            val rightColor = when (rightPixelColor) {
                Intake.PixelColor.WHITE -> Color.WHITE
                Intake.PixelColor.GREEN -> Color.GREEN
                Intake.PixelColor.PURPLE -> Color.rgb(255, 0, 255)
                Intake.PixelColor.YELLOW -> Color.YELLOW
                null -> Color.BLACK
            }
            telem.addLine("<font color=\"${leftColor}\">████</font>    <font color=\"${rightColor}\">████</font>")
        }

        map(
            {
                abs(gamepad.p1.getRightStick().x) <= 0.1
            },
            Command.emptyCommand()
                .waitUntil { robot.drive.poseVelocity?.heading?.let { it.abs().degrees < 10.0 } != false }
                .then {
                    headingController.targetPosition = robot.drive.poseEstimate.heading.radians
                    headingController.reset()
                    headingLock = true
                }
                .waitUntil { abs(gamepad.p1.getRightStick().x) > 0.1 }
                .then { headingLock = false }
        )

        map(gamepad.p1.y0::isJustActivated, Command.select(robot.outtake) {
            if (robot.outtake.isExtended) robot.outtake.resetArm()
            else robot.outtake.extend()
        })

        map(gamepad.p1.left_bumper::isJustActivated, Command.of {
            leftPixelColor = null
            robot.outtake.releaseLeft()
        }.requires(robot.intake))
        map(gamepad.p1.right_bumper::isJustActivated, Command.of {
            rightPixelColor = null
            robot.outtake.releaseRight()
        }.requires(robot.outtake))
        map(
            { robot.intake.numPixels == 2 && hasIntaked },
            robot.intake.waitForServoState(Intake.ServoState.UP)
                .then(robot.intake.stop()).onEnd {
                    val (left, right) = robot.intake.getPixelColors()
                    if (hasIntaked) {
                        leftPixelColor = left
                        rightPixelColor = right
                        hasIntaked = false
                    }
                }
        )

        map(gamepad.p1.b0::isJustActivated, Command.select {
            if (!canTransfer) return@select Command.emptyCommand()
            val (left, right) = robot.intake.getPixelColors()
            if (hasIntaked) {
                leftPixelColor = left
                rightPixelColor = right
                hasIntaked = false
            }
            robot.transfer()
        }.requires(robot.intake, robot.outtake, robot.verticalExtension).setInterruptable(false))

        map(gamepad.p1.a0::isJustActivated, Command.select(robot.intake) {
            when (robot.intake.motorState) {
                Intake.MotorState.ACTIVE -> robot.intake.stop()
                else -> robot.intake.intake()
            }
        }.onEnd { hasIntaked = true })
        map(gamepad.p1.x0::isJustActivated, Command.select(robot.intake) {
            when (robot.intake.servoState) {
                Intake.ServoState.UP -> robot.intake.waitForServoState(Intake.ServoState.DOWN)
                else -> robot.intake.waitForServoState(Intake.ServoState.UP)
            }.onInit {
                robot.intake.motorState = Intake.MotorState.STOPPED
            }
        })
        map(gamepad.p1.dpad_left::isJustActivated, robot.intake.reverse().onInit {
            leftPixelColor = null
            rightPixelColor = null
        })

        map(gamepad.p1.dpad_up::isJustActivated, robot.droneLauncher::launch)
        map(gamepad.p1.dpad_down::isJustActivated, robot.droneLauncher::reset)

//        map(gamepad.p1.dpad_left::isJustActivated) {
//            if (robot.pixelPlopper.isPrimed) robot.pixelPlopper.open()
//            else robot.pixelPlopper.prime()
//        }
        map(gamepad.p1.dpad_right::isJustActivated, robot.outtake.prepareClimb())
    }
}