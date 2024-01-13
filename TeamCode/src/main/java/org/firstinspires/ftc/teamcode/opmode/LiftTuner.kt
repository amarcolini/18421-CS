package org.firstinspires.ftc.teamcode.opmode

import com.amarcolini.joos.command.Command
import com.amarcolini.joos.command.CommandOpMode
import com.amarcolini.joos.dashboard.JoosConfig
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.teamcode.CSRobot

@JoosConfig
@TeleOp(group = "lol")
class LiftTuner : CommandOpMode() {
    private val robot by robot<CSRobot>()

    companion object {
        var targetPosition = 0.0
    }

    override fun preInit() {
        robot.verticalExtension.positionControlEnabled = true

        robot.verticalExtension.initializeEncoders().then(Command.of {
            robot.verticalExtension.setTargetPosition(targetPosition)
            telem.addLine("initialization complete!")
        }.runForever().requires(robot.verticalExtension)).schedule()

        schedule(true) {
            telem.addData("bottomSensor", robot.verticalExtension.bottomSensor.state)
        }

        map(gamepad.p1.a0::isJustActivated, ::stopOpMode)
    }
}