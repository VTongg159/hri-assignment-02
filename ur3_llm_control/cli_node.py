import argparse
import sys
from pathlib import Path

import rclpy
import yaml
from ament_index_python.packages import get_package_share_directory
from rclpy.node import Node
from rcl_interfaces.msg import Parameter, ParameterType, ParameterValue
from rcl_interfaces.srv import SetParametersAtomically

from .llm_planner import Planner
from .plan_models import PlanStep
from .skill_executor import SkillExecutor, SkillResult
from .student_task import assignment_for_id, student_remainder
from .task_validator import PlanValidationError, validate_plan


RULE = "=" * 60


class TaskCli(Node):
    def __init__(self, mode: str):
        super().__init__("task_cli")
        share = Path(get_package_share_directory("ur3_llm_control"))
        config = yaml.safe_load((share / "config/student_config.yaml").read_text())
        self.student_id = str(config.get("student_id", "FILL_ME"))
        try:
            self.assignment = assignment_for_id(self.student_id)
            self.xx, self.remainder = student_remainder(self.student_id)
        except ValueError:
            self.assignment = {}
            self.get_logger().warning("student_id is not configured; advanced task is unavailable")
        self.planner = Planner(mode, str(share / "prompts/task_planner.txt"), self.assignment)
        self.client = self.create_client(SetParametersAtomically, "/execute_skill")
        if self.assignment:
            print(f"STUDENT ID: {self.student_id}\nXX: {self.xx}\nP: {self.remainder}\n")
            print("PERSONALIZED TASK")
            for zone, obj in self.assignment.items():
                print(f"{zone.replace('_', ' ').title()} <- {obj}")

    def call_skill(self, step: PlanStep) -> SkillResult:
        if not self.client.wait_for_service(timeout_sec=15.0):
            return SkillResult(False, "SERVICE_UNAVAILABLE", "skill server is not ready")
        request = SetParametersAtomically.Request()
        values = (("skill", step.skill), ("object", step.object or ""), ("zone", step.zone or ""))
        request.parameters = [Parameter(name=name, value=ParameterValue(
            type=ParameterType.PARAMETER_STRING, string_value=value)) for name, value in values]
        future = self.client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=300.0)
        if not future.done() or future.result() is None:
            return SkillResult(False, "TIMEOUT", "skill did not finish")
        response = future.result().result
        # The service reason is "STATUS|message" so no arbitrary response data is interpreted.
        status, _, message = response.reason.partition("|")
        return SkillResult(response.successful, status, message)

    def run_command(self, command: str) -> bool:
        print(f"\n{RULE}\nUSER COMMAND\n{RULE}\n{command}")
        try:
            raw = self.planner.plan(command)
            plan = validate_plan(raw)
        except (RuntimeError, PlanValidationError) as exc:
            print(f"\n{RULE}\nPLAN REJECTED\n{RULE}\nreason: {exc}")
            return False
        print(f"\n{RULE}\nLLM PLAN\n{RULE}")
        for number, step in enumerate(plan.steps, 1):
            print(f"{number}. {step.display()}")
        print(f"\n{RULE}\nVALIDATION\n{RULE}\nPlan valid.")
        print(f"\n{RULE}\nEXECUTION\n{RULE}")
        results = SkillExecutor(self.call_skill).execute(plan)
        for step, result in results:
            suffix = f" ({result.message})" if result.message else ""
            print(f"{step.display():35} {result.status}{suffix}")
        success = len(results) == len(plan.steps) and all(result.success for _, result in results)
        print(f"\n{RULE}\n{'TASK SUCCESS' if success else 'TASK FAILED'}\n{RULE}")
        return success


def main():
    parser = argparse.ArgumentParser(description="UR3e natural-language task interface")
    parser.add_argument("command", nargs="*", help="one-shot command; omit for interactive mode")
    parser.add_argument("--mode", choices=("9router", "mock"), default="9router")
    args, ros_args = parser.parse_known_args()
    rclpy.init(args=ros_args)
    node = TaskCli(args.mode)
    try:
        if args.command:
            ok = node.run_command(" ".join(args.command))
            raise SystemExit(0 if ok else 1)
        while rclpy.ok():
            try:
                command = input("\nEnter command (or 'quit'): ").strip()
            except EOFError:
                break
            if command.casefold() in ("quit", "exit"):
                break
            if command:
                node.run_command(command)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
