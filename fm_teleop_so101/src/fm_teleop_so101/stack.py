"""One SO-101 pair, one worker thread, and the mode it should be in.

Requests never touch the hardware. They set the wanted state under a lock and
ask LeRobot's loop to exit early; the worker thread alone reads and writes the
buses, so two requests at once cannot interleave packets on a serial line.
"""

from __future__ import annotations

import logging
import re
import threading
import time
from dataclasses import dataclass
from pathlib import Path

from lerobot.datasets import (
    LeRobotDataset,
    aggregate_pipeline_dataset_features,
    create_initial_features,
)
from lerobot.processor import make_default_processors
from lerobot.scripts.lerobot_record import record_loop
from lerobot.utils.feature_utils import combine_feature_dicts

log = logging.getLogger(__name__)

#: A dataset name becomes a directory, so it may not carry a path.
DATASET_NAME = re.compile(r"^[A-Za-z0-9][A-Za-z0-9_.-]{0,63}$")

#: How often idle mode reads the joints for telemetry. Torque is off, so this
#: is the only bus traffic while nothing runs.
IDLE_POLL_S = 0.1


class Refused(Exception):
    """A request the stack will not carry out, phrased for the operator."""


@dataclass(frozen=True)
class Take:
    dataset: str
    task: str
    started_at: float


class Stack:
    def __init__(self, robot, teleop, fps: int, root: Path, owner: str) -> None:
        self.robot = robot
        self.teleop = teleop
        self.fps = fps
        self.root = root
        self.owner = owner
        self.processors = make_default_processors()
        teleop_proc, _, obs_proc = self.processors
        use_videos = bool(robot.cameras)
        self.features = combine_feature_dicts(
            aggregate_pipeline_dataset_features(
                pipeline=teleop_proc,
                initial_features=create_initial_features(action=robot.action_features),
                use_videos=use_videos,
            ),
            aggregate_pipeline_dataset_features(
                pipeline=obs_proc,
                initial_features=create_initial_features(observation=robot.observation_features),
                use_videos=use_videos,
            ),
        )
        self._lock = threading.Lock()
        self._events = {"exit_early": False, "stop_recording": False, "rerecord_episode": False}
        self._wanted = "idle"
        self._take: Take | None = None
        self._mode = "idle"
        self._joints: dict[str, float] = {}
        self._last_episode: dict | None = None
        # record_loop reads the follower through get_observation; keep the last
        # joint reading so status can report it without a second bus read.
        observe = robot.get_observation

        def observe_and_keep():
            obs = observe()
            self._joints = {k.removesuffix(".pos"): float(v) for k, v in obs.items() if k.endswith(".pos")}
            return obs

        robot.get_observation = observe_and_keep

    # -- requests ---------------------------------------------------------

    def status(self) -> dict:
        with self._lock:
            take = self._take
            return {
                "mode": self._mode,
                "wanted": self._wanted,
                "joints": dict(self._joints),
                "recording": None
                if take is None
                else {"dataset": take.dataset, "task": take.task, "started_at": take.started_at},
                "last_episode": self._last_episode,
                "fps": self.fps,
                "owner": self.owner,
                "root": str(self.root),
                "cameras": sorted(self.robot.cameras),
            }

    def set_mode(self, mode: str) -> None:
        if mode not in ("idle", "teleop"):
            raise Refused(f"unknown mode {mode!r}; the modes are idle and teleop")
        with self._lock:
            if mode == "idle" and self._take is not None:
                raise Refused(f"recording into {self._take.dataset}; stop the take first")
            self._wanted = mode
            self._events["exit_early"] = True

    def start_take(self, dataset: str, task: str) -> None:
        if not DATASET_NAME.match(dataset or "") or ".." in dataset:
            raise Refused(f"dataset name {dataset!r} must be letters, digits, '_', '.', '-' (max 64)")
        if not task.strip():
            raise Refused("a take needs a task sentence, the instruction the episode demonstrates")
        with self._lock:
            if self._wanted != "teleop":
                raise Refused("switch to teleop first; recording never starts motion by itself")
            if self._take is not None:
                raise Refused(f"already recording into {self._take.dataset}")
            self._take = Take(dataset, task.strip(), time.time())
            self._events["exit_early"] = True

    def stop_take(self) -> None:
        with self._lock:
            if self._take is None:
                raise Refused("no take is recording")
            self._take = None
            self._events["exit_early"] = True

    def stop(self) -> None:
        """End any take (it is saved), then torque off."""
        with self._lock:
            self._take = None
            self._wanted = "idle"
            self._events["exit_early"] = True

    # -- worker -----------------------------------------------------------

    def run(self, stopping: threading.Event) -> None:
        torque_on = True  # LeRobot's connect() leaves the follower enabled
        while not stopping.is_set():
            with self._lock:
                wanted, take = self._wanted, self._take
                self._events["exit_early"] = False
            if wanted == "idle":
                if torque_on:
                    self.robot.bus.disable_torque()
                    torque_on = False
                self._mode = "idle"
                self.robot.get_observation()
                stopping.wait(IDLE_POLL_S)
                continue
            if not torque_on:
                self.robot.bus.enable_torque()
                torque_on = True
            self._mode = "recording" if take else "teleop"
            dataset = self._open(take.dataset) if take else None
            teleop_proc, robot_proc, obs_proc = self.processors
            try:
                record_loop(
                    robot=self.robot,
                    events=self._events,
                    fps=self.fps,
                    teleop_action_processor=teleop_proc,
                    robot_action_processor=robot_proc,
                    robot_observation_processor=obs_proc,
                    dataset=dataset,
                    teleop=self.teleop,
                    control_time_s=float("inf"),
                    single_task=take.task if take else None,
                )
            finally:
                if dataset is not None:
                    self._close(dataset, take)
        if torque_on:
            self.robot.bus.disable_torque()

    def _open(self, name: str) -> LeRobotDataset:
        root = self.root / name
        repo_id = f"{self.owner}/{name}"
        cameras = len(self.robot.cameras)
        if (root / "meta" / "info.json").exists():
            return LeRobotDataset.resume(repo_id, root=root, image_writer_threads=4 * cameras)
        return LeRobotDataset.create(
            repo_id,
            self.fps,
            root=root,
            robot_type=self.robot.name,
            features=self.features,
            use_videos=bool(cameras),
            image_writer_threads=4 * cameras,
        )

    def _close(self, dataset: LeRobotDataset, take: Take) -> None:
        if dataset.has_pending_frames():
            dataset.save_episode()
            episode = dataset.meta.total_episodes - 1
            self._last_episode = {"dataset": take.dataset, "episode": episode, "task": take.task}
            log.info("saved %s episode %d", take.dataset, episode)
        else:
            dataset.clear_episode_buffer()
            log.info("take in %s ended before its first frame; nothing saved", take.dataset)
        dataset.finalize()
