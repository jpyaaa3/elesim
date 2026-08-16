from __future__ import annotations

import time

import numpy as np

from elesim_sim.media import MediaWorkerClient, VideoStreamSpec
from elesim_sim.vision.webrtc import NamedWebRtcVideoSender
from elesim_ui.webrtc import WebRtcVideoReceiver


def test_observer_and_hand_eye_are_independent_real_webrtc_streams() -> None:
    observer = np.zeros((48, 64, 3), dtype=np.uint8)
    observer[:, :, 2] = 220
    hand_eye = np.zeros((48, 64, 3), dtype=np.uint8)
    hand_eye[:, :, 1] = 220
    sender = NamedWebRtcVideoSender(
        {
            "observer": lambda: observer,
            "hand_eye_preview": lambda: hand_eye,
        },
        fps={"observer": 15.0, "hand_eye_preview": 15.0},
    )
    receivers = {
        "observer": WebRtcVideoReceiver(),
        "hand_eye_preview": WebRtcVideoReceiver(),
    }
    all_receivers = list(receivers.values())

    try:
        for stream, receiver in receivers.items():
            offer = receiver.create_offer()
            answer = sender.accept_offer(
                stream,
                offer["sdp"],
                offer["type"],
                None,
                "integration-session",
            )
            receiver.accept_answer(answer["sdp"], answer["type"])

        deadline = time.monotonic() + 8.0
        while (
            any(receiver.latest_bgr is None for receiver in receivers.values())
            and time.monotonic() < deadline
        ):
            time.sleep(0.02)

        observer_frame = receivers["observer"].latest_bgr
        hand_eye_frame = receivers["hand_eye_preview"].latest_bgr
        assert observer_frame is not None
        assert hand_eye_frame is not None
        assert observer_frame.shape == (48, 64, 3)
        assert hand_eye_frame.shape == (48, 64, 3)
        assert float(observer_frame[:, :, 2].mean()) > 150.0
        assert float(observer_frame[:, :, 1].mean()) < 40.0
        assert float(hand_eye_frame[:, :, 1].mean()) > 150.0
        assert float(hand_eye_frame[:, :, 2].mean()) < 40.0

        replacements = {
            "observer": WebRtcVideoReceiver(),
            "hand_eye_preview": WebRtcVideoReceiver(),
        }
        all_receivers.extend(replacements.values())
        for stream, receiver in replacements.items():
            offer = receiver.create_offer()
            answer = sender.accept_offer(
                stream,
                offer["sdp"],
                offer["type"],
                None,
                "integration-session",
            )
            receiver.accept_answer(answer["sdp"], answer["type"])

        deadline = time.monotonic() + 8.0
        while (
            any(receiver.latest_bgr is None for receiver in replacements.values())
            and time.monotonic() < deadline
        ):
            time.sleep(0.02)

        assert all(receiver.latest_bgr is not None for receiver in replacements.values())
        assert all(
            len(sender.senders[stream].peers_by_session["integration-session"]) == 1
            for stream in replacements
        )
    finally:
        for receiver in all_receivers:
            receiver.close()
        sender.close()


def test_private_media_worker_keeps_two_streams_independent() -> None:
    observer = np.zeros((48, 64, 3), dtype=np.uint8)
    observer[:, :, 2] = 220
    hand_eye = np.zeros((48, 64, 3), dtype=np.uint8)
    hand_eye[:, :, 1] = 220
    worker = MediaWorkerClient(
        {
            "observer": VideoStreamSpec("observer", 15.0, 64, 48),
            "hand_eye_preview": VideoStreamSpec("hand_eye_preview", 15.0, 64, 48),
        },
        command_timeout_s=8.0,
    )
    receivers = {
        "observer": WebRtcVideoReceiver(),
        "hand_eye_preview": WebRtcVideoReceiver(),
    }
    try:
        worker.start()
        worker.mailboxes["observer"].publish(observer)
        worker.mailboxes["hand_eye_preview"].publish(hand_eye)
        for stream, receiver in receivers.items():
            offer = receiver.create_offer()
            answer = worker.accept_offer(
                stream,
                offer["sdp"],
                offer["type"],
                None,
                "worker-session",
            )
            receiver.accept_answer(answer["sdp"], answer["type"])

        deadline = time.monotonic() + 8.0
        while (
            any(receiver.latest_bgr is None for receiver in receivers.values())
            and time.monotonic() < deadline
        ):
            time.sleep(0.02)
        observer_frame = receivers["observer"].latest_bgr
        hand_eye_frame = receivers["hand_eye_preview"].latest_bgr
        assert observer_frame is not None
        assert hand_eye_frame is not None
        assert float(observer_frame[:, :, 2].mean()) > 150.0
        assert float(observer_frame[:, :, 1].mean()) < 40.0
        assert float(hand_eye_frame[:, :, 1].mean()) > 150.0
        assert float(hand_eye_frame[:, :, 2].mean()) < 40.0
        worker.close_session("worker-session")
    finally:
        for receiver in receivers.values():
            receiver.close()
        worker.close()
