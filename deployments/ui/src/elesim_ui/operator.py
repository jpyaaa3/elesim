from __future__ import annotations

import threading
import time
import uuid
from typing import Any

from elesim_protocol import (
    EndpointClient,
    EndpointDescriptor,
    SERVICE_CALLS,
    SERVICE_VALUES,
    STATE_CALLS,
    decode_value,
    encode_value,
)


class OperatorClient:
    def __init__(
        self,
        server_endpoint: str,
        *,
        ui_id: str,
        controller_id: str,
        timeout_ms: int = 3000,
    ) -> None:
        self.controller_id = str(controller_id)
        self.timeout_ms = int(timeout_ms)
        self.endpoint = EndpointClient(
            server_endpoint,
            EndpointDescriptor(str(ui_id), "ui", ()),
        )
        self.lock = threading.Lock()

    def request(self, operation: str, name: str = "", *args: Any, **kwargs: Any) -> Any:
        request_id = uuid.uuid4().hex
        payload = {
            "request_id": request_id,
            "operation": str(operation),
            "name": str(name),
            "args": [encode_value(value) for value in args],
            "kwargs": {str(key): encode_value(value) for key, value in kwargs.items()},
        }
        with self.lock:
            self.endpoint.heartbeat()
            self.endpoint.send(
                "operator_intent",
                target_id=self.controller_id,
                payload=payload,
            )
            deadline = time.monotonic() + self.timeout_ms / 1000.0
            while time.monotonic() < deadline:
                for message in self.endpoint.receive(timeout_ms=50):
                    body = message.payload or {}
                    if message.message_type == "operator_result" and body.get("request_id") == request_id:
                        if not bool(body.get("ok", False)):
                            raise RuntimeError(str(body.get("error", "operator request failed")))
                        return decode_value(body.get("result"))
                    if message.message_type == "error" and body.get("reply_to"):
                        raise RuntimeError(str(body.get("reason", "router rejected operator request")))
        raise TimeoutError(f"controller did not answer {operation} {name}")

    def close(self) -> None:
        self.endpoint.close()


class RemotePanelState:
    def __init__(self, client: OperatorClient) -> None:
        object.__setattr__(self, "_client", client)
        object.__setattr__(self, "_cache", {})
        self.sync()

    def sync(self) -> None:
        snapshot = self._client.request("snapshot")
        object.__getattribute__(self, "_cache").update(snapshot)

    def __getattr__(self, name: str) -> Any:
        if name in STATE_CALLS:
            return lambda *args, **kwargs: self._client.request("state_call", name, *args, **kwargs)
        cache = object.__getattribute__(self, "_cache")
        if name in cache:
            return cache[name]
        raise AttributeError(name)

    def __setattr__(self, name: str, value: Any) -> None:
        if name.startswith("_"):
            object.__setattr__(self, name, value)
            return
        object.__getattribute__(self, "_cache")[name] = value
        self._client.request("state_set", name, value=value)


class RemoteControlService:
    def __init__(self, client: OperatorClient, state: RemotePanelState) -> None:
        self.client = client
        self.state = state

    def refresh_host_state(self) -> Any:
        result = self.client.request("service_call", "refresh_host_state")
        self.state.sync()
        return result

    def close(self) -> None:
        self.client.close()

    def __getattr__(self, name: str) -> Any:
        if name in SERVICE_CALLS:
            return lambda *args, **kwargs: self.client.request("service_call", name, *args, **kwargs)
        if name in SERVICE_VALUES:
            return self.client.request("service_get", name)
        raise AttributeError(name)
