FROM python:3.10-slim

ARG PROTOCOL_WHEEL
ARG APP_WHEEL
COPY wheels/ /tmp/wheels/
COPY requirements.lock /opt/elesim/requirements.lock
RUN pip install --no-cache-dir -r /opt/elesim/requirements.lock && \
    pip install --no-cache-dir --no-deps "/tmp/wheels/${PROTOCOL_WHEEL}" "/tmp/wheels/${APP_WHEEL}"
COPY model/ /opt/elesim/model/
COPY config/ /opt/elesim/config/
WORKDIR /opt/elesim

ENTRYPOINT ["elesim-simulator", "--config", "/opt/elesim/config/default.yaml", "--runtime-config", "/opt/elesim/config/runtime.yaml", "--model-bundle", "/opt/elesim/model/bundles/default"]
