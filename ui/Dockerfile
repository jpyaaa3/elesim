FROM python:3.10-slim

ARG PROTOCOL_WHEEL
ARG APP_WHEEL
COPY wheels/ /tmp/wheels/
COPY requirements.lock /opt/elesim/requirements.lock
COPY config/ /opt/elesim/config/
RUN pip install --no-cache-dir -r /opt/elesim/requirements.lock && \
    pip install --no-cache-dir --no-deps "/tmp/wheels/${PROTOCOL_WHEEL}" "/tmp/wheels/${APP_WHEEL}"
WORKDIR /opt/elesim

ENTRYPOINT ["elesim-ui", "--config", "/opt/elesim/config/default.yaml"]
