FROM python:3.10-slim

ARG PROTOCOL_WHEEL
ARG APP_WHEEL
COPY wheels/ /tmp/wheels/
COPY requirements.lock /opt/elesim/requirements.lock
RUN pip install --no-cache-dir -r /opt/elesim/requirements.lock && \
    pip install --no-cache-dir --no-deps "/tmp/wheels/${PROTOCOL_WHEEL}" "/tmp/wheels/${APP_WHEEL}"

EXPOSE 5558
ENTRYPOINT ["elesim-router"]
