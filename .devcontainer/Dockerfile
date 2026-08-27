ARG BUILD_BASE_VERSION=2025.04.0


FROM ghcr.io/esphome/docker-base:debian-${BUILD_BASE_VERSION} AS base

RUN git config --system --add safe.directory "*"

RUN apt update \
    && apt install -y \
      protobuf-compiler \
      git \
      cmake \
      nodejs \
      python3

# ── Emscripten SDK 3.1.6 for chime WASM (esphome/components/chime/core → wasm/chime.js)
# Pinned to 3.1.6 per Makefile; installed system-wide so `make -C esphome/components/chime/wasm` works without download.
ENV EMSDK_VERSION=3.1.6
ENV EMSDK=/opt/emsdk
ENV EMSDK_DIR=/opt/emsdk
RUN git clone https://github.com/emscripten-core/emsdk.git $EMSDK \
    && $EMSDK/emsdk install $EMSDK_VERSION \
    && $EMSDK/emsdk activate $EMSDK_VERSION \
    && rm -rf $EMSDK/upstream/emscripten/cache/*
# Make emcc available without sourcing emsdk_env.sh
ENV PATH="$EMSDK:$EMSDK/upstream/emscripten:$PATH"

RUN pip install uv

RUN useradd esphome -m

USER esphome
ENV VIRTUAL_ENV=/home/esphome/.local/esphome-venv
RUN uv venv $VIRTUAL_ENV
ENV PATH="$VIRTUAL_ENV/bin:$PATH"
# Override this set to true in the docker-base image
ENV UV_SYSTEM_PYTHON=false

WORKDIR /tmp

COPY requirements.txt ./
RUN uv pip install -r requirements.txt
COPY requirements_dev.txt requirements_test.txt ./
RUN uv pip install -r requirements_dev.txt -r requirements_test.txt

RUN \
    platformio settings set enable_telemetry No \
    && platformio settings set check_platformio_interval 1000000

#COPY script/platformio_install_deps.py platformio.ini ./
#RUN ./platformio_install_deps.py platformio.ini --libraries --platforms --tools

WORKDIR /workspaces
