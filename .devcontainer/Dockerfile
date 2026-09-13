# Linux build environment for the EAE_Firmware project.
# The project is edited on the host machine but compiled and run inside
# this container, which satisfies the "build on Linux" requirement.

FROM ubuntu:24.04

# Avoid interactive prompts from apt during the image build.
ENV DEBIAN_FRONTEND=noninteractive

# build-essential provides g++ and make.
# cmake drives the build. git is needed by CMake to fetch dependencies.
# gdb allows debugging from inside VS Code.
RUN apt-get update && apt-get install -y --no-install-recommends \
        build-essential \
        cmake \
        git \
        gdb \
        ca-certificates \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /workspace
