# Use Ubuntu 22.04 as the base image
FROM nvidia/cuda:12.4.0-devel-ubuntu22.04
# Set environment variables to avoid interactive prompts during package installation
ENV DEBIAN_FRONTEND=noninteractive

# Update the package list and install necessary tools
RUN apt-get update && \
    apt-get install -y \
    wget \
    unzip \
    dpkg \
    apt-utils \
    sudo \
    git \
    cmake \
    build-essential \
    clang \
    libstdc++6 \
    libeigen3-dev \
    libpoco-dev \
    libboost-all-dev \
    libusb-1.0-0-dev \
    libfmt-dev \
    lsb-release \
    curl \
    xvfb \
    x11-apps \
    libopencv-dev \
    ninja-build \
    vim \
    libfastcdr-dev \
    libfastrtps-dev \
    pkg-config \
    libgtk-3-dev \
    libavcodec-dev \
    libavformat-dev \
    libswscale-dev \
    libv4l-dev \
    libxvidcore-dev \
    libx264-dev \
    libjpeg-dev \
    libpng-dev \
    libtiff-dev \
    gfortran \
    openexr \
    libatlas-base-dev \
    libtbb2 \
    libtbb-dev \
    software-properties-common \
    python3 \
    python3-pip \
    python3-dev
RUN rm -rf /var/lib/apt/lists/*

WORKDIR /workspace

RUN pip install numpy \
    urchin \
    setuptools \
    pybind11 \
    pybind11-stubgen \
    fire \
    pybullet \
    pyyaml \
    tqdm \
    pandas \
    tabulate \
    xmltodict \
    pillow \
    typing_extensions

WORKDIR /workspace


CMD ["bash"]
