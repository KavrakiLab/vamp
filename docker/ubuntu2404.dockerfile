FROM ubuntu:noble

ENV DEBIAN_FRONTEND=noninteractive
RUN apt-get update && \
    apt-get install -y git cmake clang libstdc++6 python3 python3-dev python3-pip libeigen3-dev
COPY . vamp
WORKDIR vamp
RUN pip install --break-system-packages -v .
