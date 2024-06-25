FROM ubuntu:jammy

ENV DEBIAN_FRONTEND=noninteractive

RUN apt-get update && \
    apt-get install -y git wget && \
    mkdir -p ~/miniconda3 && \
    wget https://repo.anaconda.com/miniconda/Miniconda3-latest-Linux-x86_64.sh -O ~/miniconda3/miniconda.sh && \
    bash ~/miniconda3/miniconda.sh -b -u -p ~/miniconda3 && \
    rm -rf ~/miniconda3/miniconda.sh

COPY environment.yaml .
RUN ~/miniconda3/bin/conda env create -f environment.yaml

COPY . vamp
WORKDIR vamp

SHELL ["/root/miniconda3/bin/conda", "run", "-n", "vamp", "/bin/bash", "-c"]
RUN pip install -v .
