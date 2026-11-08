# Container that runs every tool in VRHealth_Challenge13/scripts:
#   bag2edex.py, convert_all.py, merge_rig_edex.py, compare_runs.py   (numpy, OpenCV, matplotlib)
#   track_edex_rerun.py, track_bag_rerun.py                                 (+ PyCuVSLAM, Rerun SDK, GPU)
#   db3_to_mcap.sh, all_db3_to_mcap.sh                                      (+ rosbags)
#
# Build (the build context is the scripts folder):
#   cd ~/dotlumen/VRHealth_Challenge13/scripts
#   docker build -f docker/Dockerfile -t vrhealth-cuvslam .
#   # faster build for one GPU only, e.g. RTX 30xx = 86, RTX 40xx = 89:
#   docker build -f docker/Dockerfile --build-arg CUDA_ARCHITECTURES=89 -t vrhealth-cuvslam .
#
# Run: docker/run.sh (GPU, X11 for the Rerun viewer, ~/dotlumen mounted at the same path).
# Host requirements: NVIDIA driver supporting CUDA 12.6 (>= 560) and the NVIDIA Container Toolkit.

ARG CUDA_VERSION=12.6.3
ARG UBUNTU_VERSION=24.04

# =============================================================================
# Stage 1: build cuVSLAM and its Python bindings
# =============================================================================
FROM nvidia/cuda:${CUDA_VERSION}-devel-ubuntu${UBUNTU_VERSION} AS build

ARG DEBIAN_FRONTEND=noninteractive
# cuVSLAM version: the commit used so far (17.0.0+2a0882c). Any branch, tag or commit works.
ARG CUVSLAM_REPO=https://github.com/nvidia-isaac/cuVSLAM.git
ARG CUVSLAM_REF=2a0882c
# "all" works on any GPU but takes long to compile; set the compute capability of your GPU to build faster
ARG CUDA_ARCHITECTURES=all

RUN apt-get update && apt-get install -y --no-install-recommends \
        g++ gcc git git-lfs cmake make patch ca-certificates \
        python3 python3-dev python3-venv python3-pip \
    && rm -rf /var/lib/apt/lists/*

# LFS files (images, test data) are not needed to build the library
RUN GIT_LFS_SKIP_SMUDGE=1 git clone ${CUVSLAM_REPO} /src/cuvslam \
    && cd /src/cuvslam \
    && GIT_LFS_SKIP_SMUDGE=1 git checkout ${CUVSLAM_REF} \
    && git log -1 --format='cuVSLAM %h %ad %s' > /src/cuvslam_version.txt

# Same options as the local build: cuNLS (CUDA >= 12.6) and LMDB on; the Rerun C++ SDK is not needed
# by the Python tools (they use rerun-sdk from pip)
RUN cmake -S /src/cuvslam -B /src/build \
        -DCMAKE_BUILD_TYPE=Release \
        -DUSE_CUDA=ON -DUSE_CUNLS=ON -DUSE_LMDB=ON -DUSE_RERUN=OFF \
        -DCMAKE_CUDA_ARCHITECTURES=${CUDA_ARCHITECTURES} \
    && cmake --build /src/build --target cuvslam --parallel $(nproc)

# Python environment: the bindings copy libcuvslam.so inside the package, so /opt/venv is self-contained
RUN python3 -m venv /opt/venv \
    && /opt/venv/bin/pip install --no-cache-dir --upgrade pip \
    && CUVSLAM_BUILD_DIR=/src/build /opt/venv/bin/pip install --no-cache-dir /src/cuvslam/python

COPY requirements.txt /tmp/requirements.txt
RUN /opt/venv/bin/pip install --no-cache-dir -r /tmp/requirements.txt \
        "rerun-sdk==0.33.1" \
        matplotlib \
        rosbags

# =============================================================================
# Stage 2: runtime
# =============================================================================
FROM nvidia/cuda:${CUDA_VERSION}-runtime-ubuntu${UBUNTU_VERSION}

ARG DEBIAN_FRONTEND=noninteractive

# python3 for the venv; libgl/libx*/vulkan for the Rerun viewer (X11); poppler-utils/less for convenience
RUN apt-get update && apt-get install -y --no-install-recommends \
        python3 python3-venv \
        libgl1 libegl1 libglib2.0-0 libvulkan1 mesa-vulkan-drivers \
        libx11-6 libx11-xcb1 libxcursor1 libxi6 libxrandr2 libxkbcommon-x11-0 libxcb1 \
        libfontconfig1 libdbus-1-3 \
        less ca-certificates \
    && rm -rf /var/lib/apt/lists/*

COPY --from=build /opt/venv /opt/venv
COPY --from=build /src/cuvslam_version.txt /opt/cuvslam_version.txt

# The scripts are copied into the image; mount the repository over /opt/vrhealth/scripts
# (docker/run.sh does) to use the current version without rebuilding.
COPY . /opt/vrhealth/scripts
RUN chmod +x /opt/vrhealth/scripts/*.py /opt/vrhealth/scripts/*.sh \
    && rm -rf /opt/vrhealth/scripts/.venv /opt/vrhealth/scripts/__pycache__

ENV PATH=/opt/venv/bin:/opt/vrhealth/scripts:${PATH} \
    PYTHONUNBUFFERED=1 \
    MPLBACKEND=Agg \
    NVIDIA_DRIVER_CAPABILITIES=compute,utility,graphics,display

# Fail the build if a shared library needed by cuVSLAM is missing in the runtime image
# (libcuda.so.1 comes from the host driver at run time and is ignored here)
RUN libs=$(find /opt/venv -path '*cuvslam*' \( -name 'libcuvslam.so' -o -name '*cuvslam*.so' \)) \
    && test -n "$libs" && echo "$libs" \
    && missing=$(for f in $libs; do ldd "$f"; done | grep 'not found' | grep -v 'libcuda.so' | sort -u || true) \
    && if [ -n "$missing" ]; then echo "Missing libraries:"; echo "$missing"; exit 1; fi

# Fail the build if a dependency of the scripts cannot be imported (no GPU needed for these imports)
RUN python3 -c "import numpy, cv2, zstandard, lz4, matplotlib, rerun, rosbags; print('python deps ok')" \
    && test -x /opt/venv/bin/rosbags-convert \
    && cat /opt/cuvslam_version.txt
    
# If all ok, set execute permissions on scripts in /opt/vrhealth/scripts
RUN chmod 755 /opt/vrhealth/scripts/*.py && chmod 755 /opt/vrhealth/scripts/*.sh

WORKDIR /opt/vrhealth/scripts
CMD ["bash"]
