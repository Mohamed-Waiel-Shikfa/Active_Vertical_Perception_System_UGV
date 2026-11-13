# Same Clearpath Jazzy simulation image as the user's working command, pinned.
ARG CLEARPATH_BASE=ghcr.io/clearpathrobotics/clearpath_docker:jazzy-sim-latest@sha256:0355e4bb50d50dddfdb2416365c4f155e28eb7ff3a9a53de3c3d42864eba0979
FROM ${CLEARPATH_BASE} AS navigation_dependencies

USER root
WORKDIR /opt/greenhouse
COPY config/nav-packages.txt config/nav-packages.lock.json ./config/
# Exact compatible binaries and BehaviorTree.CPP runtime ABI are checked together.
COPY tools/install_nav.py ./tools/install_nav.py
RUN /ros_entrypoint.sh python3 tools/install_nav.py

FROM navigation_dependencies AS native_compiled
COPY requirements-generator.txt ./
# Isolated generator dependencies: do not replace ROS's system NumPy/OpenCV.
RUN python3 -m pip install --disable-pip-version-check --no-cache-dir \
    --only-binary=:all: --target /opt/greenhouse/python -r requirements-generator.txt

COPY native ./native-src
RUN /ros_entrypoint.sh bash -c 'qmllint native-src/*.qml && cmake -S native-src -B /tmp/greenhouse-native-build -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX=/opt/greenhouse/native && cmake --build /tmp/greenhouse-native-build --parallel 2 && cmake --install /tmp/greenhouse-native-build'

FROM native_compiled AS native_runtime
RUN /ros_entrypoint.sh bash -c '/tmp/greenhouse-native-build/gimbal-math-test && QT_QPA_PLATFORM=offscreen QT_QUICK_BACKEND=software /tmp/greenhouse-native-build/qml-contract'

FROM native_runtime
COPY greenhouse ./greenhouse
COPY third_party ./third_party
COPY config ./config
COPY licenses ./licenses
COPY launch ./launch
COPY docker ./docker
COPY tools ./tools
COPY tests ./tests
COPY cad ./cad
COPY self_driving/self_driving_api.py self_driving/run_script.py ./self_driving/
COPY docs ./docs
COPY README.md THIRD_PARTY_NOTICES.md LICENSE run.sh ./

# The container runs as the host user, not root: a host file saved as 0600 would copy in
# unreadable and crash startup (build-time checks run as root and would not notice).
RUN chmod -R a+rX greenhouse third_party config licenses launch docker tools tests cad docs self_driving \
      README.md THIRD_PARTY_NOTICES.md LICENSE run.sh \
    && PYTHONPATH=/opt/greenhouse/python:/opt/greenhouse \
      python3 -m greenhouse.generate --output /opt/greenhouse/generated \
    && /ros_entrypoint.sh bash -c \
      'PYTHONPATH=/opt/greenhouse/python:/opt/greenhouse:$PYTHONPATH python3 -m unittest discover -s tests' \
    && /ros_entrypoint.sh python3 tools/check_image.py \
    && python3 -m greenhouse.mount --config config/actuator.json --output cad/generated \
    && chmod 0755 docker/*.sh tools/*.sh

ENV SETUP_PATH=/etc/clearpath \
    GREENHOUSE_CONFIG=/opt/greenhouse/config/greenhouse.json \
    GREENHOUSE_NAV_CACHE=/tmp/greenhouse-navigation-status.json \
    PYTHONUNBUFFERED=1

# Last, so changing it never invalidates the cached layers above; run.sh compares it on --no-build.
ARG GREENHOUSE_SOURCE=unknown
LABEL org.jackal-greenhouse.source=$GREENHOUSE_SOURCE

USER robot
# Keep the base /ros_entrypoint.sh: it sources ROS and execs this command.
# Replace only the warehouse-specific CMD, not Clearpath's generators or robot.
CMD ["bash", "/opt/greenhouse/docker/start.sh"]
HEALTHCHECK --interval=20s --timeout=12s --start-period=120s --retries=3 \
    CMD ["python3", "/opt/greenhouse/tools/healthcheck.py"]
