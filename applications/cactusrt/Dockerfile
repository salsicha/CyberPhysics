# cactus-rt real-time demo: builds the cactus-rt library with its
# simple_example (a 1 kHz SCHED_FIFO loop, the default command) and
# tracing_example (writes Perfetto traces of the loop's wakeup latency).

FROM ubuntu:noble AS build

ENV LANG=C.UTF-8 \
    LC_ALL=C.UTF-8 \
    TERM=xterm \
    DEBIAN_FRONTEND=noninteractive

SHELL ["/bin/bash","-c"]

# protobuf: cactus-rt's tracing support (on by default) is built from protos;
# libgtest-dev: the library's public headers include gtest/gtest_prod.h
RUN apt-get update && apt-get install -q -y --no-install-recommends \
      build-essential ca-certificates cmake git libgtest-dev \
      libprotobuf-dev protobuf-compiler vim && \
    rm -rf /var/lib/apt/lists/*

# cactus-rt has no release tags: master as of 2024-11-25. Its CMake fetches
# its own dependencies pinned (quill at a fixed commit, readerwriterqueue v1.0.6).
ARG CACTUS_RT_COMMIT=1ac871c818d4698684461e9fdbb6555c59cf549b

RUN git init -q /cactus-rt && \
    git -C /cactus-rt fetch -q --depth 1 \
      https://github.com/cactusdynamics/cactus-rt.git ${CACTUS_RT_COMMIT} && \
    git -C /cactus-rt checkout -q FETCH_HEAD

WORKDIR /cactus-rt

# Same flags as upstream's `make release`. BUILD_TESTING=OFF because CTest
# turns it on by default and the tests also need google-benchmark.
RUN cmake -B build/release \
      -DCMAKE_BUILD_TYPE=RelWithDebInfo \
      -DBUILD_TESTING=OFF \
      -DENABLE_EXAMPLES=ON && \
    cmake --build build/release -j "$(nproc)" \
      --target rt_simple_example rt_tracing_example && \
    install -m 755 \
      build/release/examples/simple_example/rt_simple_example \
      build/release/examples/tracing_example/rt_tracing_example \
      /usr/local/bin/

# tracing_example writes build/data{1,2}.perfetto relative to its working dir
RUN mkdir -p /workspace/build

COPY entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh


#####################################################################

FROM scratch

ENV LANG=C.UTF-8 \
    LC_ALL=C.UTF-8 \
    TERM=xterm \
    DEBIAN_FRONTEND=noninteractive

COPY --from=build / /

WORKDIR /workspace

ENTRYPOINT ["/entrypoint.sh"]
# Runs until SIGINT/SIGTERM (docker stop, Ctrl-C), then stops its thread
CMD ["rt_simple_example"]
