FROM ubuntu:22.04

# Avoid interactive tzdata prompt
ENV DEBIAN_FRONTEND=noninteractive

# Install build tools, NLOPT and Point Cloud Library (PCL)
RUN apt-get update && apt-get install -y --no-install-recommends \
    build-essential cmake pkg-config \
    libnlopt-dev \
    libpcl-dev \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /app
COPY CMakeLists.txt .
COPY racingline_nlopt.cpp .

RUN mkdir -p build \
    && cd build \
    && cmake .. -DCMAKE_BUILD_TYPE=Release \
    && cmake --build . -j$(nproc)

CMD ["/app/build/racingline_nlopt"]
