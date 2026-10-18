#!/usr/bin/env bash
set -eu
export JAVA_HOME="${HOME}/orbiedge-fprime-venv/lib/python3.12/site-packages/fprime_jre/runtime"
export PATH="${JAVA_HOME}/bin:${PATH}"
cd /mnt/c/Users/antmi/Downloads/orbiedge-agent
/usr/bin/cmake -S . -B "${HOME}/orbiedge-build" -G "Unix Makefiles" -DCMAKE_BUILD_TYPE=Release \
  -DCMAKE_PREFIX_PATH="${HOME}/opt/libzmq" \
  -DBUILD_TESTING=OFF \
  -DFPRIME_ENABLE_FRAMEWORK_UTS=OFF \
  -DFPRIME_ENABLE_AUTOCODER_UTS=OFF
/usr/bin/cmake --build "${HOME}/orbiedge-build" --target OrbiEdgeApp -j 4
echo BUILD_OK
