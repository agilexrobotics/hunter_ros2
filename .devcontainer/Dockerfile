FROM lcas.lincoln.ac.uk/lcas/ros:humble as base

ENV DEBIAN_FRONTEND=noninteractive

RUN apt-get update \
    && apt-get install -qq -y --no-install-recommends \
    git \
    python3-pip \
    python3-rosdep


# get the source tree and analyse it for its package.xml only
FROM base as sourcefilter
COPY . /tmp/src
# remove everything that isn't package.xml
RUN find /tmp/src -type f \! -name "package.xml" -print | xargs rm -rf

# install all dependencies listed in the package.xml
FROM base as depbuilder
# copy the reduced source tree (only package.xml) from previous stage
COPY --from=sourcefilter /tmp/src /tmp/src
RUN rosdep update && apt-get update
RUN cd /tmp/src && rosdep install --from-paths . --ignore-src -r -y && cd && rm -rf /tmp/src

FROM depbuilder as final
# add sudo without password
RUN echo "ALL ALL=(ALL) NOPASSWD: ALL" >> /etc/sudoers

RUN echo "source /opt/ros/humble/setup.bash" >> /etc/bash.bashrc

USER ros
