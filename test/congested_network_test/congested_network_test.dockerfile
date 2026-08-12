# Base image pulled through Google's public mirror of Docker Hub rather than Docker Hub
# itself. Docker Hub rate-limits UNAUTHENTICATED pulls per source IP over an hourly window and
# answers a request over the limit with an immediate HTTP 429 ("toomanyrequests: You have
# reached your unauthenticated pull rate limit"). That is not a transient fault: retrying
# cannot outlast an hourly window, so a build retry loop only wastes time and muddies the log.
# Self-hosted runners share one egress IP across every concurrent job in the organisation,
# which is exactly the shape that hits the limit.
#
# mirror.gcr.io serves byte-identical Docker Hub content (it is a pull-through cache, so
# library/ubuntu here IS ubuntu there) and is not subject to Docker Hub's limits, and it needs
# no credentials — so this works out of the box. Override BASE_IMAGE to point at a registry
# the host is authenticated against (or back at Docker Hub) if that is ever preferable.
ARG BASE_IMAGE=mirror.gcr.io/library/ubuntu:22.04
FROM ${BASE_IMAGE}

ENV DEBIAN_FRONTEND=noninteractive

# Start with install_dependencies.sh prior to everything else to avoid reinstalling dependencies on any change in provizio_dds
COPY ./install_dependencies.sh /opt
RUN /opt/install_dependencies.sh ON
RUN apt install -y iproute2 iputils-ping net-tools socat

COPY . /opt/provizio_dds
SHELL ["/bin/bash", "-eo", "pipefail", "-c"]
RUN python3 -m pip install -v /opt/provizio_dds 2>&1 | tee /tmp/pip_install.log && \
    grep -q "Bin cache located and will be used" /tmp/pip_install.log && \
    rm -f /tmp/pip_install.log

ARG SERVICE
ARG NETWORK_DELAY
ARG PACKETS_LOSS
ARG NETWORK_RATE
ARG XML_PROFILE=""
ENV SERVICE=$SERVICE
ENV NETWORK_DELAY=$NETWORK_DELAY
ENV PACKETS_LOSS=$PACKETS_LOSS
ENV NETWORK_RATE=$NETWORK_RATE
ENV XML_PROFILE=$XML_PROFILE

CMD ["/bin/bash", "-c", "if [ -f ${XML_PROFILE} ]; then export FASTDDS_DEFAULT_PROFILES_FILE=${XML_PROFILE}; fi; tc qdisc add dev eth0 root netem delay ${NETWORK_DELAY} loss ${PACKETS_LOSS} rate ${NETWORK_RATE} && python3 /opt/provizio_dds/test/congested_network_test/congested_network_${SERVICE}.py"]
