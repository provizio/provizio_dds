FROM ubuntu:22.04

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
