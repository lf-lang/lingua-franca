# Docker file for building the image of the rti
FROM alpine:latest
COPY . /lingua-franca/core
WORKDIR /lingua-franca/core/federated/RTI
RUN set -ex && apk add --no-cache gcc musl-dev cmake make && \
    mkdir container && \
    cd container && \
    cmake ../ && \
    make && \
    make install

# Use ENTRYPOINT not CMD so that command-line arguments go through
ENTRYPOINT ["RTI"]
