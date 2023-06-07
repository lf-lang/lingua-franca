#!/bin/bash
cd src/lib/c/reactor-c/core/federated/RTI
docker build -t rti:rti -f rti.Dockerfile ../../../
