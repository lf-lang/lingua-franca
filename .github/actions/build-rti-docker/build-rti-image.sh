#!/bin/bash
cd org.lflang/src/lib/c/reactor-c/core/federated/RTI
docker build -t rti:rti -f rti.Dockerfile ../../../
