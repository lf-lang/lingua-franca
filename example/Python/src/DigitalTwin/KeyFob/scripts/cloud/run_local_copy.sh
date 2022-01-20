#!/bin/bash
if ! command -v gcloud &> /dev/null
then
    echo "ERROR: gcloud could not be found"
    exit
fi

if ! command -v docker &> /dev/null
then
    echo "ERROR: docker could not be found"
    exit
fi

RTI_IP=`gcloud compute instances list | grep 'rti-vm' | awk '{print $5}'`

if [ -z "$RTI_IP" ]
then
      echo "ERROR: IP address of the RTI is empty or not set. Did you spawn the RTI on the cloud?".
      exit
fi

if [[ "$OSTYPE" == "linux-gnu"* ]]; then
    docker_compose="docker-compose"
else 
    docker_compose="docker compose"
fi

cd ../../../src-gen/DigitalTwin/KeyFob/KeyFobDemo
$docker_compose run --rm fob -i 1 --rti $RTI_IP
