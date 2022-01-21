#!/bin/bash
if ! command -v gcloud &> /dev/null
then
    echo "ERROR: gcloud could not be found"
    exit
fi

if ! command -v lfc &> /dev/null
then
    echo "ERROR: lfc could not be found"
    exit
fi

if ! command -v docker &> /dev/null
then
    echo "ERROR: docker could not be found"
    exit
fi

if [ -z ${PROJECT_ID+x} ]; 
then
    echo "ERROR: environment variable PROJECT_ID is not set"
    exit
fi

if [[ "$OSTYPE" == "linux-gnu"* ]]; then
    docker_compose="docker-compose"
else 
    docker_compose="docker compose"
fi

set -e
gcloud config set compute/zone us-central1-c
gcloud compute firewall-rules create rti-firewall-egress --direction=egress --action=allow --rules=all
gcloud compute firewall-rules create rti-firewall-ingress --direction=ingress --action=allow --rules=all

gcloud compute instances create-with-container rti-vm \
  --container-image=lflang/rti:rti \
  --container-command=RTI \
  --container-arg="-i" \
  --container-arg=1 \
  --container-arg="-n" \
  --container-arg=2

RTI_IP=`gcloud compute instances list | grep 'rti-vm' | awk '{print $5}'`

if [ -z "$RTI_IP" ]
then
      echo "ERROR: IP address of the RTI is empty or not set. Did you spawn the RTI on the cloud?".
      exit
fi

lfc KeyFobDemo.lf

cd ../../../src-gen/DigitalTwin/KeyFob/KeyFobDemo

$docker_compose build --no-cache fob twin

docker tag keyfobdemo_twin gcr.io/$PROJECT_ID/twin

docker push gcr.io/$PROJECT_ID/twin

gcloud compute instances create-with-container twin-vm \
  --container-image=gcr.io/$PROJECT_ID/twin \
  --container-arg="-i" \
  --container-arg=1 \
  --container-arg="--rti" \
  --container-arg=$RTI_IP \
  --container-stdin \
  --container-tty
