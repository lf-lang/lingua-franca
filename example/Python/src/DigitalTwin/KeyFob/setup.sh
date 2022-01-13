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

set -e
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

lfc --rti_host $RTI_IP DigitalTwin.lf

cd ../../src-gen/DigitalTwin/DigitalTwin

docker compose build fob twin --no-cache

docker tag digitaltwin_twin gcr.io/$PROJECT_ID/twin

docker push gcr.io/$PROJECT_ID/twin

gcloud compute instances create-with-container twin-vm \
  --container-image=gcr.io/$PROJECT_ID/twin \
  --container-arg="-i" \
  --container-arg=1 \
  --container-stdin \
  --container-tty
