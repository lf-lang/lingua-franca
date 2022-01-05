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

lfc DigitalTwin.lf

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

sleep 3

gcloud compute ssh --project=$PROJECT_ID twin-vm
