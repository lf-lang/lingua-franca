#!/bin/bash
if ! command -v gcloud &> /dev/null
then
    echo "ERROR: gcloud could not be found"
    exit
fi

if [ -z ${PROJECT_ID+x} ]; 
then
    echo "ERROR: environment variable PROJECT_ID is not set"
    exit
fi

set -e
gcloud compute instances delete rti-vm --quiet
gcloud compute instances delete twin-vm --quiet
gcloud container images delete gcr.io/$PROJECT_ID/twin --quiet
gcloud compute firewall-rules delete rti-firewall-egress --quiet
gcloud compute firewall-rules delete rti-firewall-ingress --quiet
unset RTI_IP
