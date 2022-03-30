# Digital Twin Example: Developer Docs

This document contains detailed descriptions of each command used in the digital twin demo.

### Hosting the RTI on the cloud

Set up firewall rules to allow ingress and egress traffic to VMs:
```bash
user$ gcloud compute firewall-rules create rti-firewall-egress --direction=egress --action=allow --rules=all
user$ gcloud compute firewall-rules create rti-firewall-ingress --direction=ingress --action=allow --rules=all
```

Create a VM for the RTI:
```bash
user$ gcloud compute instances create-with-container rti-vm \
  --container-image=lflang/rti:rti \
  --container-command=RTI \
  --container-arg="-i" \
  --container-arg=1 \
  --container-arg="-n" \
  --container-arg=2
```

A list of statuses for the newly created virtual machine instance should pop up, which looks like this:
```bash
Created [https://www.googleapis.com/compute/v1/projects/linguafranca-333319/zones/us-central1-a/instances/rti-vm].
NAME    ZONE           MACHINE_TYPE   PREEMPTIBLE  INTERNAL_IP  EXTERNAL_IP     STATUS
rti-vm  us-central1-a  n1-standard-1               10.128.0.7   34.133.143.163  RUNNING
```

Export the `EXTERNAL_IP` of the RTI:
```bash
RTI_IP=`gcloud compute instances list | grep 'rti-vm' | awk '{print $5}'`
```

### Running the digital twin

Build `KeyFobDemo.lf` locally:
```bash
user$ lfc KeyFobDemo.lf
```

Go to the directory where the generated code is located, which is usually located at `src-gen/DigitalTwin/KeyFob/KeyFobDemo`. There should be a docker-compose.yml in that directory. Build the docker images of the two key fobs:
```bash
user$ docker compose build fob twin --no-cache
```

Tag and push the digital twin's docker image to the cloud:
```bash
user$ docker tag keyfobdemo_twin gcr.io/$PROJECT_ID/twin
user$ docker push gcr.io/$PROJECT_ID/twin
```

Create a VM for the digital twin, passing the address of the RTI:
```bash
gcloud compute instances create-with-container twin-vm \
  --container-image=gcr.io/$PROJECT_ID/twin \
  --container-arg="-i" \
  --container-arg=1 \
  --container-arg="--rti" \
  --container-arg=$RTI_IP \
  --container-stdin \
  --container-tty
```

ssh into the digital twin:
```bash
user$ gcloud compute ssh --project=$PROJECT_ID twin-vm
```

Find the container ID of the digital twin:
```bash
user@twin-vm ~ $ docker container list
```

Attach the digital twin container using the container ID from the previous step:
```bash
user@twin-vm ~ $ docker container attach CONTAINER_ID
```

### Running the local key fob

Open another terminal in the directory where the `docker-compose.yml` is located. Pass in the address of the RTI. Run:
```bash
user$ docker compose run --rm fob -i 1 --rti $RTI_IP
```

Now you should see the key fobs in each terminal syncing with each other through the RTI on the cloud.

### Clean up

Remove the RTI VM:
```bash
user$ gcloud compute instances delete rti-vm --quiet
```

Remove the digital twin VM:
```bash
user$ gcloud compute instances delete twin-vm --quiet
```

Remove the digital twin image from google cloud:
```bash
user$ gcloud container images delete gcr.io/$PROJECT_ID/twin --quiet
```

Delete firewall rules:
```bash
user$ gcloud compute firewall-rules delete rti-firewall-egress --quiet
user$ gcloud compute firewall-rules delete rti-firewall-ingress --quiet
```