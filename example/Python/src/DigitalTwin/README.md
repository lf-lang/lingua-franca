# Digital Twin Example

This example shows two federates, one hosted locally and the other hosted on Google Cloud, interacting via an RTI that is also hosted on Google Cloud.

## Before we start

Make sure you have a Google Cloud Platform (GCP) account and project set up. 

Set up an account here:
https://console.cloud.google.com

To set up a GCP project, see here:
https://cloud.google.com/resource-manager/docs/creating-managing-projects

We will be working with the Google Cloud command line interface. Mac users can download it using brew.

```bash
user$ brew install --cask google-cloud-sdk
```

This tutorial also assumes that you have built a docker image for the RTI. You can refer to the README at `org.lflang/src/lib/c/reactor-c/core/federated/RTI/README.md` for instructions related to building the RTI image. 

For clarity purposes, I will use `user$ ` to denote a local terminal, `user@rti-vm ~ $ ` to denote a terminal of the virtual machine instance on the cloud, and `/lingua-franca/core/federated/RTI # ` to denote the terminal inside the container that has the RTI. 


## Instructions

### Hosting the RTI on the cloud

Authenticate google cloud for docker:
```bash
user$ gcloud auth login
user$ gcloud auth configure-docker
```

Export the project ID of the project you want to use for this example:
```bash
user$ gcloud projects list
user$ export PROJECT_ID=YOUR_PROJECT_ID
```

Tag and push the local RTI image to Google cloud:
```bash
user$ docker tag rti:rti gcr.io/$PROJECT_ID/rti
user$ docker push gcr.io/$PROJECT_ID/rti
```

Creating a virtual machine instance using the docker image:
```bash
user$ gcloud compute instances create-with-container rti-vm \
  --coetainer-image=gcr.io/$PROJECT_ID/rti \
  --container-command=ash \
  --container-stdin \
  --container-tty
```

A list of statuses for the newly created virtual machine instance should pop up, which looks like this:
```bash
Created [https://www.googleapis.com/compute/v1/projects/linguafranca-333319/zones/us-central1-a/instances/rti-vm].
NAME    ZONE           MACHINE_TYPE   PREEMPTIBLE  INTERNAL_IP  EXTERNAL_IP     STATUS
rti-vm  us-central1-a  n1-standard-1               10.128.0.7   34.133.143.163  RUNNING
```

Export the external IP of the vm instance:
```bash
user$ export RTI_IP=YOUR_EXTERNAL_IP
```

Set up firewall rules to allow ingress and egress traffic to the VM:
```bash
user$ gcloud compute firewall-rules create rti-firewall-egress --direction=egress --action=allow --rules=all
user$ gcloud compute firewall-rules create rti-firewall-ingress --direction=ingress --action=allow --rules=all
```

Double check that the virtual machine instance is accepting connection:
```bash
user$ ping -c 1 $RTI_IP
```

ssh into the container:
```bash
user$ gcloud compute ssh --project=$PROJECT_ID rti-vm
```

In the VM, find the container ID of the container running the RTI:
```bash
user@rti-vm ~ $ docker container list
CONTAINER ID   IMAGE                                                       COMMAND                  CREATED          STATUS          PORTS     NAMES
9917d5201ed8   gcr.io/linguafranca-333319/rti                              "ash"                    37 minutes ago   Up 2 seconds              klt-rti-vm-fxpe
675602cedb90   gcr.io/stackdriver-agents/stackdriver-logging-agent:1.8.9   "/entrypoint.sh /usr…"   37 minutes ago   Up 37 minutes             stackdriver-logging-agent
```

Attach to the container running the RTI:
```bash
user@rti-vm ~ $ docker attach YOUR_RTI_CONTAINER_ID
```

Run the RTI inside the container:
```bash
/lingua-franca/core/federated/RTI # RTI -i 1 -n 2
```

### Running the key fob federates

Build Digital Twin example locally, after changing the “at” line of the federated reactor:
```bash
user$ lfc DigitalTwin.lf
```

Go to the directory where the generated code is located, which is usually located at `src-gen/DigitalTwin/DigitalTwin`. There should be a docker-compose.yml in that directory. Build the docker images of the two key fobs:
```bash
user$ docker compose build fob twin —no-cache
```

Open two terminals. In the first terminal, run:
```bash
user$ docker compose run --rm fob
```

In the second one, run:
```bash
user$ docker compose run --rm twin
```

Now you should see the key fobs in each terminal syncing with each other through the RTI on the cloud.

### Clean up

Delete firewall rules (type `Y` when prompted):
```bash
user$ gcloud compute firewall-rules delete rti-firewall-egress
user$ gcloud compute firewall-rules delete rti-firewall-ingress
```

Remove the virtual machine instance running the RTI:
```bash
user$ gcloud compute instances delete rti-vm
```

Remove the RTI image from google cloud:
```bash
user$ gcloud container images delete gcr.io/$PROJECT_ID/rti
```

### Conclusion

Congrats! You just federated a LF program using an RTI hosted on the cloud.