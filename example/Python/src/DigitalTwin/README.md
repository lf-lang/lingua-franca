# Digital Twin Example

This example shows two federates, one hosted locally and the other hosted on Google Cloud, interacting via an RTI that is also hosted on Google Cloud.

## Before we start

Make sure you have a Google Cloud Platform (GCP) account and a project set up. 

To set up an account, see here:
https://console.cloud.google.com

To set up a GCP project, see here:
https://cloud.google.com/resource-manager/docs/creating-managing-projects

We will be working with the Google Cloud SDK. Mac users can download it using brew.

```bash
user$ brew install --cask google-cloud-sdk
```

After downloading the Google Cloud SDK, do the following and follow the [guide](
https://cloud.google.com/sdk/docs/quickstart) to finish initializing the SDK:

```bash
user$ gcloud init
```

For clarity purposes, I will use `user$ ` to denote a local terminal, `user@rti-vm ~ $ ` to denote the terminal RTI VM, and `user@twin-vm ~ $ ` to denote the terminal inside the digital twin VM. 


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

### Running the digital twin

Build Digital Twin example locally, after changing the “at” line of the federated reactor:
```bash
user$ lfc DigitalTwin.lf
```

Go to the directory where the generated code is located, which is usually located at `src-gen/DigitalTwin/DigitalTwin`. There should be a docker-compose.yml in that directory. Build the docker images of the two key fobs:
```bash
user$ docker compose build fob twin —no-cache
```

Tag and push the digital twin's docker image to the cloud:
```bash
user$ docker tag digitaltwin_twin gcr.io/$PROJECT_ID/twin
user$ docker push gcr.io/$PROJECT_ID/twin
```

Create a VM for the digital twin:
```bash
user$ gcloud compute instances create-with-container twin-vm \
  --container-image=gcr.io/$PROJECT_ID/twin \
  --container-arg="-i" \
  --container-arg=1 \
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

Open another terminal in the directory where the `docker-compose.yml` is located. Run:
```bash
user$ docker compose run --rm fob
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

Remove the RTI image from google cloud:
```bash
user$ gcloud container images delete gcr.io/$PROJECT_ID/rti --quiet
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

### Conclusion

Congrats! You just federated a LF program using an RTI hosted on the cloud.
