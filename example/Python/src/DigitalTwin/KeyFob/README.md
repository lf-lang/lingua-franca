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


Then, authenticate google cloud for docker:
```bash
user$ gcloud auth login
user$ gcloud auth configure-docker
```

Additionally, export the project ID of the project you want to use for this example:
```bash
user$ gcloud projects list
user$ export PROJECT_ID=YOUR_PROJECT_ID
```

For clarity purposes, I will use `user$ ` to denote a local terminal, `user@rti-vm ~ $ ` to denote the terminal RTI VM, and `user@twin-vm ~ $ ` to denote the terminal inside the digital twin VM. 


## Instructions

### Setting up the RTI and the digital twin on the cloud

Run the `setup.sh` script to set up the RTI and the digital twin on the cloud:
```bash
user$ ./setup.sh
```

When the script finishes, ssh into the digital twin:
```bash
user$ gcloud compute ssh twin-vm
```

Inside the VM, find the container ID of the digital twin:
```bash
user@twin-vm ~ $ docker container list
```

Attach the digital twin container using the container ID from the previous step:
```bash
user@twin-vm ~ $ docker container attach CONTAINER_ID
```

### Running the local key fob

Open another terminal in the directory where the `docker-compose.yml` is located. You might have to export `$RTI_IP` again. Run:
```bash
user$ docker compose run --rm fob -i 1 --host $RTI_IP
```

Now you should see the key fobs in each terminal syncing with each other through the RTI on the cloud.

### Clean up

Run the clean up script:
```bash
user$ ./cleanup.sh
```

### Conclusion

Congrats! You just federated a LF program using an RTI hosted on the cloud.
