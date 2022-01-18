# Deploying a Lingua Franca Server to Google Cloud Platform (GCP)

## Prerequisites
A GCP account with billing enabled is required. You can sign up for free [here](https://cloud.google.com/free).

### Instructions

#### MacOS
1. Go to [Google Cloud Console](https://console.cloud.google.com/). 

2. Click on the search bar at the top and search `Create a project`. <br />
Name your project `lf-example`. 

3. Follow this [link](https://console.cloud.google.com/flows/enableapi?apiid=artifactregistry.googleapis.com,container.googleapis.com) to enable the Artifact Registry and Google Kubernetes Engine APIs. You need to enable billing on your GCP account or this step will fail. See [here](https://cloud.google.com/billing/docs/how-to/modify-project#confirm_billing_is_enabled_on_a_project) on how to check if billing is enabled on your account. Once this step is done, you can close Cloud Console.

4. On your local machine, install google cloud sdk using brew:  <br /> 
`brew install --cask google-cloud-sdk`


5. Log into gcloud using `gcloud init` <br />
Type `y` when it asks you to log in. <br />
A browser tab to the Google login page should spawn. Log in.

6. After logging in, the sdk would ask to select or create a project. Select the project you just created in step 2. If it asks you `Do you want to configure a default Compute Region and Zone? (Y/n)?`, type `n`. 

7. Check the project id of the project you just created: <br />
`gcloud projects list`<br />
Export its Project ID:<br /> 
`export PROJECT_ID=your_project_id`

8. Set the compute engine region: <br />
`gcloud config set compute/region us-central1`.

9. Set the compute engine zone: <br />
`gcloud config set compute/zone us-central1-a`.

10. Set up a repo on the artifact registry. This is where you will push your containerized LF application: <br />
`gcloud artifacts repositories create lf-server-repo --repository-format=docker --location=us-central1 --description="Docker repository"`

11. go to `example/TypeScript` in the lingua-franca repo.

12. Compile the server example: <br />
`lfc src/HTTPServerReactor/HTTPServer.lf`

13. Copy the docker file into the generated source folder: <br />
`cp src/HTTPServerReactor/gcp/Dockerfile src-gen/HTTPServerReactor/HTTPServer/Dockerfile`

14. Cd into the generated source folder: <br /> 
`cd src-gen/src-gen/HTTPServerReactor/HTTPServer/`

15. Build the docker image: <br /> 
`docker build -t us-central1-docker.pkg.dev/${PROJECT_ID}/lf-server-repo/lf-server:v1 .`

16. Configure the Docker command-line tool to authenticate to Artifact Registry: <br />
`gcloud auth configure-docker us-central1-docker.pkg.dev`

17. Push the docker image to gcloud container registry: <br />
`docker push us-central1-docker.pkg.dev/${PROJECT_ID}/lf-server-repo/lf-server:v1`

18. Create a kubernetes cluster with one node: <br />
`gcloud container clusters create lf-cluster --num-nodes=1 --machine-type=e2-micro` <br />
This may take a few minutes to finish. 

19. Create a kubernetes deployment for the lf server: <br />
`kubectl create deployment lf-server --image=us-central1-docker.pkg.dev/${PROJECT_ID}/lf-server-repo/lf-server:v1`

20. Expose the lf server: <br />
`kubectl expose deployment lf-server --name=lf-server-service --type=LoadBalancer --port 80 --target-port 8000`

21. Check the status of the exposed service: <br />
`kubectl get service`<br />
If the external IP of the service is `<pending>`, wait a while and try `kubectl get service` again.

22. When the external IP of the service is ready, open up a browser and go to that address. You should see a `ping count` and a `helloworld` message.

23. Finally, we need to do clean up to avoid incurring charges. Clean up the service: <br />
`kubectl delete service lf-server-service`


24. Clean up the cluster:<br />
`gcloud container clusters delete lf-cluster --zone us-central1-a` <br />
This might take a couple of minutes.

25. Clean up the container images: <br />
`gcloud artifacts docker images delete us-central1-docker.pkg.dev/${PROJECT_ID}/lf-server-repo/lf-server:v1 --delete-tags --quiet`


Congratulations! You just finished deploying a lingua franca program on Google Cloud.


## References
1. https://cloud.google.com/kubernetes-engine/docs/quickstart
2. https://cloud.google.com/kubernetes-engine/docs/tutorials/hello-app
