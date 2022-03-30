# ContainerizedFederatedHelloWorld
This is a basic example showing containerized federated execution. 

For more details, see: [Containerized Execution in Lingua Franca](https://github.com/lf-lang/lingua-franca/wiki/Containerized-Execution)

## Run instructions
Put HelloWorld.lf in a src directory.
Then, run:
```bash
lfc HelloWorldContainerized.lf
```

There would be 3 build messages, 1 for the RTI and 2 for the reactors, indicating where the docker file is generated, as well as the instruction to build the docker file. 

A message would look something like this: 
```
Dockerfile for <something> written to <some_directory>/<name_of_dockerfile>
#####################################
To build the docker image, use:
   
    docker build -t <some_name> -f <some_directory>/<name_of_dockerfile> <some_path>

#####################################
```

If you cannot find the build message for the RTI, try a clean build using:
```bash
lfc --clean HelloWorld.lf
```

Then, use the printed commands to build the 3 docker images. 

Set up a docker network using 
```bash
docker network create lf
```

Open 3 terminals, 1 for the RTI and 1 for each reactor.

Run the RTI:
```bash
docker run --rm -it --network=lf --name=rti rti -i 1 -n 2
```

Run the two reactors:
```bash
docker run --rm -it --network=lf helloworldcontainerized_source -i 1
```
```bash
docker run --rm -it --network=lf helloworldcontainerized_print -i 1
```