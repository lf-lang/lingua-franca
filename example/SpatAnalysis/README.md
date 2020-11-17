## Logging and analysis of Lingua Franca programs

This example demonstrates logging and analysis of the timing of publish-and-subscribe messages sent between Lingua Franca programs using MQTT.

### Requirements:
This example has quite a few dependencies that must be installed and run:
  - [MQTT C Client library](https://github.com/eclipse/paho.mqtt.c) (used by the MQTT publisher and subscriber reactors)
  - MQTT broker, such as [mosquitto](https://mosquitto.org). This needs to be running after installation.
  - [InfluxDB](https://docs.influxdata.com/influxdb/v1.8/introduction/install/) , a time-series database. This needs to be started after installation:
    - Start `influxd` in the background.
    - Drop into influx shell using `influx` on terminal and create a database using 'CREATE DATABASE <name>'. I used LF_EVENTS as the database name. 
    - databaseName: check if this database exists from influx shell using `SHOW DATABASES`
  - [Grafana](https://grafana.com/docs/grafana/latest/), a browser-based data visualizer. This too needs to be started after installation (e.g. using `brew services start grafana` on MacOS).
    - Check that you can access the frontend from browser over the default port http://localhost:3000. You change the default port as in here (https://grafana.com/docs/grafana/latest/administration/configuration/)
  - [InfluxBD client for node](https://github.com/node-influx/node-influx): `npm install --save influx`
      	- This is needed by the `InfluxWrite` reactor, which reads the traces and writes them to the InfluxDB database.

  Check the following in this reactor
    - measurementName: default "event_times". If you change this, the Grafana query in the following steps will change as well
    - traceFilePath: "/relative/path/to/trace/my_trace"
    - schema: Modify the data schema of your Influx database record based on the `EVENT: <Field>: <Value>` pattern used in the trace file.

### Steps:
 - Run the program to create a trace file (eg:"my_trace"). The publisher and the subscriber need to be run separately:
     - Terminal 1$: `lfc spatRecommender.lf; bin/spatRecommender`
     - Terminal 2$: `lfc spatReceiver.lf; bin/spatReceiver > my_trace`
- Then run the "influxWrite" reactor to route the trace data from the file to InfluxDB:
  - Terminal $: `lfc influxWrite.js; node influxWrite/dist/influxWrite.js`
  - **Note**: This reactor has a number of variables defined in it that have to match what you did above:
    - `databaseName`: This is `LF_EVENTS` above.
    - `measurementName`: This is `event_times` to extract those events.
    - `traceFilePath`: This is `../../my_trace` above (sadly, it is relative to the `dist` directory).
    - `schema`: This needs to match how your events are written to the trace file (see below).
- Configure Grafana dashboard to include influxDS as datasource (https://grafana.com/docs/grafana/latest/datasources/influxdb/)
- Create a new panel, select the database name in query tab. We create two queries for physical and logical times on the measurement named "event_times" as below:

B
FROM default event_times WHERE
SELECT field (logical_latency) math (/1000000000)
GROUP BY
FORMAT AS Time series
ALIAS BY

A
FROM default event_times WHERE
SELECT field (physical_latency) math (/1000000000)
GROUP BY
FORMAT AS Time series
ALIAS BY

Under panel settings, customize axes, legends

### How it Works:
  - Lingua Franca program should include printf statements in "EVENT: <Field>: <Value> " format. This stdout including "Field" and "Value" are later parsed for insertion into a time series database. 
