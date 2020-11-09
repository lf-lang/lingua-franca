## Logging and analysis of Lingua Franca programs

### Requirements:
  - Lingua Franca program should include printf statements in "EVENT: <Field>: <Value> " format. This stdout including "Field" and "Value" are later parsed for insertion into a time series database. 
  - A time series data base such as InfluxDB (https://docs.influxdata.com/influxdb/v1.8/introduction/install/) is to be installed. 
    - Drop into influx shell using `influx` on terminal and create a database using 'CREATE DATABASE <name>'. I used LF_EVENTS as the database name. 
  - Install Grafana (on OSX, default brew package after an update should be fine). Check you can access the frontend from browser over the default port http://localhost:3000. You change the default port as in here (https://grafana.com/docs/grafana/latest/administration/configuration/)
  - npm install --save influx for influxWrite reactor to work. Check the following in this reactor
    - databaseName: check if this database exists from influx shell using `SHOW DATABASES`
    - measurementName: default "event_times". If you change this, the Grafana query in the following steps will change as well
    - traceFilePath: "/relative/path/to/trace/my_trace"
    - schema: Modify the data schema of your Influx database record based on the `EVENT: <Field>: <Value>` pattern used in the trace file.

### Steps:
 - Run the program and stdout to file with filename (eg:"my_trace")
  Trace files were first generated for based on sender and receiver side programs (SpatReceiver and SpatRecommender). To run these programs, Eclipse Paho MQTT library is required and make sure MQTT broker is running. 
Terminal 1$: ./bin/spatRecommender
Terminal 2$: ./bin/spatReceiver > my_trace

- Then run "influxWrite" reactor once InfluxDB is installed, and service is started
Terminal $: node dist/influxWrite.js 
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

