# Deepstream 5.0 demo applications

## deepstream-test4 

- First of all, the expected input file (that h264 elementary stream) should be a file with .h264 extension (which you may found in /opt/nvidia/deepstream/deepstream-5.0/samples/streams ;) )
- In order to run this demo you should prepare your server which consists of following these steps:
    - install kafka from [here](https://kafka.apache.org/quickstart)
    - following the same steps from the above link, in order to start the kafka environment, you should run:
        - `bin/zookeeper-server-start.sh config/zookeeper.properties` # in a terminal
        - `bin/kafka-server-start.sh config/server.properties` # in another terminal
        - `bin/kafka-topics.sh --create --topic quickstart-events --bootstrap-server localhost:9092` # in another terminal
        - `bin/kafka-console-consumer.sh --topic quickstart-events --from-beginning --bootstrap-server localhost:9092` # in the same or other terminal, for reading the messages
- On the client side (aka AGX) you should do run:
    - add the server pc name and ip in /etc/hosts like this: <ip_address>    rambo-pc
    - `./deepstream-test4-app -i /opt/nvidia/deepstream/deepstream-5.0/samples/streams/sample_720p.h264 -p /opt/nvidia/deepstream/deepstream-5.0/lib/libnvds_kafka_proto.so --conn-str="<ip_address>;9092;quickstart-events" -s 0`
