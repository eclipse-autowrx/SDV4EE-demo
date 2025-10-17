FROM python:3.12-slim-bookworm


RUN apt update && apt install nano
RUN pip3 install kuksa-client paho-mqtt
COPY mqtt-kuksa-gw.py ./

CMD [ "/usr/local/bin/python", "mqtt-kuksa-gw.py" ]