#!/bin/sh
#
# Start databroker. No need for systemd, this will
# be restarted by the host machine.
#
# It is harmless to run it again, this will just restart databroker

# Change to the directory where the script resides
cd "$(dirname "$0")"

if [ "$(docker ps -q -f name=databroker)" ]; then
    echo "Databroker container is already running. Trying to stop it..."
    docker stop databroker
fi

# Check if the container exists
if [ "$(docker ps -a -q -f name=databroker)" ]; then
    echo "Removing existing databroker container..."
    docker rm databroker
fi

echo "Starting databroker container..."
docker run  -d --restart=unless-stopped --name=databroker --net=host -v $(pwd)/vss/ew2025demo:/vss  quay.io/eclipse-kuksa/kuksa-databroker:0.5.0 --enable-viss --vss vss/vssEWCombined.json #/vss/vss5.json,/vss/ewoverlay.json
echo "If you are interested in logs, just run 'docker logs -f databroker'"