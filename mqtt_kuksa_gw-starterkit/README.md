# mqtt_kuksa_gw

## Build container

In the root of the repo do

```
docker build -t mqtt-gw-image .           
```

## Run

```
docker run -it --rm --net=host mqtt-gw-image            
```

## Cross compile

```
docker buildx build --platform linux/arm64 -t mqtt-gw-image . 

docker save mqtt-gw-image:latest > ~/mqtt-gw-image.tar

scp mqtt-gw-image.tar fio@192.168.88.11:/home/fio 

```


on target host 

```

docker load -i mqtt-gw-image.tar

```