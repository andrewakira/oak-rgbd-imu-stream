## Introduction


## Docker setup
1. Building image
    ```shell=
    sudo docker build --network=host -t oak -f dockerFile .
    ```
2. Run Container
    ```shell=
    docker run -it --rm \
      --net=host \
      --ipc=host \
      -e DISPLAY=$DISPLAY \
      -e XAUTHORITY=$XAUTHORITY \
      -v $XAUTHORITY:$XAUTHORITY \
      -v /tmp/.X11-unix:/tmp/.X11-unix \
      -v $(pwd):/workspace \
      oak
    ```
3. compile
    ```shell=
    ./build.sh
    ```
4. run inference
    ```shell=
    ./build/oak_stream
    ```
