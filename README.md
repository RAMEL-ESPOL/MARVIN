# marvin
El siguiente repositorio contiene todos los paquetes necesarios para la instalacion y uso del simulador del vehiculo M.A.R.V.I.N

docker run -it --device=/dev/myserial --device=/dev/rplidar --device=/dev/video0 --device=/dev/video1  --device=/dev/video2 --device=/dev/video3 --device=/dev/video4 --env="DISPLAY" --env="QT_X11_NO_MITSHM=1" -v /tmp/.X11-unix:/tmp/.X11-unix marvin:01 /bin/bash  <br />

## instrucciones para descargar librealsense  <br />
sudo mkdir -p /etc/apt/keyrings <br />
curl -sSf https://librealsense.intel.com/Debian/librealsense.pgp | sudo tee /etc/apt/keyrings/librealsense.pgp > /dev/null <br />
sudo apt-get install apt-transport-https <br />
echo "deb [signed-by=/etc/apt/keyrings/librealsense.pgp] https://librealsense.intel.com/Debian/apt-repo `lsb_release -cs` main" | \  <br />
sudo tee /etc/apt/sources.list.d/librealsense.list  <br />
sudo apt-get update  <br />
sudo apt-get install librealsense2*  <br />

E: Failed to fetch https://librealsense.intel.com/Debian/apt-repo/dists/lsb_release/InRelease  403  Forbidden [IP: 108.157.162.27 443]  <br />

