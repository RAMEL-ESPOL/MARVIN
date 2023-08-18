FROM yahboomtechnology/ros-foxy:3.5.3
WORKDIR /root/marvin
COPY . .
RUN colcon build
