FROM osrf/ros:humble-desktop-full-jammy
# Aggiorna il registro dei pacchetti e installa locales
RUN apt-get update && apt-get install locales
# Genera le localizzazioni
RUN locale-gen en_US en_US.UTF-8
# Imposta le localizzazioni come predefinite
RUN update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
# Imposta la variabile di ambiente LANG
ENV LANG=en_US.UTF-8
# Installa software-properties-common e aggiunge il repository universe
RUN apt-get install -y software-properties-common && \
    add-apt-repository universe
# Aggiorna il registro dei pacchetti e installa curl
RUN apt-get update && apt-get install -y curl unzip nano inetutils-ping
# Scarica la chiave di autenticazione ROS
RUN curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
# Aggiunge il repository ROS 2 al file di configurazione apt
RUN echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | tee /etc/apt/sources.list.d/ros2.list > /dev/null
# Aggiorna il registro dei pacchetti e aggiorna il sistema
#RUN apt-get update && apt-get upgrade -y

# Clona il github
RUN git clone -b node_C  https://github.com/Uzingr/ghjko.git

# Installa ROS 2 desktop e gli strumenti di sviluppo
#RUN apt-get install -y ros-humble-desktop ros-dev-tools
# Configura l'ambiente ROS
RUN echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc

# Sorgente l'ambiente ROS
#RUN /bin/bash -c "source /opt/ros/humble/setup.bash"
RUN cd /ghjko/vicon_sdk && chmod +x sdk_libs_1.12.sh
RUN cd /ghjko && unzip Vicon_SDK_Linux64.zip 
RUN cd /ghjko/vicon_sdk && ./sdk_libs_1.12.sh -i /ghjko/Vicon_SDK_Linux64.zip
RUN git clone -b release/1.14 https://github.com/PX4/px4_msgs.git /ghjko/vicon_tracker_ros/src/px4_msgs

# Costruici px4_msgs
RUN /bin/bash -c "source /opt/ros/humble/setup.bash; cd ghjko/vicon_tracker_ros/src/px4_msgs; colcon build --symlink-install"
#Imposta il CMAKE_PREFIX_PATH per px4_msgs
ENV CMAKE_PREFIX_PATH="/ghjko/vicon_tracker_ros/src_px4_msgs/install:$CMAKE_PREFIX_PATH"

#RUN cd /ghjko/vicon_tracker_ros && /bin/bash -c "cmake --trace-expand | grep px4_msgs"
RUN cd /ghjko/vicon_tracker_ros && /bin/bash -c  "source /opt/ros/humble/setup.bash; source /ghjko/vicon_tracker_ros/src/px4_msgs/install/setup.bash; colcon build --symlink-install --base-paths /ghjko/vicon_tracker_ros"
RUN echo "source /ghjko/vicon_tracker_ros/install/setup.bash" >> ~/.bashrc
RUN /bin/bash -c "source /ghjko/vicon_tracker_ros/install/setup.bash"
#Avvia la shell di bash all'interno del container
CMD ["/bin/bash"]








#FROM osrf/ros:humble-desktop-full-jammy
# Aggiorna il registro dei pacchetti e installa locales
#RUN apt-get update && apt-get install locales
# Genera le localizzazioni
#RUN locale-gen en_US en_US.UTF-8
# Imposta le localizzazioni come predefinite
#RUN update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
# Imposta la variabile di ambiente LANG
#ENV LANG=en_US.UTF-8
# Installa software-properties-common e aggiunge il repository universe
#RUN apt-get install -y software-properties-common && \
#    add-apt-repository universe
# Aggiorna il registro dei pacchetti e installa curl
#RUN apt-get update && apt-get install -y curl unzip nano inetutils-ping
# Scarica la chiave di autenticazione ROS
#RUN curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
# Aggiunge il repository ROS 2 al file di configurazione apt
#RUN echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | tee /etc/apt/sources.list.d/ros2.list > /dev/null
# Aggiorna il registro dei pacchetti e aggiorna il sistema
#RUN apt-get update && apt-get upgrade -y
# Clona il github
#RUN git clone https://github.com/Uzingr/ghjko.git
# Installa ROS 2 desktop e gli strumenti di sviluppo
#RUN apt-get install -y ros-humble-desktop ros-dev-tools
# Configura l'ambiente ROS
#RUN echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
# Sorgente l'ambiente ROS
#RUN /bin/bash -c "source /opt/ros/humble/setup.bash"
#RUN cd /ghjko/vicon_sdk && chmod +x sdk_libs_1.12.sh
#RUN cd /ghjko && unzip Vicon_SDK_Linux64.zip 
#RUN cd /ghjko/vicon_sdk && ./sdk_libs_1.12.sh -i /ghjko/Vicon_SDK_Linux64.zip
#RUN cd /ghjko/vicon_tracker_ros && /bin/bash -c "source /opt/ros/humble/setup.bash; colcon build --symlink-install"
#RUN echo "source /ghjko/vicon_tracker_ros/install/setup.bash" >> ~/.bashrc
#RUN /bin/bash -c "source /ghjko/vicon_tracker_ros/install/setup.bash"
#Avvia la shell di bash all'interno del container
#CMD ["/bin/bash"]
#RUN chmod 777 -R .
#RUN cd /ros2-vicon-receiver/vicon_receiver && colcon build --symlink-install
#CMD bash
