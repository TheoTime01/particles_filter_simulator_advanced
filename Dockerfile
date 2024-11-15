
FROM python:3.10.15-bookworm
RUN apt update
RUN apt install -y ros-nav-msgs 
RUN python3 --version

RUN mkdir /app
#Install rangelib
WORKDIR /app
RUN pip3 install Cython numpy
RUN git clone https://gitlab.inria.fr/socialnav/multisoc/range_libc.git
WORKDIR /app/range_libc/pywrapper
## Disable ROS dependencies
RUN sed -i "s|USE_ROS_MAP = True|USE_ROS_MAP = False|g" RangeLibc.pyx
RUN python3 setup.py install

#Install particles_filter_simulator_student
WORKDIR /app
RUN mkdir particles_filter_simulator_student
WORKDIR /app/particles_filter_simulator_student

COPY ./ /app/particles_filter_simulator_student/
RUN ls -al
#RUN git clone https://gitlab.com/js-ros-training/particles_filter_simulator_student.git

RUN pip install -r requirements.txt
## Remove parent entry point
ENTRYPOINT []
