FROM ubuntu:16.04
MAINTAINER noah "giallo.ho@gmail.com"
ENV PATH /usr/local/bin:$PATH
RUN apt-get update -y
RUN apt-get update
RUN apt-get install -y python
RUN apt-get install -y python-pip
RUN pip install --upgrade pip
COPY . /robot-env
WORKDIR /robot-env
RUN pip install -r requirements.txt
