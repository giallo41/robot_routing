FROM python:3.6
MAINTAINER noah "giallo.ho@gmail.com"
ADD . /robot-env
WORKDIR /robot-env
RUN pip install -r requirements.txt
