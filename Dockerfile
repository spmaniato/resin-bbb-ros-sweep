FROM resin/beaglebone-black-buildpack-deps

COPY . /usr/src/app

WORKDIR /usr/src/app
