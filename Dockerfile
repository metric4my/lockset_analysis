FROM ubuntu:20.04 AS build

RUN apt-get update && apt-get install -y wget unzip make python3-pip
RUN wget https://github.com/danmar/cppcheck/archive/refs/tags/1.90.zip -O cppcheck-1.90.zip
RUN unzip cppcheck-1.90.zip

RUN echo `python3 -V`

WORKDIR /cppcheck-1.90
RUN make MATCHCOMPILER=yes FILESDIR=/cppcheck-1.90 HAVE_RULES=no CXXFLAGS="-O2 -DNDEBUG -Wall -Wno-sign-compare -Wno-unused-function"


FROM ubuntu:22.04

COPY --from=build /cppcheck-1.90 /cppcheck-1.90

RUN mkdir roscallbax

COPY src /roscallbax/src
COPY requirements.txt /requirements.txt

COPY data /roscallbax/data
COPY script /roscallbax

RUN apt-get update && apt-get install -y python3-pip
RUN echo `python3 -V`
RUN pip3 install -r requirements.txt

WORKDIR /roscallbax

