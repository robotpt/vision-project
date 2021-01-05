#!/bin/bash

MONGODB_DIR="~/mongodb/vision-project/"
MONGODB_PORT=62345

if [ ! -d "~/mongodb/vision-project/" ]
then
    mkdir ~/mongodb/vision-project/
fi

/bin/bash -c "mongod --dbpath $MONGODB_DIR --port $MONGODB_PORT"

exit 0