#!/bin/sh
. /opt/ros/hydro/setup.sh
id=$(rosrun hokuyo_node getID $1 --)

if [ "$id" = "H1009079" ] ; then
    echo "rear"
elif [ "$id" = "H1205008" ] ; then
    echo "front"
elif [ "$id" = "H1110684" ] ; then
    echo "objects"
fi