#!/bin/bash

export openvr=~/sse/openvr
export steamvr=~/.steam/steam/steamapps/common/SteamVR
export steamruntimesmall=~/sse/steamruntime

export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:\
/usr/lib/:\
/usr/lib32/:\
$openvr/lib/linux32/:\
$openvr/lib/linux64/:\
$steamruntimesmall/:\
$steamvr/bin/linux32/:\
$steamvr/bin/linux64/:\
$steamvr/drivers/lighthouse/bin/linux32/:\
$steamvr/drivers/lighthouse/bin/linux64/:\

