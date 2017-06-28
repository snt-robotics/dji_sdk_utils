#!/bin/bash

for file in *.kml; do 
  gpsbabel -i kml -f "$file" -x track,faketime=20120901000001 -o gpx -F "${file}.gpx"
done
