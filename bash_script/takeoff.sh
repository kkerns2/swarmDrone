#!/bin/bash

./drone1_takeoff.sh & PID=$!

sleep 1 

kill $PID 

./drone2_takeoff.sh & PID=$!

sleep 1 

kill $PID 

./drone3_takeoff.sh & PID=$!

sleep 1 

kill $PID 

./drone4_takeoff.sh & PID=$!

sleep 1 

kill $PID 
