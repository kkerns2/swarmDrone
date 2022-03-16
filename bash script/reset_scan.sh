#!/bin/bash

./drone1_reset.sh & PID=$!

sleep 1 

kill $PID 

./drone2_reset.sh & PID=$!

sleep 1 

kill $PID 

./drone3_reset.sh & PID=$!

sleep 1 

kill $PID 

./drone4_reset.sh & PID=$!

sleep 1 

kill $PID 
