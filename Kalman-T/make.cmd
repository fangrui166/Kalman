@echo off
rm -rf a.exe data.txt
gcc Kalman_T.c
a.exe > data.txt
python Figure.py