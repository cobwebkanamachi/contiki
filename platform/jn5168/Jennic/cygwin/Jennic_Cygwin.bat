@echo off

C:
chdir C:\Jennic\cygwin\bin
bash.exe --login -c 'cd /cygdrive/C/Jennic/Application ; exec /bin/bash -rcfile ~/.bashrc'